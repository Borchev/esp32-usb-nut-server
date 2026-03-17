/*
 * WiFi provisioning via SoftAP captive portal.
 *
 * On first boot (no credentials in NVS) or when stored credentials fail,
 * the ESP32 creates an open access point that hosts a simple web
 * page where the user picks a network and enters the password.  Once
 * submitted the AP shuts down and the device connects as a station.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "wifi_prov.h"
#include "led_status.h"

#include <string.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"

static const char *TAG = "wifi-prov";

#define AP_SSID       "ESP32 NUT Server Setup"
#define AP_MAX_CONN   4
#define NVS_NAMESPACE "wifi_creds"
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"
#define SCAN_MAX      20
#define STA_CONNECT_TIMEOUT_MS 15000
#define RESET_HOLD_MS 3000
#define RESET_BUTTON_GPIO CONFIG_NUT_BUTTON_GPIO

static EventGroupHandle_t s_wifi_events;
#define BIT_CONNECTED BIT0
#define BIT_FAIL      BIT1

static esp_netif_t *s_netif_sta;
static esp_netif_t *s_netif_ap;
static httpd_handle_t s_httpd;

/* Scan results kept in module scope for the HTTP handler */
static wifi_ap_record_t s_scan_results[SCAN_MAX];
static uint16_t s_scan_count;

/* Credentials received from the portal */
static char s_prov_ssid[33];
static char s_prov_pass[65];
static bool s_prov_received;

/* ------------------------------------------------------------------ */
/* NVS helpers                                                         */
/* ------------------------------------------------------------------ */

static bool nvs_load_creds(char *ssid, size_t ssid_len,
                           char *pass, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return false;

    bool ok = (nvs_get_str(h, NVS_KEY_SSID, ssid, &ssid_len) == ESP_OK &&
               nvs_get_str(h, NVS_KEY_PASS, pass, &pass_len) == ESP_OK);
    nvs_close(h);
    return ok;
}

static void nvs_save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_SSID, ssid);
    nvs_set_str(h, NVS_KEY_PASS, pass);
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------------------------------------------ */
/* WiFi event handler                                                  */
/* ------------------------------------------------------------------ */

static int s_retry_count;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_retry_count < 3) {
                s_retry_count++;
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(s_wifi_events, BIT_FAIL);
            }
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_events, BIT_CONNECTED);
    }
}

/* ------------------------------------------------------------------ */
/* URL-decode helper                                                   */
/* ------------------------------------------------------------------ */

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(char *dst, const char *src, size_t dst_max)
{
    size_t di = 0;
    while (*src && di < dst_max - 1) {
        if (*src == '%' && src[1] && src[2]) {
            int hi = hex_val(src[1]), lo = hex_val(src[2]);
            if (hi >= 0 && lo >= 0) {
                dst[di++] = (char)((hi << 4) | lo);
                src += 3;
                continue;
            }
        }
        if (*src == '+') {
            dst[di++] = ' ';
            src++;
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

/* Extract a form field value from URL-encoded body */
static bool form_get(const char *body, const char *key,
                     char *val, size_t val_max)
{
    size_t klen = strlen(key);
    const char *p = body;
    while ((p = strstr(p, key)) != NULL) {
        /* Make sure it's at start or after '&' */
        if (p != body && *(p - 1) != '&') { p += klen; continue; }
        if (p[klen] != '=') { p += klen; continue; }
        const char *v = p + klen + 1;
        const char *end = strchr(v, '&');
        size_t vlen = end ? (size_t)(end - v) : strlen(v);
        char encoded[128];
        if (vlen >= sizeof(encoded)) vlen = sizeof(encoded) - 1;
        memcpy(encoded, v, vlen);
        encoded[vlen] = '\0';
        url_decode(val, encoded, val_max);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* HTML page                                                           */
/* ------------------------------------------------------------------ */

static const char PORTAL_HTML_START[] =
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>UPS WiFi Setup</title>"
    "<style>"
    "body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px;background:#f4f4f4;}"
    "h2{color:#333;} "
    ".net{padding:10px 14px;margin:4px 0;background:#fff;border:1px solid #ddd;"
    "border-radius:6px;cursor:pointer;display:flex;justify-content:space-between;}"
    ".net:hover{background:#e8f0fe;}"
    ".net.sel{border-color:#1a73e8;background:#e8f0fe;}"
    "input[type=password]{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;"
    "border:1px solid #ccc;border-radius:4px;font-size:16px;}"
    "button{width:100%;padding:12px;background:#1a73e8;color:#fff;border:none;"
    "border-radius:6px;font-size:16px;cursor:pointer;margin-top:8px;}"
    "button:hover{background:#1557b0;}"
    ".rssi{color:#888;font-size:13px;}"
    "</style></head><body>"
    "<h2>UPS WiFi Setup</h2>"
    "<form method='POST' action='/connect'>"
    "<input type='hidden' name='ssid' id='ssid_field' value=''>"
    "<p>Select a network:</p>";

static const char PORTAL_HTML_END[] =
    "<p>Password:</p>"
    "<input type='password' name='pass' placeholder='WiFi password'>"
    "<button type='submit'>Connect</button>"
    "</form>"
    "<script>"
    "document.querySelectorAll('.net').forEach(function(el){"
    "el.addEventListener('click',function(){"
    "document.querySelectorAll('.net').forEach(function(n){n.classList.remove('sel');});"
    "el.classList.add('sel');"
    "document.getElementById('ssid_field').value=el.dataset.ssid;"
    "});"
    "});"
    "</script></body></html>";

static const char PORTAL_HTML_SUCCESS[] =
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Connecting...</title>"
    "<style>body{font-family:sans-serif;max-width:420px;margin:60px auto;padding:0 16px;text-align:center;}</style>"
    "</head><body>"
    "<h2>Connecting...</h2>"
    "<p>The device is now connecting to the selected network. "
    "This access point will shut down shortly.</p>"
    "</body></html>";

/* ------------------------------------------------------------------ */
/* HTTP handlers                                                       */
/* ------------------------------------------------------------------ */

static void do_scan(void)
{
    wifi_scan_config_t scan_cfg = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 300,
    };
    esp_wifi_scan_start(&scan_cfg, true);
    s_scan_count = SCAN_MAX;
    esp_wifi_scan_get_ap_records(&s_scan_count, s_scan_results);
    ESP_LOGI(TAG, "Scan found %d networks", s_scan_count);
}

static esp_err_t handler_root(httpd_req_t *req)
{
    do_scan();

    httpd_resp_set_type(req, "text/html");

    /* Send start of page */
    httpd_resp_send_chunk(req, PORTAL_HTML_START, sizeof(PORTAL_HTML_START) - 1);

    /* Send network list */
    char buf[256];
    for (int i = 0; i < s_scan_count; i++) {
        const char *ssid = (const char *)s_scan_results[i].ssid;
        if (ssid[0] == '\0') continue; /* skip hidden */

        /* signal strength indicator */
        int rssi = s_scan_results[i].rssi;
        const char *bars = rssi > -50 ? "&#9679;&#9679;&#9679;&#9679;" :
                           rssi > -60 ? "&#9679;&#9679;&#9679;&#9675;" :
                           rssi > -70 ? "&#9679;&#9679;&#9675;&#9675;" :
                                        "&#9679;&#9675;&#9675;&#9675;";

        /* HTML-escape SSID for display, use data attribute for value */
        int n = snprintf(buf, sizeof(buf),
            "<div class='net' data-ssid='%s'>"
            "<span>%s</span><span class='rssi'>%s %ddBm</span></div>",
            ssid, ssid, bars, rssi);
        httpd_resp_send_chunk(req, buf, n);
    }

    /* Send end of page */
    httpd_resp_send_chunk(req, PORTAL_HTML_END, sizeof(PORTAL_HTML_END) - 1);
    httpd_resp_send_chunk(req, NULL, 0); /* finish */
    return ESP_OK;
}

static esp_err_t handler_connect(httpd_req_t *req)
{
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char ssid[33] = {0}, pass[65] = {0};
    if (!form_get(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No SSID selected");
        return ESP_FAIL;
    }
    form_get(body, "pass", pass, sizeof(pass));

    ESP_LOGI(TAG, "Portal: SSID='%s'", ssid);

    strncpy(s_prov_ssid, ssid, sizeof(s_prov_ssid) - 1);
    strncpy(s_prov_pass, pass, sizeof(s_prov_pass) - 1);
    s_prov_received = true;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_HTML_SUCCESS, sizeof(PORTAL_HTML_SUCCESS) - 1);
    return ESP_OK;
}

/* Captive portal redirect: any unknown path -> root */
static esp_err_t handler_redirect(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* AP + HTTP server lifecycle                                          */
/* ------------------------------------------------------------------ */

static void start_ap(void)
{
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = sizeof(AP_SSID) - 1,
            .channel = 1,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started: SSID=\"%s\"", AP_SSID);
}

static void start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = handler_root,
    };
    httpd_uri_t uri_connect = {
        .uri = "/connect", .method = HTTP_POST, .handler = handler_connect,
    };
    /* Wildcard catch-all for captive portal redirect */
    httpd_uri_t uri_catchall = {
        .uri = "/*", .method = HTTP_GET, .handler = handler_redirect,
    };

    httpd_register_uri_handler(s_httpd, &uri_root);
    httpd_register_uri_handler(s_httpd, &uri_connect);
    httpd_register_uri_handler(s_httpd, &uri_catchall);
}

static void stop_ap_and_http(void)
{
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ------------------------------------------------------------------ */
/* STA connect attempt                                                 */
/* ------------------------------------------------------------------ */

static bool try_sta_connect(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, "Connecting to \"%s\"...", ssid);

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password) - 1);

    s_retry_count = 0;
    xEventGroupClearBits(s_wifi_events, BIT_CONNECTED | BIT_FAIL);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                           BIT_CONNECTED | BIT_FAIL,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(STA_CONNECT_TIMEOUT_MS));

    if (bits & BIT_CONNECTED) {
        ESP_LOGI(TAG, "Connected to \"%s\"", ssid);
        return true;
    }

    ESP_LOGW(TAG, "Failed to connect to \"%s\"", ssid);
    return false;
}

/* ------------------------------------------------------------------ */
/* Credential reset via button hold                                    */
/* ------------------------------------------------------------------ */

static void nvs_erase_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

/* Returns true if the button was held for RESET_HOLD_MS at boot. */
static bool check_reset_button(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RESET_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);

    /* Button is active-low (pressed = 0) */
    if (gpio_get_level(RESET_BUTTON_GPIO) != 0)
        return false;

    ESP_LOGW(TAG, "Button held - hold for %d s to reset WiFi credentials...",
             RESET_HOLD_MS / 1000);

    int elapsed = 0;
    while (elapsed < RESET_HOLD_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
        if (gpio_get_level(RESET_BUTTON_GPIO) != 0)
            return false;  /* released early */
    }

    ESP_LOGW(TAG, "WiFi credentials erased!");
    nvs_erase_creds();
    return true;
}

/* ------------------------------------------------------------------ */
/* Runtime button monitor - hold 3 s to wipe creds and reboot          */
/* ------------------------------------------------------------------ */

static void button_monitor_task(void *arg)
{
    int held_ms = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
            held_ms += 100;
            if (held_ms >= RESET_HOLD_MS) {
                ESP_LOGW(TAG, "Button held %d s - erasing WiFi credentials and restarting...",
                         RESET_HOLD_MS / 1000);
                nvs_erase_creds();
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        } else {
            held_ms = 0;
        }
    }
}

static void start_button_monitor(void)
{
    xTaskCreate(button_monitor_task, "btn_mon", 2048, NULL, 3, NULL);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void wifi_prov_start(void)
{
    s_wifi_events = xEventGroupCreate();

    /* Check if user is holding the button to reset WiFi */
    bool force_portal = check_reset_button();

    /* Create default netifs */
    s_netif_sta = esp_netif_create_default_wifi_sta();
    s_netif_ap  = esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    /* Try stored credentials first (unless reset was requested) */
    char ssid[33] = {0}, pass[65] = {0};
    bool have_creds = !force_portal &&
                      nvs_load_creds(ssid, sizeof(ssid), pass, sizeof(pass));

    if (have_creds && ssid[0] != '\0') {
        ESP_LOGI(TAG, "Found stored credentials for \"%s\"", ssid);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        if (try_sta_connect(ssid, pass)) {
            start_button_monitor();
            return; /* success */
        }
        ESP_LOGW(TAG, "Stored credentials failed, starting portal");
        esp_wifi_stop();
    } else {
        ESP_LOGI(TAG, "No stored WiFi credentials");
    }

    /* Start AP + captive portal */
    start_ap();
    start_http();
    led_status_set_disconnected();

    ESP_LOGI(TAG, "Connect to WiFi \"%s\" and open http://192.168.4.1", AP_SSID);

    /* Wait for user to submit credentials */
    while (!s_prov_received) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Give the browser a moment to receive the response */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Shut down portal */
    stop_ap_and_http();

    /* Try connecting with the new credentials */
    if (try_sta_connect(s_prov_ssid, s_prov_pass)) {
        nvs_save_creds(s_prov_ssid, s_prov_pass);
        start_button_monitor();
        return;
    }

    /* If it failed, reboot and try again */
    ESP_LOGE(TAG, "Could not connect with provided credentials, restarting...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}
