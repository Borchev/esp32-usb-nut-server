/*
 * Generic USB-HID UPS NUT Server for ESP32-S3
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

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "wifi_prov.h"
#include "hid_ups.h"
#include "nut_server.h"
#include "led_status.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    led_status_init();

    ESP_LOGI(TAG, "Starting WiFi provisioning...");
    wifi_prov_start();

    hid_ups_init();
    nut_server_start();
}
