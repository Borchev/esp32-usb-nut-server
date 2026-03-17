/*
 * NUT (Network UPS Tools) protocol TCP server - multi-UPS support.
 * NUT protocol version 1.3 (RFC 9271) compliant.
 * Ref: https://networkupstools.org/docs/developer-guide.chunked/ar01s09.html
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

#include "nut_server.h"
#include "hid_ups.h"
#include "led_status.h"

#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "nut-svr";

#define INVALID_SOCK (-1)
#define YIELD_MS      50
#define RX_BUF_SIZE   256
#define TX_BUF_SIZE   8192
#define MAX_CLIENTS   (CONFIG_LWIP_MAX_SOCKETS - 1)

/* ------------------------------------------------------------------ */
/* Per-client session state                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    int  sock;
    bool username_set;
    bool password_set;
    char login_ups[32];     /* UPS name this client sent LOGIN for */
    bool is_primary;
} client_session_t;

/* Per-UPS FSD (Forced Shutdown) flag - indexed by device enumeration */
static bool s_fsd[MAX_UPS_DEVICES];

/* Shared var buffer for list responses (only used from tcp_server_task) */
static nut_var_t s_var_buf[MAX_NUT_VARS];

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static bool starts_with(const char *str, const char *prefix)
{
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

static int try_receive(int sock, char *buf, size_t max)
{
    int len = recv(sock, buf, max - 1, 0);
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK ||
            errno == ECONNRESET)
            return 0;
        if (errno == ENOTCONN)
            return -2;
        ESP_LOGW(TAG, "[sock=%d] recv error %d: %s", sock, errno, strerror(errno));
        return -1;
    }
    if (len > 0) buf[len] = '\0';
    return len;
}

static int send_all(int sock, const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int n = send(sock, data + sent, len - sent, 0);
        if (n < 0) {
            if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            return -1;
        }
        sent += n;
    }
    return (int)sent;
}

static int send_str(int sock, const char *s)
{
    return send_all(sock, s, strlen(s));
}

/* ------------------------------------------------------------------ */
/* UPS name matching                                                   */
/* ------------------------------------------------------------------ */

/* Try to match the start of `s` against a connected UPS name.
 * On success, copies the name into `out` and returns a pointer past
 * the name (skipping one trailing space if present).
 * Returns NULL if no match. */
static const char *match_ups(const char *s, char *out, size_t out_max)
{
    int n = hid_ups_get_device_count();
    for (int i = 0; i < n; i++) {
        const char *name = hid_ups_get_device_name(i);
        if (!name) continue;
        size_t nlen = strlen(name);
        if (strncmp(s, name, nlen) == 0 &&
            (s[nlen] == ' ' || s[nlen] == '\0')) {
            strncpy(out, name, out_max - 1);
            out[out_max - 1] = '\0';
            return s[nlen] == ' ' ? s + nlen + 1 : s + nlen;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Login / FSD helpers                                                 */
/* ------------------------------------------------------------------ */

static int count_logins(const char *ups_name,
                        client_session_t *sessions, int nsessions)
{
    int count = 0;
    for (int i = 0; i < nsessions; i++) {
        if (sessions[i].sock != INVALID_SOCK &&
            strcmp(sessions[i].login_ups, ups_name) == 0)
            count++;
    }
    return count;
}

static int ups_name_to_slot(const char *name)
{
    int n = hid_ups_get_device_count();
    for (int i = 0; i < n; i++) {
        const char *dn = hid_ups_get_device_name(i);
        if (dn && strcmp(dn, name) == 0) return i;
    }
    return -1;
}

static bool is_fsd_set(const char *ups_name)
{
    int slot = ups_name_to_slot(ups_name);
    return slot >= 0 && s_fsd[slot];
}

/* ------------------------------------------------------------------ */
/* Variable and command descriptions                                   */
/* ------------------------------------------------------------------ */

typedef struct { const char *name; const char *desc; } desc_entry_t;

static const desc_entry_t s_var_descs[] = {
    {"ups.status",              "UPS status"},
    {"ups.load",                "Load on UPS (percent)"},
    {"ups.power",               "Current value of apparent power (VA)"},
    {"ups.realpower",           "Current value of real power (Watts)"},
    {"ups.power.nominal",       "Nominal value of apparent power (VA)"},
    {"ups.realpower.nominal",   "Nominal value of real power (Watts)"},
    {"ups.temperature",         "UPS temperature (C)"},
    {"ups.beeper.status",       "UPS beeper status"},
    {"ups.delay.shutdown",      "Interval to wait before shutting down the load (seconds)"},
    {"ups.delay.start",         "Interval to wait before restarting the load (seconds)"},
    {"ups.delay.reboot",        "Interval to wait before rebooting the UPS (seconds)"},
    {"ups.vendorid",            "Vendor ID for the UPS"},
    {"ups.productid",           "Product ID for the UPS"},
    {"input.voltage",           "Input voltage (V)"},
    {"input.voltage.nominal",   "Nominal input voltage (V)"},
    {"input.current",           "Input current (A)"},
    {"input.frequency",         "Input line frequency (Hz)"},
    {"input.frequency.nominal", "Nominal input line frequency (Hz)"},
    {"input.transfer.low",      "Low voltage transfer point (V)"},
    {"input.transfer.high",     "High voltage transfer point (V)"},
    {"input.sensitivity",       "Input power sensitivity"},
    {"output.voltage",          "Output voltage (V)"},
    {"output.voltage.nominal",  "Nominal output voltage (V)"},
    {"output.current",          "Output current (A)"},
    {"output.frequency",        "Output frequency (Hz)"},
    {"output.frequency.nominal","Nominal output frequency (Hz)"},
    {"battery.charge",          "Battery charge (percent)"},
    {"battery.charge.low",      "Remaining battery level when UPS switches to LB (percent)"},
    {"battery.charge.warning",  "Battery level when UPS sends warning (percent)"},
    {"battery.voltage",         "Battery voltage (V)"},
    {"battery.voltage.nominal", "Nominal battery voltage (V)"},
    {"battery.capacity",        "Battery capacity (Ah)"},
    {"battery.capacity.nominal","Nominal battery capacity (Ah)"},
    {"battery.runtime",         "Battery runtime (seconds)"},
    {"battery.runtime.low",     "Remaining battery runtime when UPS switches to LB (seconds)"},
    {"battery.temperature",     "Battery temperature (C)"},
    {"battery.type",            "Battery chemistry"},
    {"battery.mfr.date",        "Battery manufacturing date"},
    {"battery.charger.status",  "Status of the battery charger"},
    {"battery.current",         "Battery current (A)"},
    {"battery.cyclecount",      "Number of battery charge/discharge cycles"},
    {"ups.load.nominal",        "Nominal value of output load (percent)"},
    {"ups.test.result",         "Result of last self test"},
    {"ambient.humidity",        "Ambient relative humidity (percent)"},
    {"device.type",             "Device type"},
    {"device.mfr",              "Device manufacturer"},
    {"device.model",            "Device model"},
    {"device.serial",           "Device serial number"},
    {"driver.name",             "Driver name"},
    {"driver.version",          "Driver version"},
    {"ups.firmware",            "UPS firmware version"},
    {"ups.firmware.aux",        "Auxiliary firmware version"},
    {"battery.date",            "Battery replacement date"},
    {"input.transfer.reason",   "Reason for last transfer to battery"},
    {NULL, NULL},
};

static const desc_entry_t s_cmd_descs[] = {
    {"beeper.toggle",    "Toggle the UPS beeper"},
    {"beeper.enable",    "Enable the UPS beeper"},
    {"beeper.disable",   "Disable the UPS beeper"},
    {"load.off",         "Turn off the load immediately"},
    {"load.on",          "Turn on the load immediately"},
    {"shutdown.return",  "Turn off the load and return when power is back"},
    {"shutdown.stayoff", "Turn off the load and remain off"},
    {"shutdown.stop",              "Stop a running shutdown"},
    {"test.battery.start.quick",   "Start a quick battery test"},
    {"test.battery.start.deep",    "Start a deep battery test"},
    {"test.battery.stop",          "Stop the battery test"},
    {NULL, NULL},
};

static const char *lookup_desc(const desc_entry_t *tbl, const char *name)
{
    for (int i = 0; tbl[i].name; i++)
        if (strcmp(tbl[i].name, name) == 0)
            return tbl[i].desc;
    return "No description available";
}

/* ------------------------------------------------------------------ */
/* Variable type detection                                             */
/* ------------------------------------------------------------------ */

static bool is_string_var(const char *name)
{
    static const char *names[] = {
        "ups.status", "device.type", "device.mfr", "device.model",
        "device.serial", "driver.name", "driver.version",
        "ups.vendorid", "ups.productid", "battery.type",
        "battery.mfr.date", "battery.charger.status",
        "ups.beeper.status", "ups.test.result",
        "input.sensitivity", "ups.firmware", "ups.firmware.aux",
        "battery.date", "input.transfer.reason", NULL,
    };
    for (int i = 0; names[i]; i++)
        if (strcmp(name, names[i]) == 0) return true;
    return false;
}

/* ------------------------------------------------------------------ */
/* NUT response builders                                               */
/* ------------------------------------------------------------------ */

static void build_list_ups(char *buf, size_t max)
{
    int off = snprintf(buf, max, "BEGIN LIST UPS\n");
    int n = hid_ups_get_device_count();
    for (int i = 0; i < n && off < (int)max - 80; i++) {
        const char *name = hid_ups_get_device_name(i);
        if (name)
            off += snprintf(buf + off, max - off,
                            "UPS %s \"USB HID UPS\"\n", name);
    }
    snprintf(buf + off, max - off, "END LIST UPS\n");
}

static void build_list_var(char *buf, size_t max, const char *ups_name)
{
    int n = hid_ups_get_vars(ups_name, s_var_buf, MAX_NUT_VARS);

    int off = snprintf(buf, max, "BEGIN LIST VAR %s\n", ups_name);
    for (int i = 0; i < n && off < (int)max - 80; i++)
        off += snprintf(buf + off, max - off,
                        "VAR %s %s \"%s\"\n",
                        ups_name, s_var_buf[i].name, s_var_buf[i].value);
    snprintf(buf + off, max - off, "END LIST VAR %s\n", ups_name);
}

static void build_get_var(char *buf, size_t max,
                          const char *ups_name, const char *varname)
{
    char val[NUT_VAR_VALUE_LEN];
    if (hid_ups_get_var(ups_name, varname, val, sizeof(val))) {
        /* Append FSD to ups.status if forced shutdown is set */
        if (strcmp(varname, "ups.status") == 0 && is_fsd_set(ups_name)) {
            size_t slen = strlen(val);
            if (slen + 4 < sizeof(val))
                strcat(val, " FSD");
        }
        snprintf(buf, max, "VAR %s %s \"%s\"\n", ups_name, varname, val);
    } else {
        snprintf(buf, max, "ERR VAR-NOT-SUPPORTED\n");
    }
}

static void build_list_cmd(char *buf, size_t max, const char *ups_name)
{
    nut_cmd_t cmds[MAX_NUT_CMDS];
    int n = hid_ups_get_cmdlist(ups_name, cmds, MAX_NUT_CMDS);

    int off = snprintf(buf, max, "BEGIN LIST CMD %s\n", ups_name);
    for (int i = 0; i < n && off < (int)max - 80; i++)
        off += snprintf(buf + off, max - off,
                        "CMD %s %s\n", ups_name, cmds[i].name);
    snprintf(buf + off, max - off, "END LIST CMD %s\n", ups_name);
}

static void build_list_rw(char *buf, size_t max, const char *ups_name)
{
    int n = hid_ups_get_rw_vars(ups_name, s_var_buf, MAX_NUT_VARS);

    int off = snprintf(buf, max, "BEGIN LIST RW %s\n", ups_name);
    for (int i = 0; i < n && off < (int)max - 80; i++)
        off += snprintf(buf + off, max - off,
                        "RW %s %s \"%s\"\n",
                        ups_name, s_var_buf[i].name, s_var_buf[i].value);
    snprintf(buf + off, max - off, "END LIST RW %s\n", ups_name);
}

static void build_list_enum(char *buf, size_t max,
                            const char *ups_name, const char *varname)
{
    int off = snprintf(buf, max, "BEGIN LIST ENUM %s %s\n", ups_name, varname);

    /* ups.beeper.status has enumerated values */
    if (strcmp(varname, "ups.beeper.status") == 0 &&
        hid_ups_is_rw_var(ups_name, varname)) {
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"disabled\"\n", ups_name, varname);
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"enabled\"\n", ups_name, varname);
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"muted\"\n", ups_name, varname);
    }

    /* input.sensitivity has enumerated values (CyberPower) */
    if (strcmp(varname, "input.sensitivity") == 0 &&
        hid_ups_is_rw_var(ups_name, varname)) {
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"low\"\n", ups_name, varname);
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"normal\"\n", ups_name, varname);
        off += snprintf(buf + off, max - off,
                        "ENUM %s %s \"high\"\n", ups_name, varname);
    }

    snprintf(buf + off, max - off, "END LIST ENUM %s %s\n",
             ups_name, varname);
}

static void build_list_range(char *buf, size_t max,
                             const char *ups_name, const char *varname)
{
    /* No range constraints exposed in this implementation */
    snprintf(buf, max,
             "BEGIN LIST RANGE %s %s\nEND LIST RANGE %s %s\n",
             ups_name, varname, ups_name, varname);
}

static void build_list_client(char *buf, size_t max,
                              const char *ups_name,
                              client_session_t *sessions, int nsessions)
{
    int off = snprintf(buf, max, "BEGIN LIST CLIENT %s\n", ups_name);

    for (int i = 0; i < nsessions && off < (int)max - 80; i++) {
        if (sessions[i].sock == INVALID_SOCK ||
            strcmp(sessions[i].login_ups, ups_name) != 0)
            continue;

        struct sockaddr_in addr;
        socklen_t alen = sizeof(addr);
        char ip[32] = "unknown";
        if (getpeername(sessions[i].sock,
                        (struct sockaddr *)&addr, &alen) == 0)
            inet_ntoa_r(addr.sin_addr, ip, sizeof(ip));

        off += snprintf(buf + off, max - off,
                        "CLIENT %s %s\n", ups_name, ip);
    }

    snprintf(buf + off, max - off, "END LIST CLIENT %s\n", ups_name);
}

static void build_get_type(char *buf, size_t max,
                           const char *ups_name, const char *varname)
{
    char val[NUT_VAR_VALUE_LEN];
    if (!hid_ups_get_var(ups_name, varname, val, sizeof(val))) {
        snprintf(buf, max, "ERR VAR-NOT-SUPPORTED\n");
        return;
    }

    bool rw = hid_ups_is_rw_var(ups_name, varname);
    bool str = is_string_var(varname);

    int off = snprintf(buf, max, "TYPE %s %s", ups_name, varname);
    if (rw)  off += snprintf(buf + off, max - off, " RW");
    if (str) off += snprintf(buf + off, max - off, " STRING:%d",
                             NUT_VAR_VALUE_LEN);
    else     off += snprintf(buf + off, max - off, " NUMBER");
    if (rw && (strcmp(varname, "ups.beeper.status") == 0 ||
               strcmp(varname, "input.sensitivity") == 0))
        off += snprintf(buf + off, max - off, " ENUM");
    snprintf(buf + off, max - off, "\n");
}

/* ------------------------------------------------------------------ */
/* Command dispatch                                                    */
/* ------------------------------------------------------------------ */

static void handle_command(int sock, client_session_t *cs,
                           client_session_t *all_sessions, int nsessions,
                           const char *cmd)
{
    static char tx[TX_BUF_SIZE];

    /* Strip trailing newline/CR for matching */
    char clean[RX_BUF_SIZE];
    strncpy(clean, cmd, sizeof(clean) - 1);
    clean[sizeof(clean) - 1] = '\0';
    size_t clen = strlen(clean);
    while (clen > 0 && (clean[clen - 1] == '\n' || clean[clen - 1] == '\r'))
        clean[--clen] = '\0';

    if (clen == 0) return;

    ESP_LOGI(TAG, "[sock=%d] << %s", sock, clean);

    /* ---- USERNAME ---- */
    if (starts_with(clean, "USERNAME ")) {
        if (cs->username_set)
            send_str(sock, "ERR ALREADY-SET-USERNAME\n");
        else {
            cs->username_set = true;
            send_str(sock, "OK\n");
        }
        return;
    }

    /* ---- PASSWORD ---- */
    if (starts_with(clean, "PASSWORD ")) {
        if (cs->password_set)
            send_str(sock, "ERR ALREADY-SET-PASSWORD\n");
        else {
            cs->password_set = true;
            send_str(sock, "OK\n");
        }
        return;
    }

    /* ---- LOGIN <ups> ---- */
    if (starts_with(clean, "LOGIN ")) {
        char name[32];
        if (cs->login_ups[0]) {
            send_str(sock, "ERR ALREADY-LOGGED-IN\n");
        } else if (match_ups(clean + 6, name, sizeof(name))) {
            strncpy(cs->login_ups, name, sizeof(cs->login_ups) - 1);
            cs->login_ups[sizeof(cs->login_ups) - 1] = '\0';
            send_str(sock, "OK\n");
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LOGOUT ---- */
    if (strcmp(clean, "LOGOUT") == 0) {
        cs->login_ups[0] = '\0';
        cs->is_primary = false;
        send_str(sock, "OK Goodbye\n");
        return;
    }

    /* ---- PRIMARY <ups> / MASTER <ups> ---- */
    if (starts_with(clean, "PRIMARY ") || starts_with(clean, "MASTER ")) {
        bool is_master = starts_with(clean, "MASTER ");
        const char *rest = clean + (is_master ? 7 : 8);
        char name[32];
        if (!cs->username_set) {
            send_str(sock, "ERR USERNAME-REQUIRED\n");
        } else if (!cs->password_set) {
            send_str(sock, "ERR PASSWORD-REQUIRED\n");
        } else if (match_ups(rest, name, sizeof(name))) {
            cs->is_primary = true;
            send_str(sock,
                     is_master ? "OK MASTER-GRANTED\n"
                               : "OK PRIMARY-GRANTED\n");
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- FSD <ups> ---- */
    if (starts_with(clean, "FSD ")) {
        char name[32];
        if (!cs->username_set) {
            send_str(sock, "ERR USERNAME-REQUIRED\n");
        } else if (!cs->password_set) {
            send_str(sock, "ERR PASSWORD-REQUIRED\n");
        } else if (!cs->is_primary) {
            send_str(sock, "ERR ACCESS-DENIED\n");
        } else if (match_ups(clean + 4, name, sizeof(name))) {
            int slot = ups_name_to_slot(name);
            if (slot >= 0) s_fsd[slot] = true;
            send_str(sock, "OK FSD-SET\n");
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST UPS ---- */
    if (strcmp(clean, "LIST UPS") == 0) {
        build_list_ups(tx, sizeof(tx));
        send_str(sock, tx);
        return;
    }

    /* ---- LIST VAR <ups> ---- */
    if (starts_with(clean, "LIST VAR ")) {
        char name[32];
        if (match_ups(clean + 9, name, sizeof(name))) {
            build_list_var(tx, sizeof(tx), name);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST CMD <ups> ---- */
    if (starts_with(clean, "LIST CMD ")) {
        char name[32];
        if (match_ups(clean + 9, name, sizeof(name))) {
            build_list_cmd(tx, sizeof(tx), name);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST RW <ups> ---- */
    if (starts_with(clean, "LIST RW ")) {
        char name[32];
        if (match_ups(clean + 8, name, sizeof(name))) {
            build_list_rw(tx, sizeof(tx), name);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST ENUM <ups> <var> ---- */
    if (starts_with(clean, "LIST ENUM ")) {
        char name[32];
        const char *varname = match_ups(clean + 10, name, sizeof(name));
        if (varname && *varname) {
            build_list_enum(tx, sizeof(tx), name, varname);
            send_str(sock, tx);
        } else {
            send_str(sock, varname ? "ERR INVALID-ARGUMENT\n"
                                   : "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST RANGE <ups> <var> ---- */
    if (starts_with(clean, "LIST RANGE ")) {
        char name[32];
        const char *varname = match_ups(clean + 11, name, sizeof(name));
        if (varname && *varname) {
            build_list_range(tx, sizeof(tx), name, varname);
            send_str(sock, tx);
        } else {
            send_str(sock, varname ? "ERR INVALID-ARGUMENT\n"
                                   : "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- LIST CLIENT <ups> ---- */
    if (starts_with(clean, "LIST CLIENT ")) {
        char name[32];
        if (match_ups(clean + 12, name, sizeof(name))) {
            build_list_client(tx, sizeof(tx), name,
                              all_sessions, nsessions);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET VAR <ups> <var> ---- */
    if (starts_with(clean, "GET VAR ")) {
        char name[32];
        const char *varname = match_ups(clean + 8, name, sizeof(name));
        if (varname && *varname) {
            build_get_var(tx, sizeof(tx), name, varname);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET TYPE <ups> <var> ---- */
    if (starts_with(clean, "GET TYPE ")) {
        char name[32];
        const char *varname = match_ups(clean + 9, name, sizeof(name));
        if (varname && *varname) {
            build_get_type(tx, sizeof(tx), name, varname);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET DESC <ups> <var> ---- */
    if (starts_with(clean, "GET DESC ")) {
        char name[32];
        const char *varname = match_ups(clean + 9, name, sizeof(name));
        if (varname && *varname) {
            char val[NUT_VAR_VALUE_LEN];
            if (hid_ups_get_var(name, varname, val, sizeof(val)))
                snprintf(tx, sizeof(tx), "DESC %s %s \"%s\"\n",
                         name, varname, lookup_desc(s_var_descs, varname));
            else
                snprintf(tx, sizeof(tx), "ERR VAR-NOT-SUPPORTED\n");
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET CMDDESC <ups> <cmd> ---- */
    if (starts_with(clean, "GET CMDDESC ")) {
        char name[32];
        const char *cmdname = match_ups(clean + 12, name, sizeof(name));
        if (cmdname && *cmdname) {
            snprintf(tx, sizeof(tx), "CMDDESC %s %s \"%s\"\n",
                     name, cmdname, lookup_desc(s_cmd_descs, cmdname));
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET NUMLOGINS <ups> ---- */
    if (starts_with(clean, "GET NUMLOGINS ")) {
        char name[32];
        if (match_ups(clean + 14, name, sizeof(name))) {
            int n = count_logins(name, all_sessions, nsessions);
            snprintf(tx, sizeof(tx), "NUMLOGINS %s %d\n", name, n);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- GET UPSDESC <ups> ---- */
    if (starts_with(clean, "GET UPSDESC ")) {
        char name[32];
        if (match_ups(clean + 12, name, sizeof(name))) {
            snprintf(tx, sizeof(tx), "UPSDESC %s \"USB HID UPS\"\n", name);
            send_str(sock, tx);
        } else {
            send_str(sock, "ERR UNKNOWN-UPS\n");
        }
        return;
    }

    /* ---- INSTCMD <ups> <cmd> [<param>] ---- */
    if (starts_with(clean, "INSTCMD ")) {
        char name[32];
        const char *rest = match_ups(clean + 8, name, sizeof(name));
        if (!rest) {
            send_str(sock, "ERR UNKNOWN-UPS\n");
            return;
        }
        if (!cs->username_set) {
            send_str(sock, "ERR USERNAME-REQUIRED\n");
            return;
        }
        if (!cs->password_set) {
            send_str(sock, "ERR PASSWORD-REQUIRED\n");
            return;
        }
        /* Extract command name (first word) */
        char cmdname[NUT_CMD_NAME_LEN];
        int i = 0;
        while (*rest && *rest != ' ' && i < (int)sizeof(cmdname) - 1)
            cmdname[i++] = *rest++;
        cmdname[i] = '\0';

        if (hid_ups_instcmd(name, cmdname))
            send_str(sock, "OK\n");
        else
            send_str(sock, "ERR CMD-NOT-SUPPORTED\n");
        return;
    }

    /* ---- SET VAR <ups> <var> "<value>" ---- */
    if (starts_with(clean, "SET VAR ")) {
        char name[32];
        const char *rest = match_ups(clean + 8, name, sizeof(name));
        if (!rest) {
            send_str(sock, "ERR UNKNOWN-UPS\n");
            return;
        }
        if (!cs->username_set) {
            send_str(sock, "ERR USERNAME-REQUIRED\n");
            return;
        }
        if (!cs->password_set) {
            send_str(sock, "ERR PASSWORD-REQUIRED\n");
            return;
        }

        /* Parse: <varname> "<value>" */
        char varname[NUT_VAR_NAME_LEN];
        int i = 0;
        while (*rest && *rest != ' ' && i < (int)sizeof(varname) - 1)
            varname[i++] = *rest++;
        varname[i] = '\0';

        if (*rest == ' ') rest++;
        if (*rest == '"') rest++;

        char value[NUT_VAR_VALUE_LEN];
        i = 0;
        while (*rest && *rest != '"' && i < (int)sizeof(value) - 1)
            value[i++] = *rest++;
        value[i] = '\0';

        if (!hid_ups_is_rw_var(name, varname))
            send_str(sock, "ERR READONLY\n");
        else if (hid_ups_set_var(name, varname, value))
            send_str(sock, "OK\n");
        else
            send_str(sock, "ERR SET-FAILED\n");
        return;
    }

    /* ---- STARTTLS ---- */
    if (strcmp(clean, "STARTTLS") == 0) {
        send_str(sock, "ERR FEATURE-NOT-CONFIGURED\n");
        return;
    }

    /* ---- VER ---- */
    if (strcmp(clean, "VER") == 0) {
        send_str(sock, "esp32-usb-nut-server\n");
        return;
    }

    /* ---- NETVER / PROTVER ---- */
    if (strcmp(clean, "NETVER") == 0 || strcmp(clean, "PROTVER") == 0) {
        send_str(sock, "1.3\n");
        return;
    }

    /* ---- HELP ---- */
    if (strcmp(clean, "HELP") == 0) {
        send_str(sock,
            "Commands: HELP VER NETVER PROTVER STARTTLS\n"
            "  LOGIN LOGOUT USERNAME PASSWORD PRIMARY MASTER FSD\n"
            "  GET VAR TYPE DESC CMDDESC NUMLOGINS UPSDESC\n"
            "  LIST UPS VAR RW CMD ENUM RANGE CLIENT\n"
            "  SET INSTCMD\n");
        return;
    }

    /* ---- Unknown ---- */
    send_str(sock, "ERR UNKNOWN-COMMAND\n");
}

/* ------------------------------------------------------------------ */
/* TCP server task                                                     */
/* ------------------------------------------------------------------ */

static void tcp_server_task(void *arg)
{
    struct addrinfo hints = {.ai_socktype = SOCK_STREAM};
    struct addrinfo *addr_info;

    int res = getaddrinfo(CONFIG_NUT_TCP_BIND_ADDRESS, CONFIG_NUT_TCP_BIND_PORT,
                          &hints, &addr_info);
    if (res != 0 || !addr_info) {
        ESP_LOGE(TAG, "getaddrinfo failed: %d", res);
        vTaskDelete(NULL);
        return;
    }

    int listen_sock = socket(addr_info->ai_family, addr_info->ai_socktype,
                             addr_info->ai_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        goto cleanup;
    }

    int flags = fcntl(listen_sock, F_GETFL);
    fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, addr_info->ai_addr, addr_info->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        goto cleanup;
    }

    if (listen(listen_sock, 2) != 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Listening on %s:%s (NUT protocol v1.3)",
             CONFIG_NUT_TCP_BIND_ADDRESS, CONFIG_NUT_TCP_BIND_PORT);

    /* Per-client session state */
    client_session_t sessions[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        memset(&sessions[i], 0, sizeof(client_session_t));
        sessions[i].sock = INVALID_SOCK;
    }
    memset(s_fsd, 0, sizeof(s_fsd));

    while (1) {
        /* Accept new connections */
        struct sockaddr_storage src;
        socklen_t src_len = sizeof(src);

        int free_idx = -1;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (sessions[i].sock == INVALID_SOCK) { free_idx = i; break; }
        }

        if (free_idx >= 0) {
            int ns = accept(listen_sock, (struct sockaddr *)&src, &src_len);
            if (ns >= 0) {
                memset(&sessions[free_idx], 0, sizeof(client_session_t));
                sessions[free_idx].sock = ns;
                flags = fcntl(ns, F_GETFL);
                fcntl(ns, F_SETFL, flags | O_NONBLOCK);
                led_status_set_client();
                ESP_LOGI(TAG, "[sock=%d] connected", ns);
            }
        }

        /* Serve connected clients */
        static char rx[RX_BUF_SIZE];
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (sessions[i].sock == INVALID_SOCK) continue;

            int len = try_receive(sessions[i].sock, rx, sizeof(rx));
            if (len < 0) {
                ESP_LOGI(TAG, "[sock=%d] disconnected", sessions[i].sock);
                close(sessions[i].sock);
                memset(&sessions[i], 0, sizeof(client_session_t));
                sessions[i].sock = INVALID_SOCK;
            } else if (len > 0) {
                /* Process line by line */
                char *saveptr;
                char *line = strtok_r(rx, "\n", &saveptr);
                while (line) {
                    handle_command(sessions[i].sock, &sessions[i],
                                   sessions, MAX_CLIENTS, line);
                    line = strtok_r(NULL, "\n", &saveptr);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(YIELD_MS));
    }

cleanup:
    if (listen_sock >= 0) close(listen_sock);
    freeaddrinfo(addr_info);
    vTaskDelete(NULL);
}

void nut_server_start(void)
{
    xTaskCreate(tcp_server_task, "nut_tcp", 6144, NULL, 5, NULL);
}
