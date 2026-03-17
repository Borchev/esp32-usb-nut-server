/*
 * Generic USB-HID UPS driver - multi-device support.
 *
 * Parses the HID Report Descriptor at connect time to discover which
 * Feature/Input reports carry UPS data, then polls them periodically
 * and exposes the values as NUT variables.
 *
 * Up to MAX_UPS_DEVICES UPS units can be connected simultaneously
 * (e.g. via a USB hub).  Each gets its own NUT name:
 *   slot 0 ->CONFIG_NUT_UPS_NAME          (default "ups")
 *   slot 1 ->CONFIG_NUT_UPS_NAME "-2"     (e.g. "ups-2")
 *   slot 2 ->CONFIG_NUT_UPS_NAME "-3"     ...
 *
 * Uses espressif/usb_host_hid (^1.1.0)
 *   https://github.com/espressif/esp-usb
 *
 * HID Usage Tables referenced:
 *   Power Device Page (0x84)
 *   Battery System Page (0x85)
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

#include "hid_ups.h"
#include "led_status.h"

#include <string.h>
#include <stdio.h>
#include <wchar.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid.h"

static const char *TAG = "hid-ups";

/* ================================================================== */
/* Internal limits                                                     */
/* ================================================================== */

#define MAX_MAPPED_FIELDS    128
#define MAX_COLLECTION_DEPTH 16
#define MAX_USAGES_PER_ITEM  32
#define MAX_REPORT_BUF       64
#define MAX_POLL_REPORTS      32

/* ================================================================== */
/* HID usage context (from collection hierarchy)                       */
/* ================================================================== */

typedef enum {
    CTX_NONE = 0,
    CTX_UPS,
    CTX_INPUT,
    CTX_OUTPUT,
    CTX_BATTERY,
    CTX_CHARGER,
    CTX_POWER_SUMMARY,
} usage_ctx_t;

/* ================================================================== */
/* Status flags - combined to form ups.status                          */
/* ================================================================== */

typedef enum {
    FLAG_AC_PRESENT = 0,
    FLAG_CHARGING,
    FLAG_DISCHARGING,
    FLAG_GOOD,
    FLAG_INTERNAL_FAILURE,
    FLAG_OVERLOAD,
    FLAG_SHUTDOWN_IMMINENT,
    FLAG_BOOST,
    FLAG_BUCK,
    FLAG_BELOW_REMAINING_CAP,
    FLAG_OVERCHARGED,
    FLAG_OVER_TEMPERATURE,
    FLAG_NEED_REPLACEMENT,
    FLAG_VOLTAGE_OUT_OF_RANGE,
    FLAG_FREQUENCY_OUT_OF_RANGE,
    FLAG_FULLY_CHARGED,
    FLAG_FULLY_DISCHARGED,
    FLAG_AWAITING_POWER,
    FLAG_COMM_LOST,
    FLAG_BATTERY_PRESENT,
    FLAG_REMAINING_TIME_EXPIRED,
    FLAG_MAX
} flag_id_t;

/* ================================================================== */
/* Mapped field - one HID report field -> one NUT variable             */
/* ================================================================== */

typedef enum { RPT_FEATURE = 0, RPT_INPUT } rpt_type_t;

typedef struct {
    char       nut_name[NUT_VAR_NAME_LEN];
    uint8_t    report_id;
    rpt_type_t report_type;
    uint16_t   bit_offset;      /* within report data (after report-ID byte) */
    uint8_t    bit_size;
    int32_t    logical_min;
    int32_t    logical_max;
    int32_t    physical_min;
    int32_t    physical_max;
    bool       have_phys;       /* true if Physical Min/Max explicitly set */
    int8_t     unit_exp;        /* HID Unit Exponent */
    int32_t    unit;            /* HID Unit type */
    flag_id_t  flag_id;         /* FLAG_MAX when not a boolean flag */
} mapped_field_t;

/* ================================================================== */
/* Per-UPS instance state                                              */
/* ================================================================== */

/* Known USB vendor/product IDs for device-specific quirks */
#define VID_CYBERPOWER  0x0764
#define PID_CPS_0501    0x0501   /* Most common CPS PID - has voltage bug */
#define VID_APC         0x051D
#define PID_APC_BACKUPS 0x0002   /* Most Back-UPS models (ES, CS, RS, XS) */
#define PID_APC_5G_A    0x0003   /* 5G Smart-UPS models */
#define PID_APC_5G_B    0x0004   /* Smart-UPS 1000 FW 04.3+ */
#define VID_ECOFLOW     0x3746
#define VID_LEGRAND     0x1CB0
#define VID_TRIPPLITE   0x09AE
#define VID_LIEBERT     0x10AF
#define VID_PHOENIXTEC  0x06DA   /* Liebert / Ever */

typedef struct {
    hid_host_device_handle_t dev;
    bool connected;
    bool slot_used;

    uint16_t vid;
    uint16_t pid;

    mapped_field_t fields[MAX_MAPPED_FIELDS];
    int            nfields;

    nut_var_t vars[MAX_NUT_VARS];
    int       nvars;

    bool flag_val[FLAG_MAX];
    bool flag_present[FLAG_MAX];

    int  beeper_idx;
    int  test_idx;
    char name[32];
} ups_instance_t;

/* ================================================================== */
/* Usage ->NUT mapping table                                           */
/* ================================================================== */

typedef struct {
    uint16_t  page;
    uint16_t  usage;
    usage_ctx_t ctx;          /* CTX_NONE = match any context */
    const char *nut_name;     /* NULL for boolean flags */
    flag_id_t  flag_id;       /* FLAG_MAX when not a flag */
} usage_map_t;

static const usage_map_t s_map[] = {
    /*
     * Power Device page (0x84) - numeric values
     * Ref: USB HID Usage Tables for Power Devices, Release 1.0
     * Cross-referenced with NUT drivers: apc-hid.c, belkin-hid.c,
     * cps-hid.c, tripplite-hid.c, delta_ups-hid.c, mge-hid.c,
     * liebert-hid.c, salicru-hid.c, and libhid.c
     */
    /* 0x30 = Voltage */
    {0x84, 0x30, CTX_OUTPUT,        "output.voltage",          FLAG_MAX},
    {0x84, 0x30, CTX_INPUT,         "input.voltage",           FLAG_MAX},
    {0x84, 0x30, CTX_BATTERY,       "battery.voltage",         FLAG_MAX},
    {0x84, 0x30, CTX_POWER_SUMMARY, "battery.voltage",         FLAG_MAX},
    {0x84, 0x30, CTX_NONE,          "output.voltage",          FLAG_MAX},
    /* 0x31 = Current */
    {0x84, 0x31, CTX_OUTPUT,        "output.current",          FLAG_MAX},
    {0x84, 0x31, CTX_INPUT,         "input.current",           FLAG_MAX},
    {0x84, 0x31, CTX_NONE,          "output.current",          FLAG_MAX},
    /* 0x32 = Frequency */
    {0x84, 0x32, CTX_OUTPUT,        "output.frequency",        FLAG_MAX},
    {0x84, 0x32, CTX_INPUT,         "input.frequency",         FLAG_MAX},
    {0x84, 0x32, CTX_NONE,          "output.frequency",        FLAG_MAX},
    /* 0x33 = ApparentPower */
    {0x84, 0x33, CTX_OUTPUT,        "ups.power",               FLAG_MAX},
    {0x84, 0x33, CTX_NONE,          "ups.power",               FLAG_MAX},
    /* 0x34 = ActivePower */
    {0x84, 0x34, CTX_OUTPUT,        "ups.realpower",           FLAG_MAX},
    {0x84, 0x34, CTX_NONE,          "ups.realpower",           FLAG_MAX},
    /* 0x35 = PercentLoad */
    {0x84, 0x35, CTX_NONE,          "ups.load",                FLAG_MAX},
    /* 0x36 = Temperature */
    {0x84, 0x36, CTX_BATTERY,       "battery.temperature",     FLAG_MAX},
    {0x84, 0x36, CTX_NONE,          "ups.temperature",         FLAG_MAX},
    /* 0x37 = Humidity */
    {0x84, 0x37, CTX_NONE,          "ambient.humidity",        FLAG_MAX},
    /* 0x40 = ConfigVoltage */
    {0x84, 0x40, CTX_OUTPUT,        "output.voltage.nominal",  FLAG_MAX},
    {0x84, 0x40, CTX_INPUT,         "input.voltage.nominal",   FLAG_MAX},
    {0x84, 0x40, CTX_BATTERY,       "battery.voltage.nominal", FLAG_MAX},
    {0x84, 0x40, CTX_POWER_SUMMARY, "battery.voltage.nominal", FLAG_MAX},
    {0x84, 0x40, CTX_NONE,          "output.voltage.nominal",  FLAG_MAX},
    /* 0x42 = ConfigFrequency */
    {0x84, 0x42, CTX_OUTPUT,        "output.frequency.nominal",FLAG_MAX},
    {0x84, 0x42, CTX_INPUT,         "input.frequency.nominal", FLAG_MAX},
    {0x84, 0x42, CTX_NONE,          "output.frequency.nominal",FLAG_MAX},
    /* 0x43 = ConfigApparentPower */
    {0x84, 0x43, CTX_NONE,          "ups.power.nominal",       FLAG_MAX},
    /* 0x44 = ConfigActivePower */
    {0x84, 0x44, CTX_NONE,          "ups.realpower.nominal",   FLAG_MAX},
    /* 0x45 = ConfigPercentLoad */
    {0x84, 0x45, CTX_NONE,          "ups.load.nominal",        FLAG_MAX},
    /* 0x53 = LowVoltageTransfer */
    {0x84, 0x53, CTX_NONE,          "input.transfer.low",      FLAG_MAX},
    /* 0x54 = HighVoltageTransfer */
    {0x84, 0x54, CTX_NONE,          "input.transfer.high",     FLAG_MAX},
    /* 0x55 = DelayBeforeReboot */
    {0x84, 0x55, CTX_NONE,          "ups.delay.reboot",        FLAG_MAX},
    /* 0x56 = DelayBeforeStartup */
    {0x84, 0x56, CTX_NONE,          "ups.delay.start",         FLAG_MAX},
    /* 0x57 = DelayBeforeShutdown */
    {0x84, 0x57, CTX_NONE,          "ups.delay.shutdown",      FLAG_MAX},
    /* 0x58 = Test */
    {0x84, 0x58, CTX_NONE,          "ups.test.result",         FLAG_MAX},
    /* 0x5A = AudibleAlarmControl */
    {0x84, 0x5A, CTX_NONE,          "ups.beeper.status",       FLAG_MAX},

    /*
     * Battery System page (0x85) - numeric values
     */
    /* 0x29 = RemainingCapacityLimit */
    {0x85, 0x29, CTX_NONE,          "battery.charge.low",      FLAG_MAX},
    /* 0x2A = RemainingTimeLimit */
    {0x85, 0x2A, CTX_NONE,          "battery.runtime.low",     FLAG_MAX},
    /* 0x62 = AverageCurrent */
    {0x85, 0x62, CTX_NONE,          "battery.current",         FLAG_MAX},
    /* 0x64 = RelativeStateOfCharge (fallback for battery.charge) */
    {0x85, 0x64, CTX_NONE,          "battery.charge",          FLAG_MAX},
    /* 0x66 = RemainingCapacity */
    {0x85, 0x66, CTX_NONE,          "battery.charge",          FLAG_MAX},
    /* 0x67 = FullChargeCapacity */
    {0x85, 0x67, CTX_NONE,          "battery.capacity",        FLAG_MAX},
    /* 0x68 = RunTimeToEmpty */
    {0x85, 0x68, CTX_NONE,          "battery.runtime",         FLAG_MAX},
    /* 0x69 = AverageTimeToEmpty (fallback for battery.runtime) */
    {0x85, 0x69, CTX_NONE,          "battery.runtime",         FLAG_MAX},
    /* 0x6B = CycleCount */
    {0x85, 0x6B, CTX_NONE,          "battery.cyclecount",      FLAG_MAX},
    /* 0x83 = DesignCapacity */
    {0x85, 0x83, CTX_NONE,          "battery.capacity.nominal",FLAG_MAX},
    /* 0x85 = ManufacturerDate */
    {0x85, 0x85, CTX_NONE,          "battery.mfr.date",        FLAG_MAX},
    /* 0x8C = WarningCapacityLimit */
    {0x85, 0x8C, CTX_NONE,          "battery.charge.warning",  FLAG_MAX},
    /* 0x89 = iDeviceChemistry (string index, treated as int) */
    {0x85, 0x89, CTX_NONE,          "battery.type",            FLAG_MAX},

    /*
     * Power Device page (0x84) - boolean status flags
     */
    {0x84, 0x61, CTX_NONE, NULL, FLAG_GOOD},
    {0x84, 0x62, CTX_NONE, NULL, FLAG_INTERNAL_FAILURE},
    {0x84, 0x63, CTX_NONE, NULL, FLAG_VOLTAGE_OUT_OF_RANGE},
    {0x84, 0x64, CTX_NONE, NULL, FLAG_FREQUENCY_OUT_OF_RANGE},
    {0x84, 0x65, CTX_NONE, NULL, FLAG_OVERLOAD},
    {0x84, 0x66, CTX_NONE, NULL, FLAG_OVERCHARGED},
    {0x84, 0x67, CTX_NONE, NULL, FLAG_OVER_TEMPERATURE},
    {0x84, 0x69, CTX_NONE, NULL, FLAG_SHUTDOWN_IMMINENT},
    {0x84, 0x6E, CTX_NONE, NULL, FLAG_BOOST},
    {0x84, 0x6F, CTX_NONE, NULL, FLAG_BUCK},
    {0x84, 0x72, CTX_NONE, NULL, FLAG_AWAITING_POWER},
    {0x84, 0x73, CTX_NONE, NULL, FLAG_COMM_LOST},

    /*
     * Battery System page (0x85) - boolean status flags
     */
    {0x85, 0x42, CTX_NONE, NULL, FLAG_BELOW_REMAINING_CAP},
    {0x85, 0x43, CTX_NONE, NULL, FLAG_REMAINING_TIME_EXPIRED},
    {0x85, 0x44, CTX_NONE, NULL, FLAG_CHARGING},
    {0x85, 0x45, CTX_NONE, NULL, FLAG_DISCHARGING},
    {0x85, 0x46, CTX_NONE, NULL, FLAG_FULLY_CHARGED},
    {0x85, 0x47, CTX_NONE, NULL, FLAG_FULLY_DISCHARGED},
    {0x85, 0x4B, CTX_NONE, NULL, FLAG_NEED_REPLACEMENT},
    {0x85, 0xD0, CTX_NONE, NULL, FLAG_AC_PRESENT},
    {0x85, 0xD1, CTX_NONE, NULL, FLAG_BATTERY_PRESENT},

    /*
     * Belkin vendor page (0x86) - unique usage IDs
     * These IDs have no equivalent on the standard 0x84/0x85 pages.
     * Common IDs (Voltage, Frequency, etc.) are handled by page
     * normalization in find_mapping() instead of explicit entries.
     */
    {0x86, 0x39, CTX_NONE,          "battery.charge",          FLAG_MAX},
    {0x86, 0x6C, CTX_NONE,          "battery.runtime",         FLAG_MAX},

    /*
     * CyberPower vendor page (0xFF01) - unique usage IDs
     * NOT a mirroring page; only vendor-specific usages live here.
     */
    {0xFF01, 0x43, CTX_NONE,        "input.sensitivity",       FLAG_MAX},

    /*
     * APC vendor page (0xFF86) - unique usage IDs
     * Standard usage IDs mirrored on this page are handled by page
     * normalization; only APC-specific usages need explicit entries.
     * Usages that collide with standard page IDs but have different
     * semantics must be listed here to prevent wrong normalization.
     */
    {0xFF86, 0x16, CTX_NONE,        "battery.date",            FLAG_MAX},
    {0xFF86, 0x42, CTX_NONE,        "ups.firmware",            FLAG_MAX},
    {0xFF86, 0x52, CTX_NONE,        "input.transfer.reason",   FLAG_MAX},
    {0xFF86, 0x60, CTX_NONE,        "ups.status.flag",         FLAG_MAX},
    {0xFF86, 0x61, CTX_NONE,        "input.sensitivity",       FLAG_MAX},
    {0xFF86, 0x79, CTX_NONE,        "ups.firmware.aux",        FLAG_MAX},
    /* Back-UPS ES models use these for shutdown/startup delays */
    {0xFF86, 0x7D, CTX_NONE,        "ups.delay.shutdown",      FLAG_MAX},
    {0xFF86, 0x7E, CTX_NONE,        "ups.delay.start",         FLAG_MAX},
    {0xFF86, 0x7C, CTX_NONE,        "ups.delay.reboot",        FLAG_MAX},

    /* sentinel */
    {0, 0, CTX_NONE, NULL, FLAG_MAX},
};

/* ================================================================== */
/* Module state                                                        */
/* ================================================================== */

static ups_instance_t   s_ups[MAX_UPS_DEVICES];
static SemaphoreHandle_t s_mutex;
static QueueHandle_t     s_event_q;

/* ================================================================== */
/* HID event queue entry                                               */
/* ================================================================== */

typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t  event;
    void                    *arg;
} hid_event_t;

/* ================================================================== */
/* Slot management                                                     */
/* ================================================================== */

static ups_instance_t *alloc_slot(hid_host_device_handle_t dev)
{
    for (int i = 0; i < MAX_UPS_DEVICES; i++) {
        if (!s_ups[i].slot_used) {
            memset(&s_ups[i], 0, sizeof(ups_instance_t));
            s_ups[i].dev = dev;
            s_ups[i].slot_used = true;
            s_ups[i].beeper_idx = -1;
            s_ups[i].test_idx = -1;
            s_ups[i].flag_val[FLAG_GOOD] = true;
            s_ups[i].flag_val[FLAG_AC_PRESENT] = true;
            s_ups[i].flag_val[FLAG_BATTERY_PRESENT] = true;

            if (i == 0)
                snprintf(s_ups[i].name, sizeof(s_ups[i].name),
                         "%s", CONFIG_NUT_UPS_NAME);
            else
                snprintf(s_ups[i].name, sizeof(s_ups[i].name),
                         "%s-%d", CONFIG_NUT_UPS_NAME, i + 1);
            return &s_ups[i];
        }
    }
    return NULL;
}

static ups_instance_t *find_by_name(const char *name)
{
    if (!name) return NULL;
    for (int i = 0; i < MAX_UPS_DEVICES; i++) {
        if (s_ups[i].connected && strcmp(s_ups[i].name, name) == 0)
            return &s_ups[i];
    }
    return NULL;
}

/* ================================================================== */
/* Tiny helpers                                                        */
/* ================================================================== */

static uint32_t rd_u(const uint8_t *d, uint8_t sz)
{
    switch (sz) {
    case 0: return 0;
    case 1: return d[0];
    case 2: return d[0] | ((uint32_t)d[1] << 8);
    case 4: return d[0] | ((uint32_t)d[1] << 8) |
                   ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    }
    return 0;
}

static int32_t rd_s(const uint8_t *d, uint8_t sz)
{
    switch (sz) {
    case 0: return 0;
    case 1: return (int8_t)d[0];
    case 2: return (int16_t)(d[0] | ((uint16_t)d[1] << 8));
    case 4: return (int32_t)(d[0] | ((uint32_t)d[1] << 8) |
                             ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
    }
    return 0;
}

static int32_t extract_bits(const uint8_t *data, uint16_t off, uint8_t sz)
{
    int32_t v = 0;
    for (int i = 0; i < sz; i++) {
        if (data[(off + i) / 8] & (1 << ((off + i) % 8)))
            v |= (1 << i);
    }
    return v;
}

static void inject_bits(uint8_t *data, uint16_t off, uint8_t sz, int32_t v)
{
    for (int i = 0; i < sz; i++) {
        int byte = (off + i) / 8, bit = (off + i) % 8;
        if (v & (1 << i))
            data[byte] |= (1 << bit);
        else
            data[byte] &= ~(1 << bit);
    }
}

/* ================================================================== */
/* Context from collection stack                                       */
/* ================================================================== */

static usage_ctx_t ctx_from_stack(const uint32_t *stk, int depth)
{
    for (int i = depth - 1; i >= 0; i--) {
        uint16_t pg = (stk[i] >> 16) & 0xFFFF;
        uint16_t us = stk[i] & 0xFFFF;
        if (pg == 0x84 || pg == 0x86 || pg == 0x02 || pg == 0xFF86) {
            /* Standard Power Device page (0x84) collection usages.
             * Vendors like Belkin (0x86), PowerCOM (0x02), and
             * APC (0xFF86) mirror the same collection usage IDs
             * on their proprietary pages. */
            if (us == 0x1A) return CTX_INPUT;
            if (us == 0x1C) return CTX_OUTPUT;
            if (us == 0x10 || us == 0x12) return CTX_BATTERY;
            if (us == 0x14) return CTX_CHARGER;
            if (us == 0x24) return CTX_POWER_SUMMARY;
            if (us == 0x04 || us == 0x05) return CTX_UPS;
        }
    }
    return CTX_NONE;
}

/* ================================================================== */
/* Look up mapping for (page, usage, context)                          */
/* ================================================================== */

static const usage_map_t *find_mapping_page(uint16_t pg, uint16_t us,
                                             usage_ctx_t c)
{
    const usage_map_t *fb = NULL;
    for (int i = 0; s_map[i].page || s_map[i].usage; i++) {
        if (s_map[i].page == pg && s_map[i].usage == us) {
            if (s_map[i].ctx == c) return &s_map[i];
            if (s_map[i].ctx == CTX_NONE && !fb) fb = &s_map[i];
        }
    }
    return fb;
}

/* Vendor pages known to mirror standard 0x84/0x85 usage IDs.
 * Pages NOT in this list (e.g. CyberPower 0xFF01) use unique
 * vendor-specific usages and must NOT be normalized. */
static bool is_mirror_page(uint16_t pg)
{
    return pg == 0x86     /* Belkin */
        || pg == 0x02     /* PowerCOM */
        || pg == 0xFFFF   /* Tripp Lite */
        || pg == 0xFF86   /* APC */
        || pg == 0xFF00;  /* Ever */
}

static const usage_map_t *find_mapping(uint16_t pg, uint16_t us, usage_ctx_t c)
{
    /* Try exact page first (handles vendor-unique entries like Belkin 0x86:0x39) */
    const usage_map_t *m = find_mapping_page(pg, us, c);
    if (m) return m;

    /* Vendor page normalization: only for pages known to mirror standard
     * usage IDs.  Other vendor pages (e.g. CyberPower 0xFF01) have unique
     * usages whose IDs would collide with standard ones if normalized. */
    if (pg != 0x84 && pg != 0x85 && is_mirror_page(pg)) {
        m = find_mapping_page(0x84, us, c);
        if (m) return m;
        m = find_mapping_page(0x85, us, c);
        if (m) return m;
    }
    return NULL;
}

static bool name_mapped_i(ups_instance_t *u, const char *n)
{
    if (!n) return false;
    for (int i = 0; i < u->nfields; i++)
        if (strcmp(u->fields[i].nut_name, n) == 0) return true;
    return false;
}

/* ================================================================== */
/* Per-instance NUT variable helpers (caller must hold s_mutex)        */
/* ================================================================== */

static bool get_var_i(ups_instance_t *u, const char *name, char *out, size_t max)
{
    for (int i = 0; i < u->nvars; i++) {
        if (strcmp(u->vars[i].name, name) == 0) {
            snprintf(out, max, "%s", u->vars[i].value);
            return true;
        }
    }
    return false;
}

static void set_var_i(ups_instance_t *u, const char *name, const char *value)
{
    for (int i = 0; i < u->nvars; i++) {
        if (strcmp(u->vars[i].name, name) == 0) {
            strncpy(u->vars[i].value, value, NUT_VAR_VALUE_LEN - 1);
            u->vars[i].value[NUT_VAR_VALUE_LEN - 1] = '\0';
            return;
        }
    }
    if (u->nvars < MAX_NUT_VARS) {
        strncpy(u->vars[u->nvars].name, name, NUT_VAR_NAME_LEN - 1);
        u->vars[u->nvars].name[NUT_VAR_NAME_LEN - 1] = '\0';
        strncpy(u->vars[u->nvars].value, value, NUT_VAR_VALUE_LEN - 1);
        u->vars[u->nvars].value[NUT_VAR_VALUE_LEN - 1] = '\0';
        u->nvars++;
    }
}

/* ================================================================== */
/* Parse HID Report Descriptor (per-instance)                          */
/* ================================================================== */

static void parse_descriptor(ups_instance_t *u, const uint8_t *desc, size_t len)
{
    /* Global state */
    uint16_t g_page = 0;
    int32_t  g_lmin = 0, g_lmax = 0;
    int32_t  g_pmin = 0, g_pmax = 0;
    bool     g_have_phys = false;
    int8_t   g_unit_exp = 0;
    int32_t  g_unit = 0;
    uint32_t g_rsize = 0, g_rcount = 0;
    uint8_t  g_rid = 0;

    /* Local state */
    uint32_t usages[MAX_USAGES_PER_ITEM];
    int      ucount = 0;
    uint32_t umin = 0, umax = 0;
    bool     urange = false;

    /* Collection stack */
    uint32_t cstack[MAX_COLLECTION_DEPTH];
    int      cdepth = 0;

    /* Bit offset per report-id for Feature and Input reports */
    static uint16_t feat_off[256];
    static uint16_t inp_off[256];
    memset(feat_off, 0, sizeof(feat_off));
    memset(inp_off, 0, sizeof(inp_off));

    u->nfields = 0;
    u->beeper_idx = -1;
    u->test_idx = -1;
    memset(u->flag_present, 0, sizeof(u->flag_present));
    memset(u->flag_val, 0, sizeof(u->flag_val));
    u->flag_val[FLAG_GOOD] = true;
    u->flag_val[FLAG_AC_PRESENT] = true;
    u->flag_val[FLAG_BATTERY_PRESENT] = true;

    size_t pos = 0;
    while (pos < len) {
        uint8_t prefix = desc[pos];

        /* Long item */
        if (prefix == 0xFE) {
            if (pos + 1 >= len) break;
            pos += 3 + desc[pos + 1];
            continue;
        }

        uint8_t bSize = prefix & 0x03;
        uint8_t bType = (prefix >> 2) & 0x03;
        uint8_t bTag  = (prefix >> 4) & 0x0F;
        uint8_t dsz   = (bSize == 3) ? 4 : bSize;

        if (pos + 1 + dsz > len) break;
        const uint8_t *d = &desc[pos + 1];
        uint32_t uval = rd_u(d, dsz);
        int32_t  sval = rd_s(d, dsz);
        pos += 1 + dsz;

        switch (bType) {
        /* ---------- Global ---------- */
        case 1:
            switch (bTag) {
            case 0x0: g_page     = uval; break;
            case 0x1: g_lmin     = sval; break;
            case 0x2: g_lmax     = sval; break;
            case 0x3: g_pmin     = sval; g_have_phys = true; break;
            case 0x4: g_pmax     = sval; g_have_phys = true; break;
            case 0x5:
                g_unit_exp = (int8_t)(uval & 0x0F);
                if (g_unit_exp > 7) g_unit_exp |= (int8_t)0xF0;
                break;
            case 0x6: g_unit     = (int32_t)uval; break;
            case 0x7: g_rsize   = uval; break;
            case 0x8: g_rid     = uval; break;
            case 0x9: g_rcount  = uval; break;
            }
            break;

        /* ---------- Local ---------- */
        case 2:
            switch (bTag) {
            case 0x0:
                if (ucount < MAX_USAGES_PER_ITEM) {
                    usages[ucount++] = (dsz == 4) ? uval
                        : ((uint32_t)g_page << 16) | (uval & 0xFFFF);
                }
                break;
            case 0x1:
                umin = (dsz == 4) ? uval : ((uint32_t)g_page << 16) | (uval & 0xFFFF);
                urange = true;
                break;
            case 0x2:
                umax = (dsz == 4) ? uval : ((uint32_t)g_page << 16) | (uval & 0xFFFF);
                break;
            }
            break;

        /* ---------- Main ---------- */
        case 0:
            switch (bTag) {
            case 0xA: /* Collection */
                if (cdepth < MAX_COLLECTION_DEPTH)
                    cstack[cdepth++] = (ucount > 0) ? usages[0] : 0;
                ucount = 0; urange = false;
                break;

            case 0xC: /* End Collection */
                if (cdepth > 0) cdepth--;
                break;

            case 0xB: /* Feature */
            case 0x8: /* Input */
            {
                rpt_type_t rtype = (bTag == 0xB) ? RPT_FEATURE : RPT_INPUT;
                uint16_t *off_arr = (rtype == RPT_FEATURE) ? feat_off : inp_off;
                bool is_const = uval & 0x01;
                for (uint32_t i = 0; i < g_rcount && u->nfields < MAX_MAPPED_FIELDS; i++) {
                    uint32_t fu = 0;
                    if (urange) {
                        fu = umin + i;
                        if (fu > umax) fu = umax;
                    } else if (ucount > 0) {
                        fu = (i < (uint32_t)ucount) ? usages[i] : usages[ucount - 1];
                    }

                    if (!is_const && fu != 0) {
                        uint16_t up = (fu >> 16) & 0xFFFF;
                        uint16_t uid = fu & 0xFFFF;
                        usage_ctx_t ctx = ctx_from_stack(cstack, cdepth);
                        const usage_map_t *m = find_mapping(up, uid, ctx);

                        if (m && (m->flag_id < FLAG_MAX ||
                                  (m->nut_name && !name_mapped_i(u, m->nut_name)))) {
                            mapped_field_t *f = &u->fields[u->nfields];
                            if (m->nut_name)
                                strncpy(f->nut_name, m->nut_name, NUT_VAR_NAME_LEN - 1);
                            else
                                f->nut_name[0] = '\0';
                            f->nut_name[NUT_VAR_NAME_LEN - 1] = '\0';
                            f->report_id    = g_rid;
                            f->report_type  = rtype;
                            f->bit_offset   = off_arr[g_rid];
                            f->bit_size     = g_rsize;
                            f->logical_min  = g_lmin;
                            f->logical_max  = g_lmax;
                            f->physical_min = g_pmin;
                            f->physical_max = g_pmax;
                            f->have_phys    = g_have_phys;
                            f->unit_exp     = g_unit_exp;
                            f->unit         = g_unit;
                            f->flag_id      = m->flag_id;

                            if (m->flag_id < FLAG_MAX)
                                u->flag_present[m->flag_id] = true;

                            if (m->nut_name &&
                                strcmp(m->nut_name, "ups.beeper.status") == 0)
                                u->beeper_idx = u->nfields;

                            if (m->nut_name && rtype == RPT_FEATURE &&
                                strcmp(m->nut_name, "ups.test.result") == 0)
                                u->test_idx = u->nfields;

                            u->nfields++;
                        }
                    }
                    off_arr[g_rid] += g_rsize;
                }
                ucount = 0; urange = false;
                break;
            }

            case 0x9: /* Output - skip */
                ucount = 0; urange = false;
                break;
            }
            break;
        }
    }

    ESP_LOGI(TAG, "'%s': descriptor parsed - %d mapped fields from %zu bytes",
             u->name, u->nfields, len);
}

/* ================================================================== */
/* APC report descriptor fixup: voltage range correction               */
/* Some APC Back-UPS models (PID 0x0002) have descriptor bugs where   */
/* the Voltage/ConfigVoltage logical_max is too low for 220-240V.     */
/* Fix by using HighVoltageTransfer's logical_max as reference.       */
/* Only applied for APC VID to avoid affecting other brands.          */
/* ================================================================== */

/* Forward declaration - defined in "Writable-field helpers" section */
static mapped_field_t *find_field_i(ups_instance_t *u, const char *name);

static void apc_fix_voltage_range(ups_instance_t *u)
{
    if (u->vid != VID_APC)
        return;

    /* Find HighVoltageTransfer field to get its logical_max as reference */
    mapped_field_t *hvt = find_field_i(u, "input.transfer.high");
    if (!hvt || hvt->logical_max <= 0)
        return;

    int32_t hvt_max = hvt->logical_max;
    bool fixed = false;

    for (int i = 0; i < u->nfields; i++) {
        mapped_field_t *f = &u->fields[i];
        if (!f->nut_name[0]) continue;

        /* Fix Voltage fields (input.voltage, output.voltage) */
        if ((strcmp(f->nut_name, "input.voltage") == 0 ||
             strcmp(f->nut_name, "output.voltage") == 0) &&
            f->logical_max < hvt_max)
        {
            ESP_LOGW(TAG, "'%s': fixing %s logical_max %d -> %d "
                     "(APC descriptor bug)",
                     u->name, f->nut_name,
                     (int)f->logical_max, (int)(hvt_max * 2));
            f->logical_min = 0;
            f->logical_max = hvt_max * 2;
            fixed = true;
        }

        /* Fix ConfigVoltage fields (*.voltage.nominal) */
        if ((strcmp(f->nut_name, "input.voltage.nominal") == 0 ||
             strcmp(f->nut_name, "output.voltage.nominal") == 0) &&
            f->logical_max < hvt_max)
        {
            /* Use max representable by bit width, capped at 255 */
            int32_t max_by_bits = (1 << f->bit_size) - 1;
            int32_t new_max = (max_by_bits < 255) ? max_by_bits : 255;
            ESP_LOGW(TAG, "'%s': fixing %s logical_max %d -> %d "
                     "(APC descriptor bug)",
                     u->name, f->nut_name,
                     (int)f->logical_max, (int)new_max);
            f->logical_max = new_max;
            fixed = true;
        }

        /* Fix ConfigActivePower nominal if too low */
        if (strcmp(f->nut_name, "ups.realpower.nominal") == 0 &&
            f->logical_max < 2048)
        {
            ESP_LOGW(TAG, "'%s': fixing %s logical_max %d -> 2048 "
                     "(APC descriptor bug)",
                     u->name, f->nut_name, (int)f->logical_max);
            f->logical_max = 2048;
            fixed = true;
        }
    }

    if (fixed)
        ESP_LOGI(TAG, "'%s': APC voltage range corrections applied", u->name);
}

/* ================================================================== */
/* Derive composite NUT variables from boolean flags (per-instance)    */
/* ================================================================== */

static void derive_status(ups_instance_t *u)
{
    char st[32] = "";

    /* CyberPower UT series quirk: some models report ACPresent=1 and
     * Discharging=1 simultaneously when actually on battery.  Only
     * applied when VID matches CyberPower to avoid affecting others. */
    bool on_ac = true;
    if (u->flag_present[FLAG_AC_PRESENT]) {
        on_ac = u->flag_val[FLAG_AC_PRESENT];
        if (on_ac && u->vid == VID_CYBERPOWER &&
            u->flag_present[FLAG_DISCHARGING] && u->flag_val[FLAG_DISCHARGING])
            on_ac = false;
    }
    strcpy(st, on_ac ? "OL" : "OB");

    if ((u->flag_present[FLAG_BELOW_REMAINING_CAP]   && u->flag_val[FLAG_BELOW_REMAINING_CAP]) ||
        (u->flag_present[FLAG_SHUTDOWN_IMMINENT]      && u->flag_val[FLAG_SHUTDOWN_IMMINENT]) ||
        (u->flag_present[FLAG_FULLY_DISCHARGED]       && u->flag_val[FLAG_FULLY_DISCHARGED]) ||
        (u->flag_present[FLAG_REMAINING_TIME_EXPIRED] && u->flag_val[FLAG_REMAINING_TIME_EXPIRED]))
        strcat(st, " LB");

    if ((u->flag_present[FLAG_GOOD]             && !u->flag_val[FLAG_GOOD]) ||
        (u->flag_present[FLAG_INTERNAL_FAILURE] &&  u->flag_val[FLAG_INTERNAL_FAILURE]) ||
        (u->flag_present[FLAG_NEED_REPLACEMENT] &&  u->flag_val[FLAG_NEED_REPLACEMENT]))
        strcat(st, " RB");

    if (u->flag_present[FLAG_OVERLOAD] && u->flag_val[FLAG_OVERLOAD])
        strcat(st, " OVER");

    if (u->flag_present[FLAG_BOOST] && u->flag_val[FLAG_BOOST])
        strcat(st, " BOOST");

    if (u->flag_present[FLAG_BUCK] && u->flag_val[FLAG_BUCK])
        strcat(st, " TRIM");

    /* APC vendor status flag (0xFF86:0x60): value 0 = output off.
     * Only checked for APC devices to avoid false OFF on other brands. */
    if (u->vid == VID_APC) {
        char sflag[NUT_VAR_VALUE_LEN];
        if (get_var_i(u, "ups.status.flag", sflag, sizeof(sflag)) &&
            atoi(sflag) == 0)
            strcat(st, " OFF");
    }

    set_var_i(u, "ups.status", st);

    if (u->flag_present[FLAG_CHARGING] || u->flag_present[FLAG_DISCHARGING] ||
        u->flag_present[FLAG_FULLY_CHARGED]) {
        if (u->flag_val[FLAG_FULLY_CHARGED])
            set_var_i(u, "battery.charger.status", "charged");
        else if (u->flag_val[FLAG_CHARGING])
            set_var_i(u, "battery.charger.status", "charging");
        else if (u->flag_val[FLAG_DISCHARGING])
            set_var_i(u, "battery.charger.status", "discharging");
        else
            set_var_i(u, "battery.charger.status", "resting");
    }

    /* Derive output.voltage if the UPS doesn't report it:
     *   On battery (OB): inverter outputs nominal ->use input.voltage.nominal
     *   On AC (OL):       output tracks input     ->use input.voltage */
    if (!name_mapped_i(u, "output.voltage")) {
        char val[NUT_VAR_VALUE_LEN];
        bool on_battery = u->flag_present[FLAG_AC_PRESENT] &&
                          !u->flag_val[FLAG_AC_PRESENT];
        if (on_battery) {
            if (get_var_i(u, "input.voltage.nominal", val, sizeof(val)))
                set_var_i(u, "output.voltage", val);
        } else {
            if (get_var_i(u, "input.voltage", val, sizeof(val)))
                set_var_i(u, "output.voltage", val);
        }
    }
    if (!name_mapped_i(u, "output.voltage.nominal")) {
        char val[NUT_VAR_VALUE_LEN];
        if (get_var_i(u, "input.voltage.nominal", val, sizeof(val)))
            set_var_i(u, "output.voltage.nominal", val);
    }

    /* CyberPower PID 0x0501 battery voltage bug: some models report
     * battery voltage that is 1.5x the actual value.  Correct by
     * multiplying by 2/3 when the reported value exceeds 1.4x nominal.
     * Only applied when VID/PID match to avoid affecting other brands. */
    if (u->vid == VID_CYBERPOWER && u->pid == PID_CPS_0501) {
        char vbat[NUT_VAR_VALUE_LEN], vnom[NUT_VAR_VALUE_LEN];
        if (get_var_i(u, "battery.voltage", vbat, sizeof(vbat)) &&
            get_var_i(u, "battery.voltage.nominal", vnom, sizeof(vnom)))
        {
            double bat = atof(vbat);
            double nom = atof(vnom);
            if (nom > 0 && bat > nom * 1.4) {
                bat = bat * 2.0 / 3.0;
                char corrected[NUT_VAR_VALUE_LEN];
                snprintf(corrected, sizeof(corrected), "%.1f", bat);
                set_var_i(u, "battery.voltage", corrected);
            }
        }
    }

    /* Tripp Lite battery voltage /10 bug: some models report battery
     * voltage 10x too high.  Correct when value exceeds 5x nominal.
     * Only applied for Tripp Lite VID. */
    if (u->vid == VID_TRIPPLITE) {
        char vbat[NUT_VAR_VALUE_LEN], vnom[NUT_VAR_VALUE_LEN];
        if (get_var_i(u, "battery.voltage", vbat, sizeof(vbat)) &&
            get_var_i(u, "battery.voltage.nominal", vnom, sizeof(vnom)))
        {
            double bat = atof(vbat);
            double nom = atof(vnom);
            if (nom > 0 && bat > nom * 5.0) {
                bat /= 10.0;
                char corrected[NUT_VAR_VALUE_LEN];
                snprintf(corrected, sizeof(corrected), "%.1f", bat);
                set_var_i(u, "battery.voltage", corrected);
            }
        }
    }
}

/* ================================================================== */
/* HID Unit type ->built-in exponent offset                            */
/* Matches NUT's HIDUnits[] table in libhid.c                         */
/* ================================================================== */

static int8_t unit_expo_offset(int32_t unit_type)
{
    switch (unit_type) {
    case 0x00F0D121: return 7;  /* Voltage */
    case 0x0000D121: return 7;  /* VA / Watts */
    default:         return 0;
    }
}

/* ================================================================== */
/* Apply a single field value with proper HID->physical scaling        */
/* ================================================================== */

static double logical_to_physical(const mapped_field_t *f, int32_t logical)
{
    double val = (double)logical;

    if (f->have_phys &&
        !(f->physical_min == 0 && f->physical_max == 0) &&
        f->physical_max > f->physical_min &&
        f->logical_max > f->logical_min)
    {
        double factor = (double)(f->physical_max - f->physical_min)
                      / (double)(f->logical_max  - f->logical_min);
        val = (double)(logical - f->logical_min) * factor + f->physical_min;
    }

    int8_t expo = f->unit_exp - unit_expo_offset(f->unit);
    if (expo > 0) {
        for (int8_t i = 0; i < expo; i++) val *= 10.0;
    } else if (expo < 0) {
        for (int8_t i = 0; i > expo; i--) val /= 10.0;
    }

    return val;
}

static void apply_value(ups_instance_t *u, const mapped_field_t *f, int32_t v)
{
    if (f->flag_id < FLAG_MAX) {
        u->flag_val[f->flag_id] = (v != 0);
        return;
    }

    char buf[NUT_VAR_VALUE_LEN];

    /* Beeper status: enumerated value */
    if (strcmp(f->nut_name, "ups.beeper.status") == 0) {
        switch (v) {
        case 1:  strcpy(buf, "disabled"); break;
        case 2:  strcpy(buf, "enabled");  break;
        case 3:  strcpy(buf, "muted");    break;
        default: snprintf(buf, sizeof(buf), "%d", (int)v); break;
        }
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* Test result: HID Usage 0x84:0x58 enumerated values */
    if (strcmp(f->nut_name, "ups.test.result") == 0) {
        switch (v) {
        case 1:  strcpy(buf, "Done and passed");  break;
        case 2:  strcpy(buf, "Done and warning");  break;
        case 3:  strcpy(buf, "Done and error");    break;
        case 4:  strcpy(buf, "Aborted");           break;
        case 5:  strcpy(buf, "In progress");       break;
        case 6:  strcpy(buf, "No test initiated"); break;
        default: snprintf(buf, sizeof(buf), "Unknown (%d)", (int)v); break;
        }
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* Input sensitivity: CyberPower 0xFF01:0x43 and APC 0xFF86:0x61
     * Both use the same enumeration: 0=low, 1=normal/medium, 2=high */
    if (strcmp(f->nut_name, "input.sensitivity") == 0) {
        switch (v) {
        case 0:  strcpy(buf, "low");    break;
        case 1:  strcpy(buf, "normal"); break;
        case 2:  strcpy(buf, "high");   break;
        default: snprintf(buf, sizeof(buf), "%d", (int)v); break;
        }
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* APC battery replacement date: hex-as-decimal encoding.
     * e.g. 0x102202 -> month=0x10=16->10, day=0x22=34->22, year=0x02->2002 */
    if (strcmp(f->nut_name, "battery.date") == 0) {
        uint32_t raw = (uint32_t)v;
        int yy = (int)(raw & 0xFF);
        int mm = (int)((raw >> 16) & 0xFF);
        int dd = (int)((raw >> 8) & 0xFF);
        /* Decode hex-as-decimal: each byte's hex digits are decimal digits */
        mm = ((mm >> 4) & 0x0F) * 10 + (mm & 0x0F);
        dd = ((dd >> 4) & 0x0F) * 10 + (dd & 0x0F);
        yy = ((yy >> 4) & 0x0F) * 10 + (yy & 0x0F);
        int year = (yy >= 70) ? 1900 + yy : 2000 + yy;
        if (mm >= 1 && mm <= 12 && dd >= 1 && dd <= 31)
            snprintf(buf, sizeof(buf), "%04d/%02d/%02d", year, mm, dd);
        else
            snprintf(buf, sizeof(buf), "%d", (int)v);
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* APC line failure cause code: why UPS transferred to battery */
    if (strcmp(f->nut_name, "input.transfer.reason") == 0) {
        switch (v) {
        case 0:  strcpy(buf, "noTransfer"); break;
        case 1:  strcpy(buf, "vrange");     break;  /* Low line voltage */
        case 2:  strcpy(buf, "vrange");     break;  /* High line voltage */
        case 4:  strcpy(buf, "vrange");     break;  /* Notch/spike/blackout */
        case 7:  strcpy(buf, "frange");     break;  /* Frequency out of range */
        case 8:  strcpy(buf, "vrange");     break;  /* Notch or blackout */
        case 9:  strcpy(buf, "vrange");     break;  /* Spike or blackout */
        default: strcpy(buf, "unknown");    break;
        }
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* APC vendor status flag: supplemental on/off indicator */
    if (strcmp(f->nut_name, "ups.status.flag") == 0) {
        /* 8 or 16 = output on (normal), 0 = output off.
         * Stored as a var for derive_status() to consume. */
        snprintf(buf, sizeof(buf), "%d", (int)v);
        set_var_i(u, f->nut_name, buf);
        return;
    }

    /* APC firmware version strings: stored as raw integer (string index).
     * Since we can't resolve HID string indices from a report value,
     * store the numeric value; the real firmware version is parsed
     * from the USB product string at connect time instead. */
    if (strcmp(f->nut_name, "ups.firmware") == 0 ||
        strcmp(f->nut_name, "ups.firmware.aux") == 0) {
        snprintf(buf, sizeof(buf), "%d", (int)v);
        set_var_i(u, f->nut_name, buf);
        return;
    }

    double phys = logical_to_physical(f, v);

    /* Temperature: convert Kelvin to Celsius */
    if (strstr(f->nut_name, "temperature") != NULL) {
        if (phys >= 273.0 && phys <= 373.0)
            phys -= 273.15;
    }

    /* Frequency auto-correction: some firmware (notably CyberPower) reports
     * frequency 10x too high due to wrong Unit Exponent.  Safe for all
     * brands since no real mains frequency exceeds 70 Hz. */
    if (strstr(f->nut_name, "frequency") != NULL) {
        if (phys > 200.0) {
            double corrected = phys / 10.0;
            if (corrected >= 45.0 && corrected <= 70.0)
                phys = corrected;
        }
    }

    /* Charge clamping: some UPS models report > 100%.  Safe for all brands
     * since battery charge is always 0-100%. */
    if (strcmp(f->nut_name, "battery.charge") == 0 && phys > 100.0)
        phys = 100.0;

    /* EcoFlow runtime fix: EcoFlow devices report runtime in minutes
     * instead of seconds.  Only applied for EcoFlow VID. */
    if (u->vid == VID_ECOFLOW && strstr(f->nut_name, "runtime") != NULL)
        phys *= 60.0;

    /* Legrand/Liebert voltage plausibility fix: some firmware has broken
     * or missing HID Unit Exponent, producing values that are orders of
     * magnitude too small.  For mains voltage readings (not battery),
     * try multiplication factors to reach a plausible 80-300V range.
     * Only applied for known-affected vendors. */
    if ((u->vid == VID_LEGRAND || u->vid == VID_LIEBERT ||
         u->vid == VID_PHOENIXTEC) &&
        phys > 0.0 && phys < 1.0 &&
        (strcmp(f->nut_name, "input.voltage") == 0 ||
         strcmp(f->nut_name, "output.voltage") == 0))
    {
        static const double try_mult[] = {1e3, 1e5, 1e6, 1e7, 0};
        for (int k = 0; try_mult[k] > 0; k++) {
            double trial = phys * try_mult[k];
            if (trial >= 80.0 && trial <= 300.0) {
                phys = trial;
                break;
            }
        }
    }

    /* Legrand/Liebert power plausibility fix: same broken exponent issue
     * can affect apparent/real power readings. */
    if ((u->vid == VID_LEGRAND || u->vid == VID_LIEBERT ||
         u->vid == VID_PHOENIXTEC) &&
        phys > 0.0 && phys < 1.0 &&
        (strcmp(f->nut_name, "ups.power") == 0 ||
         strcmp(f->nut_name, "ups.realpower") == 0))
    {
        static const double try_mult[] = {1e3, 1e5, 1e6, 1e7, 0};
        for (int k = 0; try_mult[k] > 0; k++) {
            double trial = phys * try_mult[k];
            if (trial >= 50.0 && trial <= 20000.0) {
                phys = trial;
                break;
            }
        }
    }

    if (strstr(f->nut_name, "runtime") != NULL) {
        snprintf(buf, sizeof(buf), "%.0f", phys);
    } else if (strstr(f->nut_name, "charge") != NULL ||
               strstr(f->nut_name, "load") != NULL) {
        snprintf(buf, sizeof(buf), "%.0f", phys);
    } else if (strstr(f->nut_name, "current") != NULL) {
        snprintf(buf, sizeof(buf), "%.2f", phys);
    } else {
        snprintf(buf, sizeof(buf), "%.1f", phys);
    }

    set_var_i(u, f->nut_name, buf);
}

/* ================================================================== */
/* Poll all mapped Feature and Input reports (per-instance)            */
/* Caller must hold s_mutex.                                           */
/* ================================================================== */

typedef struct { uint8_t rid; rpt_type_t rtype; } poll_key_t;

static void poll_reports(ups_instance_t *u)
{
    if (!u->connected || !u->dev) return;

    poll_key_t done[MAX_POLL_REPORTS];
    int        ndone = 0;
    int        nerrs = 0;

    for (int i = 0; i < u->nfields; i++) {
        uint8_t    rid   = u->fields[i].report_id;
        rpt_type_t rtype = u->fields[i].report_type;

        bool dup = false;
        for (int j = 0; j < ndone; j++)
            if (done[j].rid == rid && done[j].rtype == rtype)
                { dup = true; break; }
        if (dup) continue;
        if (ndone >= MAX_POLL_REPORTS) break;
        done[ndone++] = (poll_key_t){rid, rtype};

        /* Zero buffer to handle short reads safely (some APC Back-UPS
         * models return fewer bytes than the descriptor declares) */
        uint8_t buf[MAX_REPORT_BUF];
        memset(buf, 0, sizeof(buf));
        size_t  len = sizeof(buf);
        hid_report_type_t hid_type = (rtype == RPT_FEATURE)
            ? HID_REPORT_TYPE_FEATURE : HID_REPORT_TYPE_INPUT;
        esp_err_t err = hid_class_request_get_report(
            u->dev, hid_type, rid, buf, &len);
        if (err != ESP_OK) { nerrs++; continue; }

        for (int j = 0; j < u->nfields; j++) {
            if (u->fields[j].report_id != rid ||
                u->fields[j].report_type != rtype) continue;

            /* Guard against reading past the returned data.
             * Some APC Back-UPS models (ES 525, CS series) may
             * return more or fewer bytes than expected. */
            uint16_t end_bit = u->fields[j].bit_offset + u->fields[j].bit_size;
            if (len > 0 && (end_bit + 7) / 8 > (uint16_t)(len - 1))
                continue;

            int32_t v = extract_bits(buf + 1,
                                     u->fields[j].bit_offset,
                                     u->fields[j].bit_size);
            apply_value(u, &u->fields[j], v);
        }
    }

    /* Poll-based disconnect detection for APC 5G models (and any
     * device where the interrupt pipe is disabled): if ALL reports
     * fail, the device is likely disconnected. */
    if (ndone > 0 && nerrs == ndone) {
        ESP_LOGW(TAG, "'%s': all %d reports failed - marking disconnected",
                 u->name, nerrs);
        u->connected = false;
        hid_host_device_close(u->dev);
        u->slot_used = false;
        return;
    }

    derive_status(u);
}

/* ================================================================== */
/* Toggle beeper (per-instance)                                        */
/* ================================================================== */

static void toggle_beeper_i(ups_instance_t *u)
{
    if (u->beeper_idx < 0 || !u->connected || !u->dev) return;

    mapped_field_t *f = &u->fields[u->beeper_idx];
    uint8_t buf[MAX_REPORT_BUF];
    size_t len = sizeof(buf);

    esp_err_t err = hid_class_request_get_report(
        u->dev, HID_REPORT_TYPE_FEATURE, f->report_id, buf, &len);
    if (err != ESP_OK) return;

    int32_t cur = extract_bits(buf + 1, f->bit_offset, f->bit_size);
    int32_t nxt = (cur == 2) ? 1 : 2;
    inject_bits(buf + 1, f->bit_offset, f->bit_size, nxt);

    hid_class_request_set_report(
        u->dev, HID_REPORT_TYPE_FEATURE, f->report_id, buf, len);
}

/* ================================================================== */
/* USB / HID host callbacks and tasks                                  */
/* ================================================================== */

static void iface_cb(hid_host_device_handle_t dev,
                     const hid_host_interface_event_t ev, void *arg)
{
    ups_instance_t *u = (ups_instance_t *)arg;

    switch (ev) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "UPS '%s' disconnected", u->name);
        u->connected = false;
        hid_host_device_close(dev);
        u->slot_used = false;
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID transfer error on '%s'", u->name);
        break;
    default:
        break;
    }
}

static void on_device_event(hid_host_device_handle_t dev,
                            const hid_host_driver_event_t ev)
{
    if (ev != HID_HOST_DRIVER_EVENT_CONNECTED) return;

    ups_instance_t *u = alloc_slot(dev);
    if (!u) {
        ESP_LOGW(TAG, "Max UPS devices (%d) reached, ignoring new device",
                 MAX_UPS_DEVICES);
        return;
    }

    const hid_host_device_config_t cfg = {
        .callback     = iface_cb,
        .callback_arg = u,
    };

    esp_err_t err = hid_host_device_open(dev, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_device_open failed: %s", esp_err_to_name(err));
        u->slot_used = false;
        return;
    }

    /* ---- Retrieve device information ---- */
    hid_host_dev_info_t info;
    if (hid_host_get_device_info(dev, &info) == ESP_OK) {
        u->vid = info.VID;
        u->pid = info.PID;

        char mfr[64], prod[64], ser[64];
        wcstombs(mfr,  info.iManufacturer,  sizeof(mfr));
        wcstombs(prod, info.iProduct,       sizeof(prod));
        wcstombs(ser,  info.iSerialNumber,  sizeof(ser));

        ESP_LOGI(TAG, "UPS '%s' connected: %s %s (VID:%04X PID:%04X serial:%s)",
                 u->name, mfr, prod, info.VID, info.PID, ser);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        set_var_i(u, "device.type",    "ups");
        set_var_i(u, "driver.name",    "esp-nut-usbhid");
        set_var_i(u, "driver.version", "1.0");
        set_var_i(u, "device.mfr",     mfr);
        set_var_i(u, "device.model",   prod);
        set_var_i(u, "device.serial",  ser);
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%04x", info.VID);
        set_var_i(u, "ups.vendorid", tmp);
        snprintf(tmp, sizeof(tmp), "%04x", info.PID);
        set_var_i(u, "ups.productid", tmp);

        /* APC firmware parsing: product string contains "FW:xxx" and
         * optionally "USB FW:yyy" delimiters for dual firmware versions */
        if (info.VID == VID_APC) {
            char *fw = strstr(prod, "FW:");
            if (fw) {
                fw += 3;
                char fwbuf[32];
                int fi = 0;
                while (*fw && *fw != ' ' && fi < (int)sizeof(fwbuf) - 1)
                    fwbuf[fi++] = *fw++;
                fwbuf[fi] = '\0';
                set_var_i(u, "ups.firmware", fwbuf);

                char *usb_fw = strstr(fw, "USB FW:");
                if (usb_fw) {
                    usb_fw += 7;
                    fi = 0;
                    while (*usb_fw && *usb_fw != ' ' &&
                           fi < (int)sizeof(fwbuf) - 1)
                        fwbuf[fi++] = *usb_fw++;
                    fwbuf[fi] = '\0';
                    set_var_i(u, "ups.firmware.aux", fwbuf);
                }
            }
        }

        xSemaphoreGive(s_mutex);
    }

    /* ---- Parse the HID report descriptor ---- */
    size_t desc_len = 0;
    const uint8_t *desc = hid_host_get_report_descriptor(dev, &desc_len);
    if (desc && desc_len > 0) {
        ESP_LOGI(TAG, "'%s': report descriptor %zu bytes", u->name, desc_len);
        parse_descriptor(u, desc, desc_len);
        apc_fix_voltage_range(u);
    } else {
        ESP_LOGW(TAG, "'%s': could not retrieve HID report descriptor", u->name);
    }

    /* Some devices require poll-only mode (no interrupt pipe):
     * - APC 5G models (PID 0x0003, 0x0004): interrupt pipe used for
     *   proprietary protocol, not standard HID events
     * - Legrand: interrupt pipe causes communication failures
     * Polling via control transfers (GET_REPORT) still works.
     * Disconnect is detected when poll_reports() gets all errors. */
    bool skip_interrupt = false;
    if (u->vid == VID_APC &&
        (u->pid == PID_APC_5G_A || u->pid == PID_APC_5G_B))
    {
        ESP_LOGW(TAG, "'%s': APC 5G model - using poll-only mode", u->name);
        skip_interrupt = true;
    }
    if (u->vid == VID_LEGRAND) {
        ESP_LOGW(TAG, "'%s': Legrand UPS - using poll-only mode", u->name);
        skip_interrupt = true;
    }
    if (!skip_interrupt) {
        ESP_ERROR_CHECK(hid_host_device_start(dev));
    }
    u->connected = true;

    ESP_LOGI(TAG, "UPS '%s' ready (%d mapped fields)", u->name, u->nfields);
}

static void device_cb(hid_host_device_handle_t dev,
                      const hid_host_driver_event_t ev, void *arg)
{
    const hid_event_t e = {.handle = dev, .event = ev, .arg = arg};
    if (s_event_q) xQueueSend(s_event_q, &e, 0);
}

/* ---- USB library task ---- */
static void usb_lib_task(void *arg)
{
    const usb_host_config_t cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&cfg));
    xTaskNotifyGive(arg);

    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

/* ---- HID event processing task ---- */
static void hid_task(void *arg)
{
    s_event_q = xQueueCreate(10, sizeof(hid_event_t));
    hid_event_t ev;
    while (true) {
        if (xQueueReceive(s_event_q, &ev, portMAX_DELAY))
            on_device_event(ev.handle, ev.event);
    }
}

/* ---- Poll task: reads HID reports for ALL UPS & updates LED ---- */
static void poll_task(void *arg)
{
    const gpio_config_t btn = {
        .pin_bit_mask = BIT64(CONFIG_NUT_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_NUT_POLL_INTERVAL_MS));

        bool any_connected = false;
        bool any_alert = false;

        for (int i = 0; i < MAX_UPS_DEVICES; i++) {
            ups_instance_t *u = &s_ups[i];
            if (!u->connected) continue;
            any_connected = true;

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            poll_reports(u);
            xSemaphoreGive(s_mutex);

            if (!u->flag_val[FLAG_AC_PRESENT] ||
                !u->flag_val[FLAG_GOOD] ||
                u->flag_val[FLAG_INTERNAL_FAILURE])
                any_alert = true;
        }

        /* Toggle beeper on ALL connected UPS when button pressed */
        if (any_connected && !gpio_get_level(CONFIG_NUT_BUTTON_GPIO)) {
            for (int i = 0; i < MAX_UPS_DEVICES; i++) {
                if (s_ups[i].connected)
                    toggle_beeper_i(&s_ups[i]);
            }
        }

        /* LED: worst-case across all UPS */
        if (!any_connected)
            led_status_set_disconnected();
        else if (any_alert)
            led_status_set_alert();
        else
            led_status_set_ok();
    }
}

/* ================================================================== */
/* Writable-field helpers                                              */
/* ================================================================== */

static const char *s_rw_names[] = {
    "ups.beeper.status",
    "ups.delay.shutdown",
    "ups.delay.start",
    "ups.delay.reboot",
    "battery.charge.low",
    "battery.runtime.low",
    "input.transfer.low",
    "input.transfer.high",
    "input.sensitivity",
    NULL,
};

static bool is_rw_var(const char *name)
{
    for (int i = 0; s_rw_names[i]; i++)
        if (strcmp(name, s_rw_names[i]) == 0) return true;
    return false;
}

static mapped_field_t *find_field_i(ups_instance_t *u, const char *name)
{
    for (int i = 0; i < u->nfields; i++)
        if (u->fields[i].nut_name[0] && strcmp(u->fields[i].nut_name, name) == 0)
            return &u->fields[i];
    return NULL;
}

static int32_t physical_to_logical(const mapped_field_t *f, double phys)
{
    /* Reverse unit exponent */
    int8_t expo = f->unit_exp - unit_expo_offset(f->unit);
    if (expo > 0) {
        for (int8_t i = 0; i < expo; i++) phys /= 10.0;
    } else if (expo < 0) {
        for (int8_t i = 0; i > expo; i--) phys *= 10.0;
    }

    /* Reverse physical-to-logical scaling */
    if (f->have_phys &&
        !(f->physical_min == 0 && f->physical_max == 0) &&
        f->physical_max > f->physical_min &&
        f->logical_max > f->logical_min)
    {
        double factor = (double)(f->logical_max - f->logical_min)
                      / (double)(f->physical_max - f->physical_min);
        phys = (phys - f->physical_min) * factor + f->logical_min;
    }

    return (int32_t)(phys + 0.5);
}

static bool write_field_i(ups_instance_t *u, mapped_field_t *f, int32_t logical)
{
    if (!u->connected || !u->dev) return false;
    if (f->report_type != RPT_FEATURE) return false;

    uint8_t buf[MAX_REPORT_BUF];
    size_t len = sizeof(buf);
    esp_err_t err = hid_class_request_get_report(
        u->dev, HID_REPORT_TYPE_FEATURE, f->report_id, buf, &len);
    if (err != ESP_OK) return false;

    inject_bits(buf + 1, f->bit_offset, f->bit_size, logical);

    err = hid_class_request_set_report(
        u->dev, HID_REPORT_TYPE_FEATURE, f->report_id, buf, len);
    return err == ESP_OK;
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

void hid_ups_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    memset(s_ups, 0, sizeof(s_ups));

    /* USB library task (pinned to core 0) */
    BaseType_t ok;
    ok = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                                 xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    assert(ok == pdTRUE);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(1000));

    /* Install HID host driver */
    const hid_host_driver_config_t drv = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .core_id                = 0,
        .callback               = device_cb,
        .callback_arg            = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&drv));

    /* HID event processing */
    ok = xTaskCreate(hid_task, "hid_evt", 4096, NULL, 2, NULL);
    assert(ok == pdTRUE);

    /* Periodic poll task */
    ok = xTaskCreate(poll_task, "ups_poll", 4096, NULL, 8, NULL);
    assert(ok == pdTRUE);

    ESP_LOGI(TAG, "HID UPS driver initialised - up to %d devices supported",
             MAX_UPS_DEVICES);
}

int hid_ups_get_device_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_UPS_DEVICES; i++)
        if (s_ups[i].connected) n++;
    return n;
}

const char *hid_ups_get_device_name(int idx)
{
    int n = 0;
    for (int i = 0; i < MAX_UPS_DEVICES; i++) {
        if (s_ups[i].connected) {
            if (n == idx) return s_ups[i].name;
            n++;
        }
    }
    return NULL;
}

bool hid_ups_is_device(const char *ups_name)
{
    return find_by_name(ups_name) != NULL;
}

bool hid_ups_is_connected(const char *ups_name)
{
    ups_instance_t *u = find_by_name(ups_name);
    return u && u->connected;
}

ups_status_t hid_ups_get_status(const char *ups_name)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return (ups_status_t){0};
    return (ups_status_t){
        .connected    = u->connected,
        .ac_present   = u->flag_val[FLAG_AC_PRESENT],
        .battery_good = u->flag_val[FLAG_GOOD] &&
                        !u->flag_val[FLAG_INTERNAL_FAILURE],
    };
}

int hid_ups_get_vars(const char *ups_name, nut_var_t *out, int max)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = (u->nvars < max) ? u->nvars : max;
    memcpy(out, u->vars, n * sizeof(nut_var_t));
    xSemaphoreGive(s_mutex);
    return n;
}

bool hid_ups_get_var(const char *ups_name, const char *varname,
                     char *out, size_t max)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return false;

    bool found = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < u->nvars; i++) {
        if (strcmp(u->vars[i].name, varname) == 0) {
            strncpy(out, u->vars[i].value, max - 1);
            out[max - 1] = '\0';
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return found;
}

void hid_ups_toggle_beeper(const char *ups_name)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (u) toggle_beeper_i(u);
}

/* ================================================================== */
/* Instant command support                                             */
/* ================================================================== */

int hid_ups_get_cmdlist(const char *ups_name, nut_cmd_t *out, int max)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return 0;

    int n = 0;

    /* Beeper commands */
    if (u->beeper_idx >= 0) {
        if (n < max) strncpy(out[n++].name, "beeper.toggle", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "beeper.enable", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "beeper.disable", NUT_CMD_NAME_LEN);
    }

    /* Shutdown / load commands */
    bool has_sd = find_field_i(u, "ups.delay.shutdown") != NULL;
    bool has_su = find_field_i(u, "ups.delay.start") != NULL;

    if (has_sd) {
        if (n < max) strncpy(out[n++].name, "load.off", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "shutdown.stayoff", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "shutdown.stop", NUT_CMD_NAME_LEN);
    }
    if (has_su && n < max)
        strncpy(out[n++].name, "load.on", NUT_CMD_NAME_LEN);
    if (has_sd && has_su && n < max)
        strncpy(out[n++].name, "shutdown.return", NUT_CMD_NAME_LEN);

    /* Battery test commands (requires writable Test field) */
    if (u->test_idx >= 0) {
        if (n < max) strncpy(out[n++].name, "test.battery.start.quick", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "test.battery.start.deep", NUT_CMD_NAME_LEN);
        if (n < max) strncpy(out[n++].name, "test.battery.stop", NUT_CMD_NAME_LEN);
    }

    return n;
}

bool hid_ups_instcmd(const char *ups_name, const char *cmdname)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u || !u->connected) return false;

    /* Beeper commands */
    if (strcmp(cmdname, "beeper.toggle") == 0) {
        toggle_beeper_i(u);
        return true;
    }
    if (strcmp(cmdname, "beeper.enable") == 0 && u->beeper_idx >= 0)
        return write_field_i(u, &u->fields[u->beeper_idx], 2);
    if (strcmp(cmdname, "beeper.disable") == 0 && u->beeper_idx >= 0)
        return write_field_i(u, &u->fields[u->beeper_idx], 1);

    /* load.off: immediate shutdown (write 0 to DelayBeforeShutdown) */
    if (strcmp(cmdname, "load.off") == 0) {
        mapped_field_t *f = find_field_i(u, "ups.delay.shutdown");
        return f ? write_field_i(u, f, physical_to_logical(f, 0)) : false;
    }

    /* load.on: immediate startup (write 0 to DelayBeforeStartup) */
    if (strcmp(cmdname, "load.on") == 0) {
        mapped_field_t *f = find_field_i(u, "ups.delay.start");
        return f ? write_field_i(u, f, physical_to_logical(f, 0)) : false;
    }

    /* shutdown.return: shutdown then restart after power returns.
     * CyberPower firmware rounds delays down to the nearest 60 s,
     * so use 60/120 for CPS and 20/30 for everyone else. */
    if (strcmp(cmdname, "shutdown.return") == 0) {
        mapped_field_t *sd = find_field_i(u, "ups.delay.shutdown");
        mapped_field_t *su = find_field_i(u, "ups.delay.start");
        if (!sd || !su) return false;
        bool cps = (u->vid == VID_CYBERPOWER);
        write_field_i(u, sd, physical_to_logical(sd, cps ? 60 : 20));
        write_field_i(u, su, physical_to_logical(su, cps ? 120 : 30));
        return true;
    }

    /* shutdown.stayoff: shutdown, do not restart.
     * Same CPS delay adjustment as above. */
    if (strcmp(cmdname, "shutdown.stayoff") == 0) {
        mapped_field_t *sd = find_field_i(u, "ups.delay.shutdown");
        if (!sd) return false;
        bool cps = (u->vid == VID_CYBERPOWER);
        write_field_i(u, sd, physical_to_logical(sd, cps ? 60 : 20));
        mapped_field_t *su = find_field_i(u, "ups.delay.start");
        if (su) write_field_i(u, su, su->logical_max);  /* max = no restart */
        return true;
    }

    /* shutdown.stop: cancel pending shutdown */
    if (strcmp(cmdname, "shutdown.stop") == 0) {
        mapped_field_t *f = find_field_i(u, "ups.delay.shutdown");
        return f ? write_field_i(u, f, f->logical_max) : false;
    }

    /* Battery test commands via HID Test usage (0x84:0x58) */
    if (strcmp(cmdname, "test.battery.start.quick") == 0 && u->test_idx >= 0) {
        /* Write 1 = quick test (10-second) */
        return write_field_i(u, &u->fields[u->test_idx], 1);
    }
    if (strcmp(cmdname, "test.battery.start.deep") == 0 && u->test_idx >= 0) {
        /* Write 2 = deep test (full discharge) */
        return write_field_i(u, &u->fields[u->test_idx], 2);
    }
    if (strcmp(cmdname, "test.battery.stop") == 0 && u->test_idx >= 0) {
        /* Write 3 = abort test */
        return write_field_i(u, &u->fields[u->test_idx], 3);
    }

    return false;
}

/* ================================================================== */
/* Writable variable support                                           */
/* ================================================================== */

int hid_ups_get_rw_vars(const char *ups_name, nut_var_t *out, int max)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return 0;

    int n = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < u->nfields && n < max; i++) {
        if (u->fields[i].nut_name[0] &&
            u->fields[i].report_type == RPT_FEATURE &&
            is_rw_var(u->fields[i].nut_name))
        {
            char val[NUT_VAR_VALUE_LEN] = "";
            get_var_i(u, u->fields[i].nut_name, val, sizeof(val));
            strncpy(out[n].name, u->fields[i].nut_name, NUT_VAR_NAME_LEN - 1);
            out[n].name[NUT_VAR_NAME_LEN - 1] = '\0';
            strncpy(out[n].value, val, NUT_VAR_VALUE_LEN - 1);
            out[n].value[NUT_VAR_VALUE_LEN - 1] = '\0';
            n++;
        }
    }
    xSemaphoreGive(s_mutex);
    return n;
}

bool hid_ups_set_var(const char *ups_name, const char *varname, const char *value)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u || !u->connected) return false;

    mapped_field_t *f = find_field_i(u, varname);
    if (!f || f->report_type != RPT_FEATURE || !is_rw_var(varname))
        return false;

    /* Beeper status: enumerated string value */
    if (strcmp(varname, "ups.beeper.status") == 0) {
        int32_t v;
        if (strcmp(value, "disabled") == 0) v = 1;
        else if (strcmp(value, "enabled") == 0) v = 2;
        else if (strcmp(value, "muted") == 0) v = 3;
        else return false;
        return write_field_i(u, f, v);
    }

    /* Input sensitivity: CyberPower enumerated value */
    if (strcmp(varname, "input.sensitivity") == 0) {
        int32_t v;
        if (strcmp(value, "low") == 0) v = 0;
        else if (strcmp(value, "normal") == 0) v = 1;
        else if (strcmp(value, "high") == 0) v = 2;
        else return false;
        return write_field_i(u, f, v);
    }

    /* Numeric value */
    double phys = atof(value);
    int32_t logical = physical_to_logical(f, phys);
    return write_field_i(u, f, logical);
}

bool hid_ups_is_rw_var(const char *ups_name, const char *varname)
{
    ups_instance_t *u = find_by_name(ups_name);
    if (!u) return false;
    mapped_field_t *f = find_field_i(u, varname);
    return f && f->report_type == RPT_FEATURE && is_rw_var(varname);
}
