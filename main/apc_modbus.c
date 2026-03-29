/*
 * APC Modbus-over-USB-HID transport for ESP32.
 *
 * Tunnels standard Modbus RTU frames inside HID reports.
 * Sends via USB interrupt OUT, receives via interrupt IN callback.
 *
 * Register map based on NUT's apc_modbus driver (reverse-engineered).
 *
 * References:
 *   https://github.com/networkupstools/nut  drivers/apc_modbus.c
 *   Modbus Application Protocol Specification V1.1b3
 */

#include "apc_modbus.h"
#include "hid_ups.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"

static const char *TAG = "apc-modbus";

/* ================================================================== */
/* Constants                                                           */
/* ================================================================== */

#define MODBUS_SLAVE_ID      1
#define MODBUS_READ_REGS     0x03   /* Function code: Read Holding Registers */

#define HID_REPORT_SIZE      64     /* Report ID + payload (as used by ESP-IDF HID host) */
#define HID_PAYLOAD_SIZE     63
#define MODBUS_BUF_SIZE      320    /* 5 reports max for multi-report reassembly */
#define MAX_READ_REPORTS     8      /* Max HID reports per Modbus response */
#define INTER_FRAME_MS       35     /* Minimum gap between Modbus frames */

/* Default report IDs (discovered from descriptor or hardcoded) */
#define DEFAULT_RX_RID       0x90   /* Host → device */
#define DEFAULT_TX_RID       0x89   /* Device → host */

/* ================================================================== */
/* Register block addresses and counts                                 */
/* ================================================================== */

#define BLK_STATUS_ADDR       0
#define BLK_STATUS_COUNT      27

#define BLK_DYNAMIC_ADDR      128
#define BLK_DYNAMIC_COUNT     32

#define BLK_CONFIG_ADDR       1026
#define BLK_CONFIG_COUNT      22

#define BLK_INVENTORY_ADDR    516
#define BLK_INVENTORY_COUNT   80   /* 516-595, enough for key fields */

/* ================================================================== */
/* Status block offsets (relative to BLK_STATUS_ADDR = 0)              */
/* ================================================================== */

#define ST_UPSSTATUS_HI       0    /* Reg 0: UPSStatus bits 31-16 */
#define ST_UPSSTATUS_LO       1    /* Reg 1: UPSStatus bits 15-0 */
#define ST_TRANSFER_REASON    2    /* Reg 2: input.transfer.reason */
#define ST_SIMPLE_SIGNAL      18   /* Reg 18: SimpleSignalingStatus */
#define ST_TEST_RESULT        23   /* Reg 23: ups.test.result */

/* UPSStatus_BF bit positions (32-bit, regs 0-1) */
#define UPSSTAT_ONLINE        (1U << 1)
#define UPSSTAT_ON_BATTERY    (1U << 2)
#define UPSSTAT_BYPASS        (1U << 3)
#define UPSSTAT_OUTPUT_OFF    (1U << 4)
#define UPSSTAT_FAULT         (1U << 5)
#define UPSSTAT_INPUT_BAD     (1U << 6)
#define UPSSTAT_TEST          (1U << 7)
#define UPSSTAT_HE            (1U << 13)
#define UPSSTAT_OVERLOAD      (1U << 21)

/* ================================================================== */
/* Dynamic block offsets (relative to BLK_DYNAMIC_ADDR = 128)          */
/* ================================================================== */

#define DYN_RUNTIME_HI        0    /* Reg 128: battery.runtime high word */
#define DYN_RUNTIME_LO        1    /* Reg 129: battery.runtime low word */
#define DYN_CHARGE            2    /* Reg 130: battery.charge (scale 9) */
#define DYN_BATT_VOLTAGE      3    /* Reg 131: battery.voltage (scale 5, signed) */
#define DYN_BATT_TEMP         7    /* Reg 135: battery.temperature (scale 7, signed) */
#define DYN_LOAD              8    /* Reg 136: ups.load (scale 8) */
#define DYN_POWER             10   /* Reg 138: ups.power (scale 8, % of nominal) */
#define DYN_OUTPUT_CURRENT    12   /* Reg 140: output.current (scale 5) */
#define DYN_OUTPUT_VOLTAGE    14   /* Reg 142: output.voltage (scale 6) */
#define DYN_OUTPUT_FREQ       16   /* Reg 144: output.frequency (scale 7) */
#define DYN_INPUT_STATUS      22   /* Reg 150: InputStatus_BF */
#define DYN_INPUT_VOLTAGE     23   /* Reg 151: input.voltage (scale 6) */
#define DYN_EFFICIENCY        26   /* Reg 154: ups.efficiency (scale 7, signed) */

/* InputStatus_BF bit positions */
#define INPSTAT_BOOST         (1U << 5)
#define INPSTAT_TRIM          (1U << 6)

/* ================================================================== */
/* Config block offsets (relative to BLK_CONFIG_ADDR = 1026)           */
/* ================================================================== */

#define CFG_TRANSFER_HIGH     0    /* Reg 1026: input.transfer.high */
#define CFG_TRANSFER_LOW      1    /* Reg 1027: input.transfer.low */
#define CFG_DELAY_SHUTDOWN    3    /* Reg 1029: ups.delay.shutdown (signed) */
#define CFG_DELAY_START       4    /* Reg 1030: ups.delay.start (signed) */
#define CFG_DELAY_REBOOT_HI   5   /* Reg 1031: ups.delay.reboot high */
#define CFG_DELAY_REBOOT_LO   6   /* Reg 1032: ups.delay.reboot low */

/* ================================================================== */
/* Inventory block offsets (relative to BLK_INVENTORY_ADDR = 516)      */
/* ================================================================== */

#define INV_FIRMWARE          0    /* Regs 516-523: ups.firmware (8 regs = 16 chars) */
#define INV_FIRMWARE_LEN      8
#define INV_MODEL             16   /* Regs 532-547: ups.model (16 regs = 32 chars) */
#define INV_MODEL_LEN         16
#define INV_SERIAL            48   /* Regs 564-571: ups.serial (8 regs = 16 chars) */
#define INV_SERIAL_LEN        8
#define INV_POWER_NOMINAL     72   /* Reg 588: ups.power.nominal (VA) */
#define INV_REALPOWER_NOMINAL 73   /* Reg 589: ups.realpower.nominal (W) */
#define INV_MFR_DATE          75   /* Reg 591: ups.mfr.date (days since 2000-01-01) */
#define INV_BATT_DATE         79   /* Reg 595: battery.date (days since 2000-01-01) */

#define MAX_MODBUS_FAILURES   3

/* ================================================================== */
/* Per-UPS Modbus context                                              */
/* ================================================================== */

typedef struct {
    usb_device_handle_t usb_dev;  /* Raw USB device handle */
    uint8_t  ep_out_addr;         /* Interrupt OUT endpoint address */
    uint16_t ep_out_mps;          /* Interrupt OUT max packet size */
    usb_transfer_t *out_xfer;     /* Pre-allocated interrupt OUT transfer */
    SemaphoreHandle_t send_sem;   /* Semaphore for send completion */
    uint8_t  rx_rid;              /* Report ID: host → device */
    uint8_t  tx_rid;              /* Report ID: device → host */
    bool     available;
    bool     in_use;
    uint16_t nominal_va;          /* Nominal apparent power (VA) */
    uint16_t nominal_w;           /* Nominal real power (W) */
    int      consecutive_failures;
    char     ups_name[32];
    /* Multi-report Modbus response reassembly */
    uint8_t           resp_buf[MODBUS_BUF_SIZE];
    size_t            resp_len;          /* Accumulated Modbus bytes (no RID) */
    bool              resp_complete;     /* True when full response received */
    SemaphoreHandle_t resp_sem;          /* Signalled when response complete */
} modbus_ctx_t;

static modbus_ctx_t s_ctx[MAX_UPS_DEVICES];

static modbus_ctx_t *find_ctx(const char *ups_name)
{
    if (!ups_name) return NULL;
    for (int i = 0; i < MAX_UPS_DEVICES; i++) {
        if (s_ctx[i].in_use && strcmp(s_ctx[i].ups_name, ups_name) == 0)
            return &s_ctx[i];
    }
    return NULL;
}

static modbus_ctx_t *alloc_ctx(const char *ups_name)
{
    for (int i = 0; i < MAX_UPS_DEVICES; i++) {
        if (!s_ctx[i].in_use) {
            memset(&s_ctx[i], 0, sizeof(modbus_ctx_t));
            s_ctx[i].in_use = true;
            strncpy(s_ctx[i].ups_name, ups_name,
                    sizeof(s_ctx[i].ups_name) - 1);
            return &s_ctx[i];
        }
    }
    return NULL;
}

/* ================================================================== */
/* Modbus CRC16 (for request frames)                                   */
/* ================================================================== */

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* ================================================================== */
/* HID report I/O via class requests                                   */
/* ================================================================== */

/* Interrupt OUT transfer completion callback. */
static void out_xfer_cb(usb_transfer_t *xfer)
{
    modbus_ctx_t *ctx = (modbus_ctx_t *)xfer->context;
    xSemaphoreGive(ctx->send_sem);
}

/* Send a Modbus frame via interrupt OUT transfer.
 * APC devices require interrupt OUT for Modbus - SET_REPORT
 * control transfers are silently ignored by the device. */
static int hid_send(modbus_ctx_t *ctx, const uint8_t *modbus_frame,
                    size_t frame_len)
{
    if (!ctx->out_xfer || !ctx->ep_out_addr || frame_len > HID_PAYLOAD_SIZE)
        return -1;

    int xfer_size = ctx->ep_out_mps ? ctx->ep_out_mps : HID_REPORT_SIZE;
    memset(ctx->out_xfer->data_buffer, 0, xfer_size);
    ctx->out_xfer->data_buffer[0] = ctx->rx_rid;
    memcpy(ctx->out_xfer->data_buffer + 1, modbus_frame, frame_len);
    ctx->out_xfer->num_bytes = xfer_size;

    esp_err_t err = usb_host_transfer_submit(ctx->out_xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "'%s': interrupt OUT failed: %s",
                 ctx->ups_name, esp_err_to_name(err));
        return -1;
    }

    /* Wait for transfer completion */
    if (xSemaphoreTake(ctx->send_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "'%s': interrupt OUT timeout", ctx->ups_name);
        return -1;
    }

    if (ctx->out_xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "'%s': interrupt OUT status %d",
                 ctx->ups_name, ctx->out_xfer->status);
        return -1;
    }

    return 0;
}

/* Receive a reassembled Modbus response via the interrupt IN pipe.
 * apc_modbus_on_input_report() accumulates reports and signals
 * the semaphore when the response is complete or on timeout.
 * Returns number of Modbus bytes, or -1 on timeout/error. */
static int hid_receive(modbus_ctx_t *ctx, uint8_t *modbus_buf,
                       size_t buf_max)
{
    /* Wait for response complete signal (set by on_input_report
     * when it detects a complete Modbus frame, or by timeout below) */
    if (xSemaphoreTake(ctx->resp_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGW(TAG, "'%s': timeout waiting for Modbus response",
                 ctx->ups_name);
        return -1;
    }

    if (ctx->resp_len == 0) return -1;

    size_t payload = ctx->resp_len;
    if (payload > buf_max) payload = buf_max;
    memcpy(modbus_buf, ctx->resp_buf, payload);
    return (int)payload;
}

/* ================================================================== */
/* Modbus protocol: read holding registers                             */
/* ================================================================== */

/* Read holding registers (single attempt).  Response may span multiple
 * HID reports; apc_modbus_on_input_report() reassembles them. */
static int modbus_read_registers_once(modbus_ctx_t *ctx, uint16_t addr,
                                       uint16_t count, uint16_t *regs_out)
{
    /* Build request: slave(1) + func(1) + addr(2) + count(2) + crc(2) */
    uint8_t req[8];
    req[0] = MODBUS_SLAVE_ID;
    req[1] = MODBUS_READ_REGS;
    req[2] = (addr >> 8) & 0xFF;
    req[3] = addr & 0xFF;
    req[4] = (count >> 8) & 0xFF;
    req[5] = count & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    /* Reset reassembly state and drain stale semaphore */
    ctx->resp_len = 0;
    ctx->resp_complete = false;
    xSemaphoreTake(ctx->resp_sem, 0);

    if (hid_send(ctx, req, sizeof(req)) < 0) return -1;

    uint8_t resp[MODBUS_BUF_SIZE];
    int rlen = hid_receive(ctx, resp, sizeof(resp));
    if (rlen < 3) {
        ESP_LOGW(TAG, "'%s': short response (%d bytes) for addr %u count %u",
                 ctx->ups_name, rlen, addr, count);
        return -1;
    }

    /* Modbus exception - return -2 to distinguish from transport failure.
     * An exception still proves the transport works. */
    if (resp[1] & 0x80) {
        ESP_LOGW(TAG, "'%s': exception 0x%02X for addr %u count %u",
                 ctx->ups_name, resp[2], addr, count);
        return -2;
    }

    if (resp[0] != MODBUS_SLAVE_ID || resp[1] != MODBUS_READ_REGS) {
        ESP_LOGW(TAG, "'%s': unexpected response: slave=0x%02X func=0x%02X",
                 ctx->ups_name, resp[0], resp[1]);
        return -1;
    }

    uint8_t byte_count = resp[2];
    if (byte_count != count * 2) {
        ESP_LOGW(TAG, "'%s': byte count mismatch: got %u, expected %u",
                 ctx->ups_name, byte_count, count * 2);
        return -1;
    }

    /* APC strips Modbus CRC in USB-HID transport - just check data length */
    size_t expected_len = 3 + byte_count;
    if ((size_t)rlen < expected_len) {
        ESP_LOGW(TAG, "'%s': incomplete: got %d, need %zu",
                 ctx->ups_name, rlen, expected_len);
        return -1;
    }

    /* Extract registers (big-endian) */
    for (int i = 0; i < count; i++)
        regs_out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1];

    return count;
}

/* Read holding registers with one retry on transport failure. */
static int modbus_read_registers(modbus_ctx_t *ctx, uint16_t addr,
                                  uint16_t count, uint16_t *regs_out)
{
    int rc = modbus_read_registers_once(ctx, addr, count, regs_out);
    if (rc == -1) {
        /* Transport failure - retry once after brief delay */
        vTaskDelay(pdMS_TO_TICKS(INTER_FRAME_MS));
        rc = modbus_read_registers_once(ctx, addr, count, regs_out);
    }
    return rc;
}

/* ================================================================== */
/* Value conversion helpers                                            */
/* ================================================================== */

static double reg_to_double(uint16_t reg, int scale)
{
    return (double)reg / (double)(1 << scale);
}

static double reg_to_double_signed(uint16_t reg, int scale)
{
    return (double)(int16_t)reg / (double)(1 << scale);
}

static uint32_t regs_to_u32(const uint16_t *regs)
{
    return ((uint32_t)regs[0] << 16) | regs[1];
}

static int32_t regs_to_s32(const uint16_t *regs)
{
    return (int32_t)(((uint32_t)regs[0] << 16) | regs[1]);
}

static void regs_to_string(const uint16_t *regs, int nregs,
                            char *out, size_t out_max)
{
    size_t pos = 0;
    for (int i = 0; i < nregs && pos + 2 < out_max; i++) {
        char hi = (regs[i] >> 8) & 0xFF;
        char lo = regs[i] & 0xFF;
        if (hi >= 0x20 && hi < 0x7F) out[pos++] = hi;
        if (lo >= 0x20 && lo < 0x7F) out[pos++] = lo;
    }
    out[pos] = '\0';
    while (pos > 0 && out[pos - 1] == ' ') out[--pos] = '\0';
}

/* Days since 2000-01-01 → "YYYY-MM-DD" string */
static void days_to_date(uint16_t days, char *out, size_t max)
{
    if (days == 0 || days == 0xFFFF) {
        snprintf(out, max, "unknown");
        return;
    }
    /* 2000-01-01 = Unix timestamp 946684800 */
    time_t ts = 946684800 + (time_t)days * 86400;
    struct tm tm;
    gmtime_r(&ts, &tm);
    snprintf(out, max, "%04d-%02d-%02d",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
}

static const char *transfer_reason_str(uint16_t code)
{
    switch (code) {
    case 0:  return "noTransfer";
    case 1:  return "highLineVoltage";
    case 2:  return "brownout";
    case 3:  return "blackout";
    case 4:  return "smallMomentarySag";
    case 5:  return "largeMomentarySag";
    case 6:  return "smallMomentarySpike";
    case 7:  return "largeMomentarySpike";
    case 8:  return "selfTest";
    case 9:  return "rateOfVoltageChange";
    case 10: return "inputFrequencyOutOfRange";
    default: return "unknown";
    }
}

static const char *test_result_str(uint16_t val)
{
    if (val & (1 << 0)) return "Done and passed";
    if (val & (1 << 1)) return "Done and warning";
    if (val & (1 << 2)) return "Done and error";
    if (val & (1 << 3)) return "Aborted";
    if (val & (1 << 4)) return "In progress";
    if (val & (1 << 5)) return "No test initiated";
    return "unknown";
}

/* ================================================================== */
/* Block readers                                                       */
/* ================================================================== */

static int read_status_block(modbus_ctx_t *ctx)
{
    uint16_t regs[BLK_STATUS_COUNT];
    if (modbus_read_registers(ctx, BLK_STATUS_ADDR, BLK_STATUS_COUNT,
                               regs) < 0)
        return -1;

    /* Build ups.status string from UPSStatus_BF */
    uint32_t status = regs_to_u32(&regs[ST_UPSSTATUS_HI]);
    uint16_t signal = regs[ST_SIMPLE_SIGNAL];

    char st[64] = "";
    if (status & UPSSTAT_ONLINE)          strcat(st, "OL");
    else if (status & UPSSTAT_ON_BATTERY) strcat(st, "OB");
    else                                   strcat(st, "OL");

    if (signal & (1 << 1))            strcat(st, " LB");
    if (signal & (1 << 2))            strcat(st, " RB");
    if (status & UPSSTAT_OVERLOAD)    strcat(st, " OVER");
    if (status & UPSSTAT_BYPASS)      strcat(st, " BYPASS");
    if (status & UPSSTAT_OUTPUT_OFF)  strcat(st, " OFF");
    if (status & UPSSTAT_TEST)        strcat(st, " TEST");
    if (status & UPSSTAT_HE)          strcat(st, " HE");

    hid_ups_set_var_ext(ctx->ups_name, "ups.status", st);

    hid_ups_set_var_ext(ctx->ups_name, "input.transfer.reason",
                        transfer_reason_str(regs[ST_TRANSFER_REASON]));

    hid_ups_set_var_ext(ctx->ups_name, "ups.test.result",
                        test_result_str(regs[ST_TEST_RESULT]));

    /* Set basic battery.charger.status from status bits alone.
     * read_dynamic_block() refines this using actual charge level. */
    if (status & UPSSTAT_ON_BATTERY)
        hid_ups_set_var_ext(ctx->ups_name, "battery.charger.status",
                            "discharging");
    else
        hid_ups_set_var_ext(ctx->ups_name, "battery.charger.status",
                            "charging");

    return 0;
}

static int read_dynamic_block(modbus_ctx_t *ctx)
{
    uint16_t regs[BLK_DYNAMIC_COUNT];
    if (modbus_read_registers(ctx, BLK_DYNAMIC_ADDR, BLK_DYNAMIC_COUNT,
                               regs) < 0)
        return -1;

    char buf[NUT_VAR_VALUE_LEN];

    /* battery.runtime (seconds, 32-bit unsigned) */
    uint32_t runtime = regs_to_u32(&regs[DYN_RUNTIME_HI]);
    snprintf(buf, sizeof(buf), "%u", (unsigned)runtime);
    hid_ups_set_var_ext(ctx->ups_name, "battery.runtime", buf);

    /* battery.charge (scale 9 = /512) */
    double charge = reg_to_double(regs[DYN_CHARGE], 9);
    if (charge > 100.0) charge = 100.0;
    snprintf(buf, sizeof(buf), "%.1f", charge);
    hid_ups_set_var_ext(ctx->ups_name, "battery.charge", buf);

    /* battery.voltage (scale 5 = /32, signed) */
    double bv = reg_to_double_signed(regs[DYN_BATT_VOLTAGE], 5);
    snprintf(buf, sizeof(buf), "%.2f", bv);
    hid_ups_set_var_ext(ctx->ups_name, "battery.voltage", buf);

    /* battery.temperature (scale 7 = /128, signed, Celsius) */
    double bt = reg_to_double_signed(regs[DYN_BATT_TEMP], 7);
    snprintf(buf, sizeof(buf), "%.1f", bt);
    hid_ups_set_var_ext(ctx->ups_name, "battery.temperature", buf);

    /* ups.load (scale 8 = /256, percent) */
    double load = reg_to_double(regs[DYN_LOAD], 8);
    snprintf(buf, sizeof(buf), "%.1f", load);
    hid_ups_set_var_ext(ctx->ups_name, "ups.load", buf);

    /* ups.realpower (derived: load% * nominal_w) */
    if (ctx->nominal_w > 0) {
        double rp = load / 100.0 * ctx->nominal_w;
        snprintf(buf, sizeof(buf), "%.1f", rp);
        hid_ups_set_var_ext(ctx->ups_name, "ups.realpower", buf);
    }

    /* ups.power (derived: power% * nominal_va) */
    if (ctx->nominal_va > 0) {
        double ap = reg_to_double(regs[DYN_POWER], 8);
        ap = ap / 100.0 * ctx->nominal_va;
        snprintf(buf, sizeof(buf), "%.1f", ap);
        hid_ups_set_var_ext(ctx->ups_name, "ups.power", buf);
    }

    /* output.current (scale 5 = /32) */
    double oc = reg_to_double(regs[DYN_OUTPUT_CURRENT], 5);
    snprintf(buf, sizeof(buf), "%.2f", oc);
    hid_ups_set_var_ext(ctx->ups_name, "output.current", buf);

    /* output.voltage (scale 6 = /64) */
    double ov = reg_to_double(regs[DYN_OUTPUT_VOLTAGE], 6);
    snprintf(buf, sizeof(buf), "%.1f", ov);
    hid_ups_set_var_ext(ctx->ups_name, "output.voltage", buf);

    /* output.frequency (scale 7 = /128) */
    double of_val = reg_to_double(regs[DYN_OUTPUT_FREQ], 7);
    snprintf(buf, sizeof(buf), "%.1f", of_val);
    hid_ups_set_var_ext(ctx->ups_name, "output.frequency", buf);

    /* InputStatus_BF: BOOST/TRIM flags - append to ups.status */
    uint16_t inp_status = regs[DYN_INPUT_STATUS];
    if (inp_status & INPSTAT_BOOST) {
        char st[NUT_VAR_VALUE_LEN];
        if (hid_ups_get_var(ctx->ups_name, "ups.status", st, sizeof(st)) &&
            !strstr(st, "BOOST")) {
            strcat(st, " BOOST");
            hid_ups_set_var_ext(ctx->ups_name, "ups.status", st);
        }
    }
    if (inp_status & INPSTAT_TRIM) {
        char st[NUT_VAR_VALUE_LEN];
        if (hid_ups_get_var(ctx->ups_name, "ups.status", st, sizeof(st)) &&
            !strstr(st, "TRIM")) {
            strcat(st, " TRIM");
            hid_ups_set_var_ext(ctx->ups_name, "ups.status", st);
        }
    }

    /* input.voltage (scale 6 = /64, 0xFFFF = N/A) */
    if (regs[DYN_INPUT_VOLTAGE] != 0xFFFF) {
        double iv = reg_to_double(regs[DYN_INPUT_VOLTAGE], 6);
        snprintf(buf, sizeof(buf), "%.1f", iv);
        hid_ups_set_var_ext(ctx->ups_name, "input.voltage", buf);
    }

    /* ups.efficiency (scale 7 = /128, signed; negative = special) */
    int16_t eff_raw = (int16_t)regs[DYN_EFFICIENCY];
    if (eff_raw >= 0) {
        double eff = (double)eff_raw / 128.0;
        snprintf(buf, sizeof(buf), "%.1f", eff);
        hid_ups_set_var_ext(ctx->ups_name, "ups.efficiency", buf);
    }

    /* battery.charger.status - derived from ups.status + battery.charge.
     * The HID path gets this from FLAG_CHARGING/DISCHARGING/FULLY_CHARGED
     * flags, but Modbus doesn't expose those directly. */
    {
        char st[NUT_VAR_VALUE_LEN] = "";
        hid_ups_get_var(ctx->ups_name, "ups.status", st, sizeof(st));
        if (strstr(st, "OB"))
            hid_ups_set_var_ext(ctx->ups_name, "battery.charger.status",
                                "discharging");
        else if (charge >= 99.5)
            hid_ups_set_var_ext(ctx->ups_name, "battery.charger.status",
                                "charged");
        else
            hid_ups_set_var_ext(ctx->ups_name, "battery.charger.status",
                                "charging");
    }

    return 0;
}

static void read_config_block(modbus_ctx_t *ctx)
{
    uint16_t regs[BLK_CONFIG_COUNT];
    if (modbus_read_registers(ctx, BLK_CONFIG_ADDR, BLK_CONFIG_COUNT,
                               regs) < 0)
        return;

    char buf[NUT_VAR_VALUE_LEN];

    snprintf(buf, sizeof(buf), "%u", regs[CFG_TRANSFER_HIGH]);
    hid_ups_set_var_ext(ctx->ups_name, "input.transfer.high", buf);

    snprintf(buf, sizeof(buf), "%u", regs[CFG_TRANSFER_LOW]);
    hid_ups_set_var_ext(ctx->ups_name, "input.transfer.low", buf);

    snprintf(buf, sizeof(buf), "%d", (int16_t)regs[CFG_DELAY_SHUTDOWN]);
    hid_ups_set_var_ext(ctx->ups_name, "ups.delay.shutdown", buf);

    snprintf(buf, sizeof(buf), "%d", (int16_t)regs[CFG_DELAY_START]);
    hid_ups_set_var_ext(ctx->ups_name, "ups.delay.start", buf);

    int32_t reboot = regs_to_s32(&regs[CFG_DELAY_REBOOT_HI]);
    snprintf(buf, sizeof(buf), "%ld", (long)reboot);
    hid_ups_set_var_ext(ctx->ups_name, "ups.delay.reboot", buf);
}

static void read_inventory_block(modbus_ctx_t *ctx)
{
    uint16_t regs[BLK_INVENTORY_COUNT];
    if (modbus_read_registers(ctx, BLK_INVENTORY_ADDR, BLK_INVENTORY_COUNT,
                               regs) < 0)
        return;

    char buf[NUT_VAR_VALUE_LEN];

    /* ups.firmware (8 regs = 16 chars) */
    regs_to_string(&regs[INV_FIRMWARE], INV_FIRMWARE_LEN,
                   buf, sizeof(buf));
    if (buf[0]) hid_ups_set_var_ext(ctx->ups_name, "ups.firmware", buf);

    /* ups.model (16 regs = 32 chars) */
    regs_to_string(&regs[INV_MODEL], INV_MODEL_LEN, buf, sizeof(buf));
    if (buf[0]) {
        hid_ups_set_var_ext(ctx->ups_name, "device.model", buf);
        hid_ups_set_var_ext(ctx->ups_name, "ups.model", buf);
    }

    /* ups.serial (8 regs = 16 chars) */
    regs_to_string(&regs[INV_SERIAL], INV_SERIAL_LEN, buf, sizeof(buf));
    if (buf[0]) {
        hid_ups_set_var_ext(ctx->ups_name, "device.serial", buf);
        hid_ups_set_var_ext(ctx->ups_name, "ups.serial", buf);
    }

    /* ups.power.nominal */
    ctx->nominal_va = regs[INV_POWER_NOMINAL];
    if (ctx->nominal_va > 0) {
        snprintf(buf, sizeof(buf), "%u", ctx->nominal_va);
        hid_ups_set_var_ext(ctx->ups_name, "ups.power.nominal", buf);
    }

    /* ups.realpower.nominal */
    ctx->nominal_w = regs[INV_REALPOWER_NOMINAL];
    if (ctx->nominal_w > 0) {
        snprintf(buf, sizeof(buf), "%u", ctx->nominal_w);
        hid_ups_set_var_ext(ctx->ups_name, "ups.realpower.nominal", buf);
    }

    /* ups.mfr.date */
    days_to_date(regs[INV_MFR_DATE], buf, sizeof(buf));
    hid_ups_set_var_ext(ctx->ups_name, "ups.mfr.date", buf);

    /* battery.date */
    days_to_date(regs[INV_BATT_DATE], buf, sizeof(buf));
    hid_ups_set_var_ext(ctx->ups_name, "battery.date", buf);
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

bool apc_modbus_init(usb_device_handle_t usb_dev,
                     uint8_t ep_out_addr, uint16_t ep_out_mps,
                     uint8_t rx_rid, uint8_t tx_rid,
                     const char *ups_name)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    if (ctx) {
        ctx->available = false;
    } else {
        ctx = alloc_ctx(ups_name);
        if (!ctx) {
            ESP_LOGE(TAG, "No free Modbus context slots");
            return false;
        }
    }

    ctx->usb_dev = usb_dev;
    ctx->ep_out_addr = ep_out_addr;
    ctx->ep_out_mps = ep_out_mps;
    ctx->rx_rid = rx_rid ? rx_rid : DEFAULT_RX_RID;
    ctx->tx_rid = tx_rid ? tx_rid : DEFAULT_TX_RID;
    ctx->available = false;
    ctx->consecutive_failures = 0;

    if (!ctx->resp_sem)
        ctx->resp_sem = xSemaphoreCreateBinary();

    if (!ctx->send_sem)
        ctx->send_sem = xSemaphoreCreateBinary();

    /* Allocate interrupt OUT transfer buffer */
    if (!ctx->out_xfer && ep_out_addr) {
        int xfer_size = ep_out_mps ? ep_out_mps : HID_REPORT_SIZE;
        esp_err_t err = usb_host_transfer_alloc(xfer_size, 0, &ctx->out_xfer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "'%s': failed to alloc OUT transfer: %s",
                     ups_name, esp_err_to_name(err));
            ctx->in_use = false;
            return false;
        }
        ctx->out_xfer->device_handle = usb_dev;
        ctx->out_xfer->bEndpointAddress = ep_out_addr;
        ctx->out_xfer->callback = out_xfer_cb;
        ctx->out_xfer->context = ctx;
    }

    if (!ep_out_addr) {
        ESP_LOGW(TAG, "'%s': no interrupt OUT endpoint found", ups_name);
        ctx->in_use = false;
        return false;
    }

    ESP_LOGI(TAG, "'%s': probing Modbus (RX RID=0x%02X, TX RID=0x%02X)",
             ups_name, ctx->rx_rid, ctx->tx_rid);

    /* Give the device time to settle */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Probe: any valid Modbus response (including exceptions) proves
     * the transport works.  Try config block first (most reliable). */
    uint16_t probe;
    static const uint16_t probe_addrs[] = {1026, 0, 128, 516};
    int probe_rc = -1;
    for (int i = 0; i < (int)(sizeof(probe_addrs) / sizeof(probe_addrs[0]));
         i++) {
        probe_rc = modbus_read_registers(ctx, probe_addrs[i], 1, &probe);
        if (probe_rc != -1) break;  /* -2 (exception) also proves transport */
    }

    if (probe_rc == -1) {
        ESP_LOGW(TAG, "'%s': no Modbus response from device", ups_name);
        ctx->in_use = false;
        return false;
    }

    ctx->available = true;
    ESP_LOGI(TAG, "'%s': Modbus transport active", ups_name);
    return true;
}

bool apc_modbus_available(const char *ups_name)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    return ctx && ctx->available;
}

void apc_modbus_read_inventory(const char *ups_name)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    if (!ctx || !ctx->available) return;

    ESP_LOGI(TAG, "'%s': reading inventory registers", ups_name);
    read_inventory_block(ctx);
    vTaskDelay(pdMS_TO_TICKS(INTER_FRAME_MS));
    read_config_block(ctx);
}

void apc_modbus_poll(const char *ups_name)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    if (!ctx || !ctx->available) return;

    bool ok = false;
    if (read_status_block(ctx) >= 0) ok = true;
    vTaskDelay(pdMS_TO_TICKS(INTER_FRAME_MS));
    if (read_dynamic_block(ctx) >= 0) ok = true;

    if (ok) {
        ctx->consecutive_failures = 0;
    } else {
        ctx->consecutive_failures++;
        if (ctx->consecutive_failures >= MAX_MODBUS_FAILURES) {
            ESP_LOGW(TAG, "'%s': Modbus failed %d times, disabling",
                     ups_name, ctx->consecutive_failures);
            ctx->available = false;
        }
    }
}

void apc_modbus_close(const char *ups_name)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    if (!ctx) return;
    ESP_LOGI(TAG, "'%s': closing Modbus context", ups_name);
    ctx->available = false;
    ctx->in_use = false;
    if (ctx->out_xfer) {
        usb_host_transfer_free(ctx->out_xfer);
        ctx->out_xfer = NULL;
    }
    if (ctx->send_sem) {
        vSemaphoreDelete(ctx->send_sem);
        ctx->send_sem = NULL;
    }
    if (ctx->resp_sem) {
        vSemaphoreDelete(ctx->resp_sem);
        ctx->resp_sem = NULL;
    }
}

/* Called from HID interrupt callback for each input report.
 * Accumulates Modbus payload (stripping report ID) across multiple
 * HID reports and signals the semaphore when the response is complete.
 *
 * Completeness check:
 *   - Exception response (func | 0x80): complete after first report
 *   - Normal read response: complete when we have 3 + byte_count bytes */
void apc_modbus_on_input_report(const char *ups_name,
                                const uint8_t *data, size_t len)
{
    modbus_ctx_t *ctx = find_ctx(ups_name);
    if (!ctx || !ctx->resp_sem || len <= 1) return;
    if (data[0] != ctx->tx_rid) return;

    /* Payload = everything after the report ID byte */
    const uint8_t *payload = data + 1;
    size_t plen = len - 1;

    /* Append to reassembly buffer */
    size_t space = sizeof(ctx->resp_buf) - ctx->resp_len;
    if (plen > space) plen = space;
    memcpy(ctx->resp_buf + ctx->resp_len, payload, plen);
    ctx->resp_len += plen;

    /* Check if the Modbus response is complete */
    bool complete = false;
    if (ctx->resp_len >= 3) {
        if (ctx->resp_buf[1] & 0x80) {
            /* Exception response: slave + (func|0x80) + exception_code */
            complete = true;
        } else {
            /* Normal response: slave + func + byte_count + data[byte_count] */
            size_t expected = 3 + (size_t)ctx->resp_buf[2];
            if (ctx->resp_len >= expected)
                complete = true;
        }
    }

    if (complete) {
        ctx->resp_complete = true;
        xSemaphoreGive(ctx->resp_sem);
    }
}
