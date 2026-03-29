/*
 * APC Modbus-over-USB-HID transport for ESP32.
 *
 * Tunnels Modbus RTU frames inside HID reports.
 * Sends via interrupt OUT, receives via interrupt IN callback.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "usb/usb_host.h"

/* Initialise Modbus transport for a specific UPS instance.
 * usb_dev:      raw USB device handle (for interrupt OUT transfers)
 * ep_out_addr:  interrupt OUT endpoint address (0 = not available)
 * ep_out_mps:   interrupt OUT max packet size
 * rx_rid/tx_rid: Modbus report IDs (0 = use defaults 0x90/0x89)
 * Returns true if the device responds to Modbus probing. */
bool apc_modbus_init(usb_device_handle_t usb_dev,
                     uint8_t ep_out_addr, uint16_t ep_out_mps,
                     uint8_t rx_rid, uint8_t tx_rid,
                     const char *ups_name);

/* Check if Modbus is active for the named UPS. */
bool apc_modbus_available(const char *ups_name);

/* Read inventory + config registers (call once at connect). */
void apc_modbus_read_inventory(const char *ups_name);

/* Poll status + dynamic registers (call periodically). */
void apc_modbus_poll(const char *ups_name);

/* Release Modbus context for the named UPS. */
void apc_modbus_close(const char *ups_name);

/* Called from HID interrupt callback when an input report arrives.
 * Filters by TX report ID and signals the receive semaphore. */
void apc_modbus_on_input_report(const char *ups_name,
                                const uint8_t *data, size_t len);
