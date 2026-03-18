#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define MAX_UPS_DEVICES   10
#define MAX_NUT_VARS     64
#define NUT_VAR_NAME_LEN 48
#define NUT_VAR_VALUE_LEN 64

typedef struct {
    char name[NUT_VAR_NAME_LEN];
    char value[NUT_VAR_VALUE_LEN];
} nut_var_t;

typedef struct {
    bool connected;
    bool ac_present;
    bool battery_good;
} ups_status_t;

void hid_ups_init(void);

/* Multi-device enumeration */
int         hid_ups_get_device_count(void);
const char *hid_ups_get_device_name(int idx);
bool        hid_ups_is_device(const char *ups_name);

/* Per-device queries (ups_name matches NUT UPS name) */
bool         hid_ups_is_connected(const char *ups_name);
ups_status_t hid_ups_get_status(const char *ups_name);
int          hid_ups_get_vars(const char *ups_name, nut_var_t *out, int max);
bool         hid_ups_get_var(const char *ups_name, const char *varname,
                             char *out, size_t max);
void         hid_ups_toggle_beeper(const char *ups_name);

/* Instant commands */
#define MAX_NUT_CMDS     10
#define NUT_CMD_NAME_LEN 32

typedef struct {
    char name[NUT_CMD_NAME_LEN];
} nut_cmd_t;

int  hid_ups_get_cmdlist(const char *ups_name, nut_cmd_t *out, int max);
bool hid_ups_instcmd(const char *ups_name, const char *cmdname);

/* Writable variables */
int  hid_ups_get_rw_vars(const char *ups_name, nut_var_t *out, int max);
bool hid_ups_set_var(const char *ups_name, const char *varname, const char *value);
bool hid_ups_is_rw_var(const char *ups_name, const char *varname);
