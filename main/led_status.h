#pragma once

#include <stdbool.h>

void led_status_init(void);
void led_status_set_ok(void);
void led_status_set_alert(void);
void led_status_set_disconnected(void);
void led_status_set_client(void);
