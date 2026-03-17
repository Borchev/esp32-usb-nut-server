/*
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

#include "led_status.h"
#include "led_strip.h"

static led_strip_handle_t s_strip;

void led_status_init(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = CONFIG_NUT_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt, &s_strip));
    led_strip_clear(s_strip);
}

void led_status_set_ok(void)
{
    led_strip_set_pixel(s_strip, 0, 0x00, 0x03, 0x00);
    led_strip_refresh(s_strip);
}

void led_status_set_alert(void)
{
    led_strip_set_pixel(s_strip, 0, 0x10, 0x00, 0x00);
    led_strip_refresh(s_strip);
}

void led_status_set_disconnected(void)
{
    led_strip_set_pixel(s_strip, 0, 0x08, 0x05, 0x00);
    led_strip_refresh(s_strip);
}

void led_status_set_client(void)
{
    led_strip_set_pixel(s_strip, 0, 0x01, 0x01, 0x01);
    led_strip_refresh(s_strip);
}
