/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdbool.h>
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "display.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"

#include <drivers/sensor/cst816s.h>

/* ********** defines ********** */
#define BAT_CHA 12
#define BTN_PORT DT_GPIO_LABEL(DT_ALIAS(sw0), gpios)
#define BTN_IN  DT_GPIO_PIN(DT_ALIAS(sw0), gpios)
#define BTN_OUT 15
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
#define PULL_UP DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)
#define TOUCH_PORT CONFIG_CST816S_NAME
#define DISPLAY_TIMEOUT K_SECONDS(5)
#define BT_TOGGLE_LOCK_TIMEOUT K_SECONDS(3)
/* ********** ******* ********** */

/* ********** variables ********** */
static struct k_timer display_off_timer;
static struct k_timer bt_toggle_timer;
static struct device *charging_dev;
static struct gpio_callback charging_cb;
static struct device *button_dev;
static struct gpio_callback button_cb;
static struct device *touch_dev;
static struct sensor_trigger tap = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

static enum cst816s_gesture gesture;
static struct sensor_value touch_point;
/* ********** ********* ********** */

/* ********** init function ********** */
void event_handler_init()
{
	/* Initialize GPIOs */
	charging_dev = device_get_binding("GPIO_0");
	gpio_pin_configure(charging_dev, BAT_CHA, GPIO_INPUT | GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&charging_cb, battery_charging_isr, BIT(BAT_CHA));
	button_dev = device_get_binding(BTN_PORT);
	gpio_pin_configure(button_dev, BTN_IN, GPIO_INPUT | GPIO_INT_EDGE_FALLING | PULL_UP);
	gpio_init_callback(&button_cb, button_pressed_isr, BIT(BTN_IN));
	touch_dev = device_get_binding(DT_LABEL(DT_INST(0, hynitron_cst816s)));

	/* Enable GPIOs */
	gpio_add_callback(charging_dev, &charging_cb);
	gpio_add_callback(button_dev, &button_cb);
	sensor_trigger_set(touch_dev, &tap, touch_tap_isr);

	/* Set button out pin to high to enable the button */
	uint32_t button_out = 1U;
	gpio_pin_configure(button_dev, BTN_OUT, GPIO_OUTPUT);
	gpio_pin_set_raw(button_dev, BTN_OUT, button_out);

	/* Initialize timers */
	k_timer_init(&display_off_timer, display_off_isr, NULL);
	k_timer_init(&bt_toggle_timer, bt_toggle_unlock_isr, NULL);

	/* Start timers */
	k_timer_start(&display_off_timer, DISPLAY_TIMEOUT, K_NO_WAIT);

	/* Special cases */
	/* Get battery charging status */
	k_sleep(K_MSEC(10));
	uint32_t res =  gpio_pin_get(charging_dev, BAT_CHA);
	battery_update_charging_status(res != 1U);

	/* Show time, date and battery status */
	clock_show_time();
	battery_show_status();

	LOG_DBG("Event handler init: Done");
}
/* ********** ************ ********** */

/* ********** interrupt handlers ********** */
void display_off_isr(struct k_timer *light_off)
{
	backlight_enable(false);
	display_sleep();
}

void bt_toggle_unlock_isr(struct k_timer *bt_toggle)
{
	bt_toggle_unlock();
}

void battery_charging_isr(struct device *gpiobat, struct gpio_callback *cb, uint32_t pins)
{
	uint32_t res = gpio_pin_get(charging_dev, BAT_CHA);
	battery_update_charging_status(res != 1U);
}

void button_pressed_isr(struct device *gpiobtn, struct gpio_callback *cb, uint32_t pins)
{
	backlight_enable(true);
	k_timer_start(&display_off_timer, DISPLAY_TIMEOUT, K_NO_WAIT);
	display_wake_up();
	clock_increment_local_time();
	clock_show_time();
	battery_show_status();
	if (bt_toggle_is_locked()) {
		return;
	}
	k_timer_start(&bt_toggle_timer, BT_TOGGLE_LOCK_TIMEOUT, K_NO_WAIT);

	if (bt_mode()) {
		gfx_bt_set_label(BT_ADVERTISING_OFF);
		bt_off();
	} else {
		gfx_bt_set_label(BT_ADVERTISING_ON);
		bt_on();
	}
	gfx_update();
}

void touch_tap_isr(struct device *touch_dev, struct sensor_trigger *tap)
{
	if (sensor_sample_fetch(touch_dev) < 0) {
		LOG_ERR("Touch sample update error.");
	}
	struct sensor_value gesture_val;
	sensor_channel_get(touch_dev, CST816S_CHAN_GESTURE, &gesture_val);
	gesture = gesture_val.val1;

	sensor_channel_get(touch_dev, CST816S_CHAN_TOUCH_POINT_1, &touch_point);
	LOG_INF("Gesture %d on x=%d, y=%d", gesture, touch_point.val1, touch_point.val2);

	backlight_enable(true);
	k_timer_start(&display_off_timer, DISPLAY_TIMEOUT, K_NO_WAIT);
	display_wake_up();
	clock_increment_local_time();
	clock_show_time();
	battery_show_status();
	gfx_update();
}

/* ********** ************** ********** */
