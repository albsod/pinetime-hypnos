/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdbool.h>
#include "backlight.h"
#include "battery.h"
#include "bt.h"
#include "clock.h"
#include "event_handler.h"
#include "gfx.h"
#include "log.h"
#include "cts_sync.h"

/* ********** defines ********** */
#define BAT_PERCENTAGE_READ_INTERVAL K_MINUTES(5)
#define BAT_CHA 12
#define BTN_PORT DT_ALIAS_SW0_GPIOS_CONTROLLER
#define BTN_IN  DT_ALIAS_SW0_GPIOS_PIN
#define BTN_OUT 15
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
#define PULL_UP DT_ALIAS_SW0_GPIOS_FLAGS
#define TOUCH_PORT DT_INST_0_HYNITRON_CST816S_LABEL
#define BACKLIGHT_TIMEOUT K_SECONDS(5)
#define BT_TIMEOUT K_SECONDS(10)

/* size of stack area used by threads */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

K_SEM_DEFINE(enable_bt_sem, 0, 1);
K_SEM_DEFINE(disable_bt_sem, 0, 1);
K_THREAD_DEFINE(clock_sync_id, STACKSIZE, bt_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
/* ********** defines ********** */

/* ********** variables ********** */
static struct k_timer battery_percentage_timer;
static struct k_timer clock_tick_timer;
static struct k_timer backlight_off_timer;
static struct k_timer bt_off_timer;
static struct device *charging_dev;
static struct gpio_callback charging_cb;
static struct device *button_dev;
static struct gpio_callback button_cb;
static struct device *touch_dev;
static struct sensor_trigger tap = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};
bool bt_enabled = false;

struct k_sem enable_bt_sem;
struct k_sem disable_bt_sem;
/* ********** variables ********** */

/* ********** init function ********** */
void event_handler_init()
{
        // TODO: Check return values for error handling.

	/* Initialize GPIOs */
        charging_dev = device_get_binding("GPIO_0");
        gpio_pin_configure(charging_dev, BAT_CHA, GPIO_INPUT | GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&charging_cb, battery_charging_isr,
                BIT(BAT_CHA));
	button_dev = device_get_binding(BTN_PORT);
	gpio_pin_configure(button_dev, BTN_IN, GPIO_INPUT | GPIO_INT_EDGE_FALLING | PULL_UP);
	gpio_init_callback(&button_cb, button_pressed_isr, BIT(BTN_IN));
	touch_dev = device_get_binding(TOUCH_PORT);

	/* Enable GPIOs */
	gpio_add_callback(charging_dev, &charging_cb);
	gpio_add_callback(button_dev, &button_cb);
	sensor_trigger_set(touch_dev, &tap, touch_tap_isr);

	/* Set button out pin to high to enable the button */
	u32_t button_out = 1U;
        gpio_pin_configure(button_dev, BTN_OUT, GPIO_OUTPUT);
        gpio_pin_set_raw(button_dev, BTN_OUT, button_out);

	/* Initialize timers */
        k_timer_init(&battery_percentage_timer,
		     battery_percentage_isr, NULL);
	k_timer_init(&clock_tick_timer, clock_tick_isr, NULL);
	k_timer_init(&backlight_off_timer, backlight_off_isr, NULL);
	k_timer_init(&bt_off_timer, bt_off_isr, NULL);

	/* Start timers */
        k_timer_start(&battery_percentage_timer, BAT_PERCENTAGE_READ_INTERVAL,
		      BAT_PERCENTAGE_READ_INTERVAL);
	k_timer_start(&clock_tick_timer, K_SECONDS(1), K_SECONDS(1));
	k_timer_start(&backlight_off_timer, BACKLIGHT_TIMEOUT, K_NO_WAIT);

	/* Special cases */
        /* Get battery charging status */
	k_sleep(K_MSEC(10));
        u32_t res =  gpio_pin_get(charging_dev, BAT_CHA);
        battery_update_charging_status(res != 1U);

        /* Show time, date and battery status */
	clock_show_time();
	battery_show_status();

	LOG_DBG("Event handler init: Done");
}
/* ********** init function ********** */

/* ********** handler functions ********** */
void backlight_off_isr(struct k_timer *light_off)
{
	backlight_enable(false);
}

void battery_percentage_isr(struct k_timer *bat)
{
        battery_update_percentage();
}

void battery_charging_isr(struct device *gpiobat, struct gpio_callback *cb, u32_t pins)
{
	u32_t res = gpio_pin_get(charging_dev, BAT_CHA);
	battery_update_charging_status(res != 1U);
	backlight_enable(true);
	k_timer_start(&backlight_off_timer, BACKLIGHT_TIMEOUT, K_NO_WAIT);
}

void button_pressed_isr(struct device *gpiobtn, struct gpio_callback *cb, u32_t pins)
{
	backlight_enable(true);
	k_timer_start(&backlight_off_timer, BACKLIGHT_TIMEOUT, K_NO_WAIT);
	bt_on();
}

void clock_tick_isr(struct k_timer *tick)
{
	clock_increment_local_time();
	clock_show_time();
	battery_show_status();
}

void touch_tap_isr(struct device *touch_dev, struct sensor_trigger *tap)
{
	backlight_enable(true);
	k_timer_start(&backlight_off_timer, BACKLIGHT_TIMEOUT, K_NO_WAIT);
}

void bt_off_isr(struct k_timer *bt)
{
	bt_off();
}

bool bt_mode(void)
{
	return bt_enabled;
}

/* ********** handler functions ********** */

/* ************* thread functions ***************/

void bt_on(void)
{
	bt_enabled = true;
	k_timer_start(&bt_off_timer, BT_TIMEOUT, K_NO_WAIT);
	k_sem_give(&enable_bt_sem);
}

void bt_await_on(void)
{
	k_sem_take(&enable_bt_sem, K_FOREVER);
}

void bt_off(void)
{
	bt_enabled = false;
	k_sem_give(&disable_bt_sem);
}

void bt_await_off(void)
{
	k_sem_take(&disable_bt_sem, K_FOREVER);
}

void main_thread(void)
{
	while (true) {
	await_disable_bt:
		bt_await_off();
		LOG_INF("Exiting bluetooth mode...");
		gfx_bt_set_label(0);
		lv_task_handler();
		bt_adv_stop();
		while (true) {
			k_sleep(K_MSEC(1));
			k_cpu_idle();
			lv_task_handler();
			if (bt_enabled)
				goto await_disable_bt;
		}
	}
}

void bt_thread(void)
{
	while (true) {
	await_enable_bt:
		bt_await_on();
		LOG_INF("Entering bluetooth mode...");
		gfx_bt_set_label(1);
		lv_task_handler();
		// FIXME: don't do this if already started during init
		bt_adv_start();
		cts_sync_loop();
		while (true) {
			clock_sync_time();
			k_sleep(K_MSEC(10));
			if (!bt_enabled) {
				goto await_enable_bt;
			}
		}
	}
}

/* ***************************************/
