/*
 * Copyright (c) 2020 Stephane Dorre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMA421_BMA421_H_
#define ZEPHYR_DRIVERS_SENSOR_BMA421_BMA421_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/bma421.h>
#include "bma4_defs.h"

#define BMA421_I2C_ADDRESS		DT_INST_REG_ADDR(0)

struct bma421_data {
	struct device *i2c;

	struct bma4_dev dev;
	struct bma4_accel_config accel_cfg;
	struct bma4_accel accel_data;
	uint32_t step_counter;
	uint32_t temperature;
	
#ifdef CONFIG_BMA421_TRIGGER
	// struct device *dev;
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

	struct sensor_trigger any_motion_trigger;
	sensor_trigger_handler_t any_motion_handler;

	struct sensor_trigger no_motion_trigger;
	sensor_trigger_handler_t no_motion_handler;

	struct sensor_trigger step_counter_trigger;
	sensor_trigger_handler_t step_counter_handler;

	struct sensor_trigger step_detection_trigger;
	sensor_trigger_handler_t step_detection_handler;

#if defined(CONFIG_BMA421_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_BMA421_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_BMA421_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device* internal_dev;
#endif

#endif /* CONFIG_BMA421_TRIGGER */
};

#ifdef CONFIG_BMA421_TRIGGER
int bma421_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int bma421_attr_set(struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val);

int bma421_init_interrupt(struct device *dev);
#endif /* CONFIG_BMA421_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_BMA421_BMA421_H_ */
