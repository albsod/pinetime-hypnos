/*
 * Copyright (c) 2020 Stephane Dorre <stephane.dorre@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hynitron_cst816s

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "cst816s.h"

#include <logging/log.h>

#define INTERRUPT_PIN 	DT_INST_GPIO_PIN(0, int1_gpios)

LOG_MODULE_DECLARE(CST816S, CONFIG_SENSOR_LOG_LEVEL);

int cst816s_attr_set(const struct device *dev,
		enum sensor_channel chan,
		enum sensor_attribute attr,
		const struct sensor_value *val)
{
	//struct cst816s_data *drv_data = dev->driver_data;

	if (chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -ENOTSUP;
	}

	return 0;
}

static void cst816s_gpio_callback(const struct device *dev,
		struct gpio_callback *cb, uint32_t pins)
{
	struct cst816s_data *drv_data =
		CONTAINER_OF(cb, struct cst816s_data, gpio_cb);

	ARG_UNUSED(pins);

#if defined(CONFIG_CST816S_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_CST816S_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void cst816s_thread_cb(const struct device *dev)
{
	struct cst816s_data *drv_data = dev->data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev, &drv_data->data_ready_trigger);
	}
}

#ifdef CONFIG_CST816S_TRIGGER_OWN_THREAD
static void cst816s_thread(struct cst816s_data *drv_data)
{
	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		cst816s_thread_cb(drv_data->dev);
	}
}
#endif

#ifdef CONFIG_CST816S_TRIGGER_GLOBAL_THREAD
static void cst816s_work_cb(struct k_work *work)
{
	struct cst816s_data *drv_data =
		CONTAINER_OF(work, struct cst816s_data, work);

	cst816s_thread_cb(drv_data->dev);
}
#endif

int cst816s_trigger_set(const struct device *dev,
		const struct sensor_trigger *trig,
		sensor_trigger_handler_t handler)
{
	struct cst816s_data *drv_data = dev->data;

	if (trig->type == SENSOR_TRIG_DATA_READY) {

		drv_data->data_ready_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->data_ready_trigger = *trig;
	} 
	else {
		return -ENOTSUP;
	}

	return 0;
}

int cst816s_init_interrupt(const struct device *dev)
{
	struct cst816s_data *drv_data = dev->data;
	
	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(DT_INST_GPIO_LABEL(0, int1_gpios));
	if (drv_data->gpio == NULL) {
		LOG_DBG("Cannot get pointer to %s device",
				DT_INST_GPIO_LABEL(0, int1_gpios));
		return -EINVAL;
	}

	gpio_init_callback(&drv_data->gpio_cb,
			cst816s_gpio_callback,
			BIT(INTERRUPT_PIN));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}

	gpio_pin_configure(drv_data->gpio, 
			INTERRUPT_PIN, GPIO_INPUT | GPIO_PULL_UP 
			| GPIO_INT_EDGE_FALLING | GPIO_ACTIVE_LOW);

#if defined(CONFIG_CST816S_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_CST816S_THREAD_STACK_SIZE,
			(k_thread_entry_t)cst816s_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_CST816S_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_CST816S_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = cst816s_work_cb;
	drv_data->dev = dev;
#endif
	return 0;
}
