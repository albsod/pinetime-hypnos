/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma421

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "bma421.h"
#include "bma421_features.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(BMA421, CONFIG_SENSOR_LOG_LEVEL);

int bma421_attr_set(struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val)
{
	struct bma421_data *drv_data = dev->driver_data;
	switch(attr) {
	case SENSOR_ATTR_SLOPE_TH:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void bma421_gpio_callback(struct device *dev,
				 struct gpio_callback *cb, u32_t pins)
{
	struct bma421_data *drv_data =
		CONTAINER_OF(cb, struct bma421_data, gpio_cb);

	ARG_UNUSED(pins);

#if defined(CONFIG_BMA421_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_BMA421_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void bma421_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct bma421_data *drv_data = dev->driver_data;
	
	uint16_t int_status = 0xffffu;
	bma421_read_int_status(&int_status, &drv_data->dev);

	/* check for data ready */
	if (((int_status & BMA4_ACCEL_DATA_RDY_INT) == BMA4_ACCEL_DATA_RDY_INT)
		&& drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev, &drv_data->data_ready_trigger);
	}

	/* check for any error */
	if (((int_status & BMA421_ERROR_INT) == BMA421_ERROR_INT)) {
		LOG_ERR("Interrupt status 0x%x - Error detected!", int_status);
		// TODO: Handle error (maybe soft reset ?)
	}
}

#ifdef CONFIG_BMA421_TRIGGER_OWN_THREAD
static void bma421_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct bma421_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		bma421_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_BMA421_TRIGGER_GLOBAL_THREAD
static void bma421_work_cb(struct k_work *work)
{
	struct bma421_data *drv_data =
		CONTAINER_OF(work, struct bma421_data, work);

	bma421_thread_cb(drv_data->internal_dev);
}
#endif

int bma421_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct bma421_data *drv_data = dev->driver_data;
	int8_t ret;
	uint16_t interrupt_mask = 0;
	uint8_t interrupt_enable = BMA4_ENABLE;

	if (handler == NULL) {
		interrupt_enable = BMA4_DISABLE;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		interrupt_mask = BMA4_DATA_RDY_INT;
		drv_data->data_ready_handler = handler;
		drv_data->data_ready_trigger = *trig;
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

	// Add Error interrupt in any case.
	interrupt_mask |= BMA421_ERROR_INT;

	ret = bma421_map_interrupt(BMA4_INTR1_MAP, interrupt_mask, interrupt_enable, &drv_data->dev);
	if (ret) {
		LOG_ERR("Map interrupt failed err %d", ret);
	}

	uint16_t int_status = 0xffffu;
	bma421_read_int_status(&int_status, &drv_data->dev);
	LOG_WRN("Reading Interrupt status 0x%x", int_status);

	ret = bma4_set_accel_enable(BMA4_ENABLE, &drv_data->dev);
	if (ret) {
		LOG_ERR("Accel enable failed err %d", ret);
	}
	
	gpio_pin_configure(drv_data->gpio, 
			DT_INST_GPIO_PIN(0, int1_gpios),
			GPIO_INPUT | GPIO_INT_EDGE_FALLING | GPIO_PULL_UP | GPIO_ACTIVE_LOW);

	return 0;
}

int bma421_init_interrupt(struct device *dev)
{
	struct bma421_data *drv_data = dev->driver_data;
	int8_t ret;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(DT_INST_GPIO_LABEL(0, int1_gpios));
	if (drv_data->gpio == NULL) {
		LOG_DBG("Cannot get pointer to %s device",
		    DT_INST_GPIO_LABEL(0, int1_gpios));
		return -EINVAL;
	}

	LOG_WRN("Configuring %s device, pin %d",
		DT_INST_GPIO_LABEL(0, int1_gpios),DT_INST_GPIO_PIN(0, int1_gpios));

	gpio_init_callback(&drv_data->gpio_cb,
			   bma421_gpio_callback,
			   BIT(DT_INST_GPIO_PIN(0, int1_gpios)));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}
		
	uint16_t int_status = 0xffffu;
	bma421_read_int_status(&int_status, &drv_data->dev);
	LOG_WRN("Interrupt status 0x%x", int_status);

	struct bma4_int_pin_config pin_config;
	bma4_get_int_pin_config(&pin_config, BMA4_INTR1_MAP, &drv_data->dev);

	pin_config.output_en = BMA4_OUTPUT_ENABLE;
	pin_config.od = BMA4_OPEN_DRAIN;
	pin_config.lvl = BMA4_ACTIVE_LOW;
	pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
	/* .edge_ctrl and .input_en are for input interrupt configuration */

	ret = bma4_set_int_pin_config(&pin_config, BMA4_INTR1_MAP, &drv_data->dev);
	if (ret) {
		LOG_ERR("Set interrupt config err %d", ret);
	}
	
	/* Latch mode means that interrupt flag are only reset once the status is read */
	bma4_set_interrupt_mode(BMA4_LATCH_MODE, &drv_data->dev);

	uint8_t bma_status = 0xffu;
	bma4_get_status(&bma_status, &drv_data->dev);
	
#if defined(CONFIG_BMA421_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_BMA421_THREAD_STACK_SIZE,
			(k_thread_entry_t)bma421_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_BMA421_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_BMA421_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = bma421_work_cb;
	drv_data->internal_dev = dev;
#endif

	return 0;
}
