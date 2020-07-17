/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "cst816s.h"



LOG_MODULE_REGISTER(CST816S, CONFIG_SENSOR_LOG_LEVEL);

static int cst816s_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct cst816s_data *drv_data = dev->driver_data;
	uint8_t buf[64];
	uint8_t msb;
	uint8_t lsb;
	//uint8_t id = 0U;
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	/*
	 * since all accel data register addresses are consecutive,
	 * a burst read can be used to read all the samples
	 */
	if (i2c_burst_read(drv_data->i2c, CST816S_I2C_ADDRESS,
				CST816S_REG_DATA, buf, 64) < 0) {
		LOG_DBG("Could not read data");
		return -EIO;
	}
	// bytes 3 to 8 are repeated 10 times
	// byte 3 (MSB bit 3..0)
	// byte 4 (LSB)
	// only first is relevant
	//
	msb = buf[3] & 0x0f;
	lsb = buf[4];
	drv_data->x_sample = (msb<<8)|lsb; 

	msb = buf[5] & 0x0f;
	lsb = buf[6];
	drv_data->y_sample = (msb<<8)|lsb; // todo check if buf[5] is indeed Y


	return 0;
}


static int cst816s_channel_get(struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct cst816s_data *drv_data = dev->driver_data;
	val->val1=drv_data->x_sample;
	val->val2=drv_data->y_sample;

	//if (chan == SENSOR_CHAN_ACCEL_XYZ) {
	//	cst816s_channel_convert(val, drv_data->x_sample);
	//	cst816s_channel_convert(val + 1, drv_data->y_sample);
	//} else {
	//	return -ENOTSUP;
	//}

	return 0;
}


static void cst816s_reset_sensor(struct cst816s_data *data)
{
	LOG_INF("Resetting touch sensor");
	gpio_pin_set_raw(data->dev, CST816S_RESET_PIN, 1);
	k_busy_wait(50000);
	gpio_pin_set_raw(data->dev, CST816S_RESET_PIN, 0);
	k_busy_wait(5000);
	gpio_pin_set_raw(data->dev, CST816S_RESET_PIN, 1);
	k_busy_wait(50000);
}

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static void cst816s_enter_sleep(struct cst816s_data *data)
{
	gpio_pin_set_raw(data->dev, CST816S_RESET_PIN, 0);
	k_busy_wait(5000);
	gpio_pin_set_raw(data->dev, CST816S_RESET_PIN, 1);
	k_busy_wait(50000);
	u8_t cmd[16];
	cmd[0] = CST816S_CMD_ENTER_SLEEP;
	(void)i2c_write(data->dev, cmd, 1, CST816S_REG_POWER_MODE);
}

static int cst816s_pm_control(struct device *dev, u32_t ctrl_command,
			      void *context, device_pm_cb cb, void *arg)
{
	int ret = 0;
	struct cst816s_data *data = (struct cst816s_data *)dev->driver_data;

	switch (ctrl_command) {
	case DEVICE_PM_SET_POWER_STATE:
		if (*((u32_t *)context) == DEVICE_PM_SUSPEND_STATE) {
			cst816s_enter_sleep(data);
			data->pm_state = DEVICE_PM_SUSPEND_STATE;
			ret = 0;
		} else {
			cst816s_reset_sensor(data);
			data->pm_state = DEVICE_PM_ACTIVE_STATE;
			ret = 0;
		}
		break;
	case DEVICE_PM_GET_POWER_STATE:
		*((u32_t *)context) = data->pm_state;
		break;
	default:
		ret = -EINVAL;
	}

	if (cb != NULL) {
		cb(dev, ret, context, arg);
	}
	return ret;
}
#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

static const struct sensor_driver_api cst816s_driver_api = {
#if CONFIG_CST816S_TRIGGER
	.attr_set = cst816s_attr_set,
	.trigger_set = cst816s_trigger_set,
#endif
	.sample_fetch = cst816s_sample_fetch,
	.channel_get = cst816s_channel_get,
};

int cst816s_init(struct device *dev)
{
	struct cst816s_data *drv_data = dev->driver_data;
	//uint8_t id = 0U;
	drv_data->i2c = device_get_binding(CONFIG_CST816S_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		LOG_DBG("Could not get pointer to %s device",
				CONFIG_CST816S_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	/* read device ID */
	//i2c_reg_read_byte(drv_data->i2c, BMA421_I2C_ADDRESS,0x40, &id); 

#ifdef CONFIG_CST816S_TRIGGER
	if (cst816s_init_interrupt(dev) < 0) {
		LOG_DBG("Could not initialize interrupts");
		return -EIO;
	}
#endif
	return 0;
}

struct cst816s_data cst816s_driver;

#ifndef CONFIG_DEVICE_POWER_MANAGEMENT
DEVICE_AND_API_INIT(cst816s, CONFIG_CST816S_NAME, cst816s_init, &cst816s_driver,
		NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		&cst816s_driver_api);
#else
DEVICE_DEFINE(cst816s, CONFIG_CST816S_NAME, cst816s_init, cst816s_pm_control,
	      &cst816s_driver, NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
	      &cst816s_driver_api);
#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */
