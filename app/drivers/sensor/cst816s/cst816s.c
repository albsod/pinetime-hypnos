/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hynitron_cst816s

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
	uint8_t buf[255];
	uint8_t msb;
	uint8_t lsb;
	//uint8_t id = 0U;
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	uint8_t mask = CST816S_MOTION_EN_DCLICK;
	uint8_t ret = i2c_reg_write_byte(drv_data->i2c, CST816S_I2C_ADDRESS,
				CST816S_REG_MOTION_MASK, mask);

	LOG_WRN("Writing config double click enable : %d", ret);
	/*
	 * since all accel data register addresses are consecutive,
	 * a burst read can be used to read all the samples
	 */
	if (i2c_burst_read(drv_data->i2c, CST816S_I2C_ADDRESS,
				CST816S_REG_DATA, buf, 255) < 0) {
		LOG_DBG("Could not read data");
		return -EIO;
	}

	switch (buf[CST816S_REG_GESTURE_ID])  {
		case CST816S_GESTURE_NONE:
			drv_data->gesture = NONE;
			break;
		case CST816S_GESTURE_SLIDE_UP:
			drv_data->gesture = SLIDE_UP;
			break;
		case CST816S_GESTURE_SLIDE_DOWN:
			drv_data->gesture = SLIDE_DOWN;
			break;
		case CST816S_GESTURE_SLIDE_LEFT:
			drv_data->gesture = SLIDE_LEFT;
			break;
		case CST816S_GESTURE_SLIDE_RIGHT:
			drv_data->gesture = SLIDE_RIGHT;
			break;
		case CST816S_GESTURE_CLICK:
			drv_data->gesture = CLICK;
			break;
		case CST816S_GESTURE_DOUBLE_CLICK:
			drv_data->gesture = DOUBLE_CLICK;
			break;
		case CST816S_GESTURE_LONG_PRESS:
			drv_data->gesture = LONG_PRESS;
			break;
	}
	
	drv_data->number_touch_point = buf[CST816S_REG_FINGER_NUM];

	msb = buf[CST816S_REG_XPOS_H] & 0x0f;
	lsb = buf[CST816S_REG_XPOS_L];
	drv_data->touch_point_1.x = (msb<<8)|lsb; 

	msb = buf[CST816S_REG_YPOS_H] & 0x0f;
	lsb = buf[CST816S_REG_YPOS_H];
	drv_data->touch_point_1.y = (msb<<8)|lsb; // todo check if buf[5] is indeed Y

	return 0;
}


static int cst816s_channel_get(struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct cst816s_data *drv_data = dev->driver_data;

	if (chan == CST816S_CHAN_GESTURE) {
		val->val1=drv_data->gesture;
	} else if (chan == CST816S_CHAN_TOUCH_POINT_1) {
		val->val1=drv_data->touch_point_1.x;
		val->val2=drv_data->touch_point_1.y;
	} else if (chan == CST816S_CHAN_TOUCH_POINT_2) {
		val->val1=drv_data->touch_point_2.x;
		val->val2=drv_data->touch_point_2.y;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

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
	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_DBG("Could not get pointer to %s device",
				DT_INST_BUS_LABEL(0));
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

DEVICE_AND_API_INIT(cst816s, CONFIG_CST816S_NAME, cst816s_init, &cst816s_driver,
		NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		&cst816s_driver_api);
