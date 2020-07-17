/*
 * Copyright (c) 2020 Stephane Dorre
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma421

#include <drivers/i2c.h>
#include <init.h>
#include <logging/log.h>

#include "bma421.h"
#include "bma421_features.h"

LOG_MODULE_REGISTER(BMA421, CONFIG_SENSOR_LOG_LEVEL);

static int8_t bm421_i2c_read(uint8_t reg_addr, uint8_t *read_data, 
				uint32_t len, void *intf_ptr)
{
	struct bma421_data *drv_data = intf_ptr;
	// LOG_INF("bm421_i2c_read 0x%x len %d", reg_addr, len);
	return i2c_burst_read(drv_data->i2c, BMA421_I2C_ADDRESS,
				reg_addr, read_data, len);
}

static int8_t bm421_i2c_write(uint8_t reg_addr, const uint8_t *read_data,
				uint32_t len, void *intf_ptr)
{
	struct bma421_data *drv_data = intf_ptr;
	// LOG_INF("bm421_i2c_write 0x%x len %d", reg_addr, len);
	return i2c_burst_write(drv_data->i2c, BMA421_I2C_ADDRESS,
				reg_addr, read_data, len);
}

static void bma421_delay_us(uint32_t period, void *intf_ptr)
{
	// LOG_INF("bma421_delay_us %d us", period);
	k_busy_wait(period);
}

static int bma421_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bma421_data *drv_data = dev->driver_data;
	int8_t res = 0;
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		res = bma4_read_accel_xyz(&drv_data->accel_data, &drv_data->dev);
		break;
	case BMA421_CHAN_STEP_COUNTER:
		res = bma421_step_counter_output(&drv_data->step_counter, &drv_data->dev);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		res = bma4_get_temperature(&drv_data->temperature, BMA4_DEG, &drv_data->dev);
		break;
	case SENSOR_CHAN_ALL:
		res = bma4_read_accel_xyz(&drv_data->accel_data, &drv_data->dev);
		res = bma421_step_counter_output(&drv_data->step_counter, &drv_data->dev);
		res = bma4_get_temperature(&drv_data->temperature, BMA4_DEG, &drv_data->dev);
		break;
	default:
		return -ENOTSUP;
	}

	if (res != 0) {
		LOG_DBG("Could not read data");
		return -EIO;
	}

	return 0;
}

/*! @brief Converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[out] val      : converted sensor value.
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 */
static void bma421_channel_accel_convert(struct sensor_value *val,
					uint16_t raw_val,
					float g_range,
					uint8_t bit_width)
{
	float half_scale = (float)(1 << bit_width) / 2.0f;
	float accel = SENSOR_G * raw_val * g_range / half_scale;

	accel *= 1000000;
	val->val1 = ((uint64_t)accel) / 1000000;
	val->val2 = ((uint64_t)accel) % 1000000;

	/* normalize val to make sure val->val2 is positive */
	if (val->val2 < 0) {
		val->val1 -= 1;
		val->val2 += 1000000;
	}
}

static int bma421_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bma421_data *drv_data = dev->driver_data;

	/*
	 * See datasheet "Sensor data" section for
	 * more details on processing sample data.
	 */
	switch((u16_t)chan) {
	case SENSOR_CHAN_ACCEL_X:
		// val->val1 = drv_data->accel_data.x;
		// val->val2 = 0;
		bma421_channel_accel_convert(val,
					drv_data->accel_data.x,
					drv_data->accel_cfg.range,
					drv_data->dev.resolution);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		val->val1 = drv_data->accel_data.y;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		val->val1 = drv_data->accel_data.z;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		val->val1 = drv_data->accel_data.x;
		val->val2 = 0;
		val++;
		val->val1 = drv_data->accel_data.x;
		val->val2 = 0;
		val++;
		val->val1 = drv_data->accel_data.x;
		val->val2 = 0;
		break;
	case BMA421_CHAN_STEP_COUNTER:
		val->val1 = drv_data->step_counter;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		val->val1 = drv_data->temperature / 1000;
		val->val2 = drv_data->temperature % 1000;

		/* normalize val to make sure val->val2 is positive */
		if (val->val2 < 0) {
			val->val1 -= 1;
			val->val2 += 1000;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api bma421_driver_api = {
#if CONFIG_BMA421_TRIGGER
	.attr_set = bma421_attr_set,
	.trigger_set = bma421_trigger_set,
#endif
	.sample_fetch = bma421_sample_fetch,
	.channel_get = bma421_channel_get,
};

int bma421_init_driver(struct device *dev)
{
	struct bma421_data *drv_data = dev->driver_data;
	uint8_t res;

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_DBG("Could not get pointer to %s device",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	memset(&drv_data->dev, 0, sizeof(struct bma4_dev));
	drv_data->dev.intf = BMA4_I2C_INTF;
	drv_data->dev.intf_ptr = drv_data;
	drv_data->dev.variant = BMA42X_VARIANT;
	drv_data->dev.bus_read = bm421_i2c_read;
	drv_data->dev.bus_write = bm421_i2c_write;
	drv_data->dev.delay_us = bma421_delay_us;
	drv_data->dev.read_write_len = 8;
	drv_data->dev.resolution = BMA4_16_BIT_RESOLUTION;

	res = bma421_init(&drv_data->dev);
	if (res)
		LOG_ERR("init failed err %d", res);

	res = bma421_write_config_file(&drv_data->dev);
	if (res)
		LOG_ERR("writing config failed err %d", res);

	res = bma4_set_accel_enable(1, &drv_data->dev);
	if (res)
		LOG_ERR("Accel enable failed err %d", res);

	res = bma4_get_accel_config(&drv_data->accel_cfg, &drv_data->dev);
	if (res)
		LOG_ERR("Failed to get Acceleration config err %d", res);

	drv_data->accel_cfg.range = BMA4_ACCEL_RANGE_2G;
	drv_data->accel_cfg.perf_mode = BMA4_CONTINUOUS_MODE;

	/*
	When perf mode disabled, ODR must be set to min 50Hz for most features
	and min 200Hz for double feature
	*/
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

	res = bma4_set_accel_config(&drv_data->accel_cfg, &drv_data->dev);
	if (res)
		LOG_ERR("Failed to set Acceleration config err %d", res);

	res = bma421_feature_enable(BMA421_STEP_CNTR, 1, &drv_data->dev);
	if (res)
		LOG_ERR("cannot activate stepcounter err %d", res);

#ifdef CONFIG_BMA421_TRIGGER
	if (bma421_init_interrupt(dev) < 0) {
		LOG_DBG("Could not initialize interrupts");
		return -EIO;
	}
#endif

	return 0;
}

struct bma421_data bma421_driver;

DEVICE_AND_API_INIT(bma421, "bma421",
		    bma421_init_driver, &bma421_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &bma421_driver_api);
