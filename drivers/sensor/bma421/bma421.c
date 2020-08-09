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
	return i2c_burst_read(drv_data->i2c, BMA421_I2C_ADDRESS,
				reg_addr, read_data, len);
}

static int8_t bm421_i2c_write(uint8_t reg_addr, const uint8_t *read_data,
				uint32_t len, void *intf_ptr)
{
	struct bma421_data *drv_data = intf_ptr;
	return i2c_burst_write(drv_data->i2c, BMA421_I2C_ADDRESS,
				reg_addr, read_data, len);
}

static void bma421_delay_us(uint32_t period, void *intf_ptr)
{
	k_busy_wait(period);
}

static int bma421_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bma421_data *drv_data = dev->driver_data;
	int8_t ret = 0;
	uint8_t status = 0xffu;

	// uint16_t int_status = 0xffffu;
	// bma421_read_int_status(&int_status, &drv_data->dev);
	// LOG_WRN("Int status 0x%x", int_status);

	// struct bma4_int_pin_config pin_config;
	// bma4_get_int_pin_config(&pin_config, BMA4_INTR1_MAP, &drv_data->dev);

	// LOG_WRN("int config, input_en %d, output_en %d , edge_ctrl %d, od %d, lvl %d",
	// 	pin_config.input_en, pin_config.output_en, pin_config.edge_ctrl, pin_config.od, pin_config.lvl);

	// uint32_t pin_state;
	// gpio_port_get_raw(drv_data->gpio, &pin_state);
	// LOG_WRN("Pin state 0x%x", pin_state);

	// bma4_get_interrupt_mode(&status, &drv_data->dev);
	// LOG_WRN("Latch mode 0x%x", status);

	// uint8_t data[3] = { 0, 0, 0 };
	// bma4_read_regs(BMA4_INT_MAP_1_ADDR, data, 3, &drv_data->dev);
	// LOG_WRN("Map interrupt 0x%x 0x%x 0x%x", data[0], data[1], data[2]);

	// struct bma4_err_reg err_reg;
	// bma4_get_error_status(&err_reg, &drv_data->dev);
	// LOG_WRN("Error Reg: fatal err 0x%x cmd err 0x%x err code 0x%x fifo err 0x%x, aux err 0x%x",
	// 	err_reg.fatal_err, err_reg.cmd_err, err_reg.err_code, err_reg.fifo_err, err_reg.aux_err);

	// bma4_get_status(&status, &drv_data->dev);
	// LOG_WRN("Before Status 0x%x", status);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = bma4_read_accel_xyz(&drv_data->accel_data, &drv_data->dev);
		break;
	case BMA421_CHAN_STEP_COUNTER:
		ret = bma421_step_counter_output(&drv_data->step_counter, &drv_data->dev);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		ret = bma4_get_temperature(&drv_data->temperature, BMA4_DEG, &drv_data->dev);
		break;
	case SENSOR_CHAN_ALL:
		ret = bma4_read_accel_xyz(&drv_data->accel_data, &drv_data->dev);
		ret = bma421_step_counter_output(&drv_data->step_counter, &drv_data->dev);
		ret = bma4_get_temperature(&drv_data->temperature, BMA4_DEG, &drv_data->dev);
		break;
	default:
		return -ENOTSUP;
	}

	if (ret != 0) {
		LOG_DBG("Could not read data");
		return -EIO;
	}

	// bma4_read_regs(BMA4_INTERNAL_STAT, &status, 1, &drv_data->dev);
	// LOG_WRN("Internal Status 0x%x", status);

	// bma4_get_status(&status, &drv_data->dev);
	// LOG_WRN("After Status 0x%x", status);

	// uint16_t major;
	// uint16_t minor;
	// bma421_get_version_config(&major, &minor, &drv_data->dev);
	// LOG_WRN("config version %d.%d", major, minor);

	// struct bma421_stepcounter_settings settings;
	// bma421_stepcounter_get_parameter(&settings, &drv_data->dev);
	// LOG_WRN("config param1 0x%x - param2 0x%x - param3 0x%x - param4 0x%x - param5 0x%x", 
	// 		settings.param1, settings.param2, settings.param3, settings.param4, settings.param5);

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
		val->val1 = drv_data->accel_data.x;
		val->val2 = 0;
		/* TODO: adapt accel_convert function to make it work
		bma421_channel_accel_convert(val,
					drv_data->accel_data.x,
					drv_data->accel_cfg.range,
					drv_data->dev.resolution);
		*/
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
		val->val1 = drv_data->accel_data.y;
		val->val2 = 0;
		val++;
		val->val1 = drv_data->accel_data.z;
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
	//.attr_get = bma421_attr_get,  //from https://github.com/zephyrproject-rtos/zephyr/commit/696fc3afbf636a6c63a203cf614c584674e81820
	.trigger_set = bma421_trigger_set,
#endif
	.sample_fetch = bma421_sample_fetch,
	.channel_get = bma421_channel_get,
};

int bma421_init_driver(struct device *dev)
{
	struct bma421_data *drv_data = dev->driver_data;
	int8_t ret;

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

	ret = bma421_init(&drv_data->dev);
	if (ret) {
		LOG_ERR("init failed err %d", ret);
	}
	ret = bma421_write_config_file(&drv_data->dev);
	if (ret) {
		LOG_ERR("writing config failed err %d", ret);
	}

	ret = bma4_get_accel_config(&drv_data->accel_cfg, &drv_data->dev);
	if (ret) {
		LOG_ERR("Failed to get Acceleration config err %d", ret);
	}

#if defined(CONFIG_BMA421_ACC_PERF_MODE)
	drv_data->accel_cfg.perf_mode = BMA4_CONTINUOUS_MODE;
#else
	drv_data->accel_cfg.perf_mode = BMA4_CIC_AVG_MODE;
#endif

#if defined(CONFIG_BMA421_ACC_RANGE_2G)
	drv_data->accel_cfg.range = BMA4_ACCEL_RANGE_2G;
#elif defined(CONFIG_BMA421_ACC_RANGE_4G)
	drv_data->accel_cfg.range = BMA4_ACCEL_RANGE_4G;
#elif defined(CONFIG_BMA421_ACC_RANGE_8G)
	drv_data->accel_cfg.range = BMA4_ACCEL_RANGE_8G;
#elif defined(CONFIG_BMA421_ACC_RANGE_16G)
	drv_data->accel_cfg.range = BMA4_ACCEL_RANGE_16G;
#endif

	/*
	When perf mode disabled, ODR must be set to min 50Hz for most features
	and min 200Hz for double feature
	*/
#if defined(CONFIG_BMA421_ACC_ODR_0_78HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_0_78HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_1_56HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_1_56HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_3_12HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_3_12HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_6_25HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_6_25HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_12_5HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_12_5HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_25HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_25HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_50HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_100HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_200HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_200HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_400HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_400HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_800HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_800HZ;
#elif defined(CONFIG_BMA421_ACC_ODR_1600HZ)
	drv_data->accel_cfg.odr = BMA4_OUTPUT_DATA_RATE_1600HZ;
#else
	drv_data->accel_cfg.odr = 0;
#endif

#if defined(CONFIG_BMA421_ACC_BWP_OSR4_AVG1)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_OSR4_AVG1;
#elif defined(CONFIG_BMA421_ACC_BWP_OSR2_AVG2)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_OSR2_AVG2;
#elif defined(CONFIG_BMA421_ACC_BWP_NORM_AVG4)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
#elif defined(CONFIG_BMA421_ACC_BWP_CIC_AVG8)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_CIC_AVG8;
#elif defined(CONFIG_BMA421_ACC_BWP_RES_AVG16)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_RES_AVG16;
#elif defined(CONFIG_BMA421_ACC_BWP_RES_AVG32)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_RES_AVG32;
#elif defined(CONFIG_BMA421_ACC_BWP_RES_AVG64)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_RES_AVG64;
#elif defined(CONFIG_BMA421_ACC_BWP_RES_AVG128)
	drv_data->accel_cfg.bandwidth = BMA4_ACCEL_RES_AVG128;
#endif
		
	ret = bma4_set_accel_config(&drv_data->accel_cfg, &drv_data->dev);
	if (ret) {
		LOG_ERR("Failed to set Acceleration config err %d", ret);
	}

	// ret = bma421_feature_enable(BMA421_STEP_CNTR, 1, &drv_data->dev);
	// if (ret) {
	// 	LOG_ERR("cannot activate stepcounter err %d", ret);
	// }

	ret = bma4_set_advance_power_save(BMA4_ENABLE, &drv_data->dev);
	if (ret) {
		LOG_ERR("cannot activate power save state err %d", ret);
	}

	uint8_t status = 0xFF;
	ret = bma4_read_regs(BMA4_INTERNAL_STAT, &status, 1, &drv_data->dev);

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
