/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Stephane Dorre <stephane.dorre@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CST816S_CST816S_H_
#define ZEPHYR_DRIVERS_SENSOR_CST816S_CST816S_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/sensor/cst816s.h>

#define CST816S_I2C_ADDRESS	            0x15

#define CST816S_CHIP_ID                 0xB4

/***
 * TODO
 * - CST816S_REG_YPOS_H : Bits 4-7 => fingerID
 * - Reg addr 0x07 : Pressure ?
 * - Reg addr 0x08 : Bit 4-7: Area ?
 */

/* See Register documentation here:
 * https://github.com/endian-albin/pinetime-hypnos/wiki/
 * Technical-documentation#cst816s-touch-controller
 */

#define CST816S_REG_DATA                0x00
#define CST816S_REG_GESTURE_ID          0x01
#define CST816S_REG_FINGER_NUM          0x02
#define CST816S_REG_XPOS_H              0x03
#define CST816S_REG_XPOS_L              0x04
#define CST816S_REG_YPOS_H              0x05
#define CST816S_REG_YPOS_L              0x06
#define CST816S_REG_BPC0H               0xB0
#define CST816S_REG_BPC0L               0xB1
#define CST816S_REG_BPC1H               0xB2
#define CST816S_REG_BPC1L               0xB3
#define CST816S_REG_POWER_MODE          0xA5
#define CST816S_REG_CHIP_ID             0xA7
#define CST816S_REG_PROJ_ID             0xA8
#define CST816S_REG_FW_VERSION          0xA9
#define CST816S_REG_MOTION_MASK         0xEC
#define CST816S_REG_IRQ_PULSE_WIDTH     0xED
#define CST816S_REG_NOR_SCAN_PER        0xEE
#define CST816S_REG_MOTION_S1_ANGLE     0xEF
#define CST816S_REG_LP_SCAN_RAW1H       0xF0
#define CST816S_REG_LP_SCAN_RAW1L       0xF1
#define CST816S_REG_LP_SCAN_RAW2H       0xF2
#define CST816S_REG_LP_SCAN_RAW2L       0xF3
#define CST816S_REG_LP_AUTO_WAKEUP_TIME 0xF4
#define CST816S_REG_LP_SCAN_TH          0xF5
#define CST816S_REG_LP_SCAN_WIN         0xF6
#define CST816S_REG_LP_SCAN_FREQ        0xF7
#define CST816S_REG_LP_SCAN_I_DAC       0xF8
#define CST816S_REG_AUTOSLEEP_TIME      0xF9
#define CST816S_REG_IRQ_CTL             0xFA
#define CST816S_REG_DEBOUNCE_TIME       0xFB
#define CST816S_REG_LONG_PRESS_TIME     0xFC
#define CST816S_REG_IOCTL               0xFD
#define CST816S_REG_DIS_AUTO_SLEEP      0xFE

#define CST816S_GESTURE_NONE          (0x00)
#define CST816S_GESTURE_SLIDE_UP      (0x01)
#define CST816S_GESTURE_SLIDE_DOWN    (0x02)
#define CST816S_GESTURE_SLIDE_LEFT    (0x03)
#define CST816S_GESTURE_SLIDE_RIGHT   (0x04)
#define CST816S_GESTURE_CLICK         (0x05)
#define CST816S_GESTURE_DOUBLE_CLICK  (0x0B)
#define CST816S_GESTURE_LONG_PRESS    (0x0C)

#define CST816S_MOTION_EN_CON_LR      (1<<2)
#define CST816S_MOTION_EN_CON_UR      (1<<1)
#define CST816S_MOTION_EN_DCLICK      (1<<0)

#define CST816S_IRQ_EN_TEST           (1<<7)
#define CST816S_IRQ_EN_TOUCH          (1<<6)
#define CST816S_IRQ_EN_CHANGE         (1<<5)
#define CST816S_IRQ_EN_MOTION         (1<<4)
#define CST816S_IRQ_ONCE_WLP          (1<<0)

#define CST816S_IOCTL_SOFT_RTS        (1<<2)
#define CST816S_IOCTL_IIC_OD          (1<<1)
#define CST816S_IOCTL_EN_1V8          (1<<0)

#define CST816S_POWER_MODE_SLEEP          (0x03)
#define CST816S_POWER_MODE_EXPERIMENTAL   (0x05)

typedef struct {
	int16_t x;
	int16_t y;
} coordinate_t;

struct cst816s_data {
	struct device *i2c;

	uint8_t chip_id;

	enum cst816s_gesture gesture;
	uint8_t number_touch_point;
	enum cst816s_action action;

	coordinate_t touch_point_1;
	coordinate_t touch_point_2;

	struct device *reset_gpio;
#ifdef CONFIG_CST816S_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_CST816S_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_CST816S_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_CST816S_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_CST816S_TRIGGER */
};

#ifdef CONFIG_CST816S_TRIGGER
int cst816s_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int cst816s_attr_set(struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val);

int cst816s_init_interrupt(struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_CST816S_CST816S_H_ */
