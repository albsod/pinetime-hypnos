/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CST816S_CST816S_H_
#define ZEPHYR_DRIVERS_SENSOR_CST816S_CST816S_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>

#define CST816S_I2C_ADDRESS		CONFIG_CST816S_I2C_ADDR

#define CST816S_REG_DATA 0x00
#define CST816S_REG_GESTURE_ID          0x01	// See CST816S_GESTURE_XXX definitions. 8-bit value.
#define CST816S_REG_FINGER_NUM          0x02	// Number of fingers. 8-bit value.
#define CST816S_REG_XPOS_H              0x03	// X coordinate 4 digits high X (Bit 3-0 : Xpos[11:8])
#define CST816S_REG_XPOS_L              0x04	// X coordinate 8 bits lower (Xpos[7:0])
#define CST816S_REG_YPOS_H              0x05	// Y coordinate 4 digits high Y (Bit 3-0 : Ypos[11:8])
#define CST816S_REG_YPOS_L              0x06	// Y coordinate 8 bits lower (Ypos[7:0])
#define CST816S_REG_BPC1L               0xB0	// High 8 bits of BPC0 value
#define CST816S_REG_BPC0L               0xB1	// Lower 8 bits of BPC0 value
#define CST816S_REG_BPC1H               0xB2	// High 8 bits of BPC1 value
#define CST816S_REG_BPC1L               0xB3	// Lower 8 bits of BPC1 value.
#define CST816S_REG_CHIP_ID             0xA7	// Chip model
#define CST816S_REG_PROJ_ID             0xA8	// Project number
#define CST816S_REG_FW_VERSION          0xA9	// Firmware Version. 8-bit value.
#define CST816S_REG_MOTION_MASK         0xEC	// See CST816S_MOTION_EN_XXX definitions.
#define CST816S_REG_IRQ_PULSE_WIDTH     0xED	// Interrupt low pulse output width. Unit: 0.1ms, Optional value: 1~200. Default value: 10. 8-bit value.
#define CST816S_REG_NOR_SCAN_PER        0xEE	// Normal fast detection cycle. This value affects LpAutoWakeTime and AutoSleepTime. in units of 10ms. Selectable value: 1~30. default value is 1. 8-bit value.
#define CST816S_REG_MOTION_S1_ANGLE     0xEF	// Gesture detection sliding partition angle control. angle=tan(c)*10 c is the angle based on the positive direction of the x-axis. 8-bit value
#define CST816S_REG_LP_SCAN_RAW1H       0xF0	// Low power consumption scans the high 8 bits of the reference value of channel 1.
#define CST816S_REG_LP_SCAN_RAW1L       0xF1	// Low power consumption scans the lower 8 bits of the reference value of channel 1.
#define CST816S_REG_LP_SCAN_RAW2H       0xF2	// Low power consumption scans the high 8 bits of the reference value of channel 1. (Seems like  bad copy paste in the chinese datasheet, chanel 2 ? I don't understand what is this...)
#define CST816S_REG_LP_SCAN_RAW2L       0xF3	// Low power consumption scan 8 bits lower than the reference value of channel 1. (Seems like  bad copy paste in the chinese datasheet, chanel 2 ? I don't understand what is this...)
#define CST816S_REG_LP_AUTO_WAKEUP_TIME 0xF4	// Auto recalibration period when the power consumption is low. Unit: 1 minute, selectable value: 1~5. default value is 5. 8-bit value.
#define CST816S_REG_LP_SCAN_TH          0xF5	// Low-power scan wake-up threshold. The smaller the smaller the more sensitive. Selectable value:1~255. default value is 48.
#define CST816S_REG_LP_SCAN_WIN         0xF6	// Low-power scanning range. The larger the range, the more sensitive it is and the higher the power consumption. Selectable values: 0, 1, 2, 3. Default value is 3. 8-bit value.
#define CST816S_REG_LP_SCAN_FREQ        0xF7	// Low power consumption scanning frequency. Smaller is more sensitive. Selectable value:1~255. default value is 7.
#define CST816S_REG_LP_SCAN_I_DAC       0xF8 	// Low-power scanning current. The smaller the current, the more sensitive it is. Selectable value:1~255. 8-bit value.
#define CST816S_REG_AUTOSLEEP_TIME      0xF9	// When there is no touch within x seconds, it automatically enters the low-power mode. The unit is in sec and the default value is 2sec. 8-bit value.
#define CST816S_REG_IRQ_CTL             0xFA	// See CST816S_IRQ_XXX definitions 
#define CST816S_REG_DEBOUNCE_TIME       0xFB 	// When there is a touch but no valid gesture within x seconds, the function is automatically reset. The unit is sec. This function is not enabled when it is 0. The default is 5s. 8-bit value.
#define CST816S_REG_LONG_PRESS_TIME     0xFC	// Press and hold x seconds to reset automatically. The unit in sec. This function is not enabled when it is 0. The default is 10s. 8-bit value.
#define CST816S_REG_IOCTL               0xFD	// See CST816S_IOCTL_XXX definitions
#define CST816S_REG_DIS_AUTO_SLEEP      0xFE	// Default 0 = enable auto sleep, other than 0 mean disable the feature. 8-bit value.

#define CST816S_GESTURE_NONE         (0x00)	// No hand gesture detected.
#define CST816S_GESTURE_SLIDE_UP     (0x01)	// Slide up gesture detected.
#define CST816S_GESTURE_SLIDE_DOWN   (0x02)	// Slide down gesture detected.
#define CST816S_GESTURE_SLIDE_LEFT   (0x03)	// Slide left gesture detected.
#define CST816S_GESTURE_SLIDE_RIGHT  (0x04)	// Slide right gesture detected.
#define CST816S_GESTURE_CLICK        (0x05)	// click(touch& release) gesture detected.
#define CST816S_GESTURE_DOUBLE_CLICK (0x0B)	// double click gesture detected.
#define CST816S_GESTURE_LONG_PRESS   (0x0C)	// long press gesture detected.

#define CST816S_MOTION_EN_CON_LR   (1<<2)	// Continuous left/right sliding action is possible
#define CST816S_MOTION_EN_CON_UR   (1<<1)	// Enables a continuous up and down sliding motion
#define CST816S_MOTION_EN_DCLICK   (1<<0)	// Enabling double-click action

#define CST816S_IRQ_EN_TEST        (1<<7)	// Interrupt pin test, enable and then automatically periodically emit low pulses.
#define CST816S_IRQ_EN_TOUCH       (1<<6)	// When a touch is detected, a low pulse is emitted periodically.
#define CST816S_IRQ_EN_CHANGE      (1<<5)	// A low pulse is emitted when a change in touch condition is detected.
#define CST816S_IRQ_EN_MOTION      (1<<4)	// When a gesture is detected, a low pulse is emitted.
#define CST816S_IRQ_ONCE_WLP       (1<<0)	// Press and hold gesture and only one low pulse signal is emitted.

#define CST816S_IOCTL_SOFT_RTS     (1<<2)	// The main control can achieve the soft reset function of touch by pulling down the IRQ pin. 0: Disables soft reset. 1: Enables soft reset.
#define CST816S_IOCTL_IIC_OD       (1<<1)	// IIC pin drive mode, default is resistor pull-up. 0: resistive pull-up  1:OD
#define CST816S_IOCTL_EN_1V8	   (1<<0)	// IIC and IRQ pin level selection, default is VDD level. 0:VDD  1:1.8V

struct cst816s_data {
	struct device *i2c;
	int16_t x_sample;
	int16_t y_sample;

#ifdef CONFIG_CST816S_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

	/*K_THREAD_STACK_MEMBER(thread_stack, CONFIG_CST816S_THREAD_STACK_SIZE);*/

#if defined(CONFIG_CST816S_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, 10);
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
