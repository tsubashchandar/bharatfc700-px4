/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#  define BOARD_HAS_USB_VALID           1
#  define BOARD_HAS_NBAT_V              1
#  define BOARD_HAS_NBAT_I              1


/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
#define GPIO_LED1                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN14)
//#define GPIO_LED2                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
//#define GPIO_LED3                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN13)

#define GPIO_LED_RED                 GPIO_LED1
//#define GPIO_LED_GREEN               GPIO_LED2
//#define GPIO_LED_BLUE                GPIO_LED3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
//#define BOARD_ARMED_LED        LED_BLUE
//#define BOARD_ARMED_STATE_LED  LED_GREEN

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PC4 */  GPIO_ADC12_INP4,  \
	/* PC0 */  GPIO_ADC123_INP10

#define ADC1_CH(n)                  (n)

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY_VOLTAGE_CHANNEL       ADC1_CH(4) /* PC4 BATT_VOLT_SENS */
#define ADC_BATTERY_CURRENT_CHANNEL       ADC1_CH(10) /* PC0 BATT_CURRENT_SENS */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))     

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* Power supply control and monitoring GPIOs */
#define GPIO_POWER_IN_A                 /* PD7 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN7)

/* HEATER */
#define GPIO_HEATER_OUTPUT   /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN11)
#define HEATER_OUTPUT_EN(on_true)      px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))


#define GPIO_VDD_BRICK1_VALID           GPIO_POWER_IN_A /* Brick 1 Is Chosen */
#define BOARD_NUMBER_BRICKS             1


/* CAN Silence: Silent mode control */

#define GPIO_VDD_3V3_SENSORS_EN         /* PC2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2) // VDD_3V3_SENSORS_EN

#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))

#define GPIO_VDD_3V3_SD_CARD_EN         /* PC1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)

#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  10


/* USB OTG FS */
#define GPIO_OTGFS_VBUS         /* PD15 */ (GPIO_INPUT|GPIO_OPENDRAIN|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN7)

/* High-resolution timer */
#define HRT_TIMER               2  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 2 */


#if 0
#define HRT_PPM_CHANNEL         /* T3C3 */  3  /* use capture/compare channel 3 */
#define GPIO_PPM_IN             /* PB0 T3C3 */ GPIO_TIM3_CH3IN_1

#endif

/* RC Serial port */
//#define RC_SERIAL_PORT          "/dev/ttyS3"
//#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT


/*
 * Board has a separate RC_IN
 *
 * GPIO PPM_IN on PB0 T3CH3
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART and can drive GPIO_PPM_IN as an output
 */
/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#define BOARD_ADC_BRICK_VALID   (px4_arch_gpioread(GPIO_VDD_BRICK1_VALID))

#define BOARD_NUM_IO_TIMERS 4
#define BOARD_DMA_ALLOC_POOL_SIZE 5120 /* This board provides a DMA pool and APIs */
#define BOARD_HAS_ON_RESET 1 /* This board provides the board_on_reset interface */
#define BOARD_ENABLE_CONSOLE_BUFFER

#define FLASH_BASED_PARAMS

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
                GPIO_POWER_IN_A,		  \
		GPIO_VDD_3V3_SENSORS_EN,	  \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_HEATER_OUTPUT,	         \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D1), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D2), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D3), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_OTGFS_VBUS,                  \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
