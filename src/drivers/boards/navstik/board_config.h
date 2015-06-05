/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * NAVSTIK NXT internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

#define UDID_START			0x1FFF7A10

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/*NAVSTIK NXT GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1			(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED2			(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)

/* Extra GPIO */
#define EXTRA_GPIO			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* External interrupts */
#define GPIO_EXTI_MAG_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN7)
#define GPIO_EXTI_MPU_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN10)
#define GPIO_EXTI_AIRSPEED_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN3)
#define GPIO_EXTI_EXT_MPU_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN4)
#define GPIO_EXTI_GPS_1_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN2)
#define GPIO_EXTI_GPS_2_DRDY		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)

/* Data ready pins off */
#define GPIO_EXTI_MAG_DRDY_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN7)
#define GPIO_EXTI_MPU_DRDY_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTA|GPIO_PIN10)
#define GPIO_EXTI_AIRSPEED_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN3)
#define GPIO_EXTI_EXT_MPU_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTD|GPIO_PIN4)
#define GPIO_EXTI_GPS_1_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN2)
#define GPIO_EXTI_GPS_2_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN10)

/* GPS */
#define GPS_1_DEFAULT_UART_PORT		"/dev/ttyS4"
#define GPS_2_DEFAULT_UART_PORT		"/dev/ttyS0"
#define GPS_DEFAULT_UART_PORT		GPS_1_DEFAULT_UART_PORT

/* SPI3 off */
#define GPIO_SPI3_SCK_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN10)
#define GPIO_SPI3_MISO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN11)
#define GPIO_SPI3_MOSI_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN12)

/* SPI4 off */
#define GPIO_SPI4_SCK_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI4_MISO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN5)
#define GPIO_SPI4_MOSI_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN6)

/* SPI chip selects off */
#define GPIO_SPI_CS_MPU_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI_CS_FRAM_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN10)
#define GPIO_SPI_CS_SDCARD_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI_CS_EXT_MPU_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI_CS_EXT_CS_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN5)

/* SPI chip selects */
#define GPIO_SPI_CS_MPU			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI_CS_FRAM		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define GPIO_SPI_CS_SDCARD		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI_CS_EXT_MPU		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI_CS_EXT_CS		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)


#define NAVSTIK_SPI_BUS_SENSORS		3
#define NAVSTIK_SPI_BUS_EXT_SENSORS	NAVSTIK_SPI_BUS_SENSORS
#define NAVSTIK_SPI_BUS_SDCARD		4
#define NAVSTIK_SPI_BUS_FRAM		4

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3 */
#define NAVSTIK_SPIDEV_MPU		1
#define NAVSTIK_SPIDEV_EXT_MPU		2

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define NAVSTIK_SPIDEV_FRAM		1
#define NAVSTIK_SPIDEV_SDCARD		2

/* I2C busses */
#define NAVSTIK_I2C_BUS_GPS_1		3
#define NAVSTIK_I2C_BUS_GPS_2		1
#define NAVSTIK_I2C_BUS_SENSORS		2
#define NAVSTIK_I2C_BUS_LED		NAVSTIK_I2C_BUS_SENSORS

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define NAVSTIK_I2C_OBDEV_LED		0x55	// RGBLED
#define NAVSTIK_I2C_OBDEV_HMC5883	0x1e	// Magnetometer
#define NAVSTIK_I2C_OBDEV_BMP280_1	0x76	// Static pressure sensor 1
#define NAVSTIK_I2C_OBDEV_BMP280_2	0x77	// Static pressure sensor 2

/* User GPIOs
 *
 * GPIO0-11 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN6)
#define GPIO_GPIO1_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN7)
#define GPIO_GPIO2_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO3_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO4_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)
#define GPIO_GPIO8_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO9_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO10_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN5)
#define GPIO_GPIO11_INPUT		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN8)
#define GPIO_GPIO0_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN6)
#define GPIO_GPIO1_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_GPIO2_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO3_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO4_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_GPIO8_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO9_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO10_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)
#define GPIO_GPIO11_OUTPUT		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)

/* Power supply control and monitoring GPIOs */
#define GPIO_GPS_PWR_EN			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_TELE_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

/* Tone alarm output */
#define TONE_ALARM_TIMER		10	/* timer 10 */
#define TONE_ALARM_CHANNEL		1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_TONE_ALARM			(GPIO_ALT|GPIO_AF3|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN8)

/* PWM
 *
 * TWELVE PWM outputs are configured.
 *
 * Pins:
 *
 * CH1  : PA6  : TIM3_CH1
 * CH2  : PA7  : TIM3_CH2
 * CH3  : PB0  : TIM3_CH3
 * CH4  : PB1  : TIM3_CH4
 * CH5  : PD12 : TIM4_CH1
 * CH6  : PD13 : TIM4_CH2
 * CH7  : PD14 : TIM4_CH3
 * CH8  : PD15 : TIM4_CH4
 * CH9  : PA0  : TIM5_CH1
 * CH10 : PA1  : TIM5_CH2
 * CH11 : PA5  : TIM8_CH1N
 * CH12 : PC8  : TIM8_CH3
 */
#define GPIO_TIM3_CH1OUT		GPIO_TIM3_CH1OUT_1
#define GPIO_TIM3_CH2OUT		GPIO_TIM3_CH2OUT_1
#define GPIO_TIM3_CH3OUT		GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT		GPIO_TIM3_CH4OUT_1
#define GPIO_TIM4_CH1OUT		GPIO_TIM4_CH1OUT_2
#define GPIO_TIM4_CH2OUT		GPIO_TIM4_CH2OUT_2
#define GPIO_TIM4_CH3OUT		GPIO_TIM4_CH3OUT_2
#define GPIO_TIM4_CH4OUT		GPIO_TIM4_CH4OUT_2
#define GPIO_TIM5_CH1OUT		GPIO_TIM5_CH1OUT_1
#define GPIO_TIM5_CH2OUT		GPIO_TIM5_CH2OUT_1
#define GPIO_TIM8_CH1OUT		GPIO_TIM8_CH1N_1
#define GPIO_TIM8_CH3OUT		GPIO_TIM8_CH3OUT_1

/* PWM
 *
 * EIGHT PWM inputs are configured.
 *
 * Pins:
 *
 * CH1  : PE9  : TIM1_CH1
 * CH2  : PE11 : TIM1_CH2
 * CH3  : PE13 : TIM1_CH3
 * CH4  : PE14 : TIM1_CH4
 * CH5  : PA15 : TIM2_CH1
 * CH6  : PB3  : TIM2_CH2
 * CH7  : PA2  : TIM2_CH3
 * CH8  : PA3  : TIM2_CH4
 */

#define GPIO_TIM1_CH1IN			GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN			GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN			GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN			GPIO_TIM1_CH4IN_2
#define GPIO_TIM2_CH1IN			GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN			GPIO_TIM2_CH2IN_2
#define GPIO_TIM2_CH3IN			GPIO_TIM2_CH3IN_1
#define GPIO_TIM2_CH4IN			GPIO_TIM2_CH4IN_1

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS			(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER			1	/* use timer9 for the HRT */
#define HRT_TIMER_CHANNEL		1	/* use capture/compare channel */
#define HRT_PPM_CHANNEL			3	/* use capture/compare channel 3 */
#define GPIO_PPM_IN				(GPIO_ALT|GPIO_AF1|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN13)

/* PWM input driver. Use ULTRASOUND pin attached to timer11 channel 1 */
#define PWMIN_TIMER			11
#define PWMIN_TIMER_CHANNEL		1
#define GPIO_PWM_IN			GPIO_TIM11_CH1IN_1

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NAVSTIK NXT board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */

__END_DECLS
