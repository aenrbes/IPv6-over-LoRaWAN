/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32l1xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l1xx.h"
//#include "stm32l1xx_hal.h"
#include "utilities.h"
#include "lora-timer.h"
#include "delay.h"
#include "gpio.h"
#include "lora-spi.h"
#include "uart.h"
#include "lora-radio.h"
#include "sx1276.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "uart-board.h"
#include "usb_device.h"
#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

//#define USE_BAND_868
//#define USE_DEBUGGER
//#define USB_VCP    						//Need define USE_DEBUGGER

/*!
 * Define indicating if an external IO expander is to be used
 */
//#define BOARD_IOE_EXT

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

/*!
 * Board IO Extender pins definitions
 */

/*!
 * Board MCU pins definitions
 */

#define RADIO_DIO_0                                 PB_11
#define RADIO_DIO_1                                 PB_10
//#define RADIO_DIO_2                                 PB_11
//#define RADIO_DIO_3                                 PB_3
//#define RADIO_DIO_4                                 PB_4
//#define RADIO_DIO_5                                 PB_5

//#define RF_RXTX1                                    PB_7
//#define RF_RXTX2                                    PB_8

#define RADIO_RESET                                 PA_3

#define RADIO_NSS                                   PA_4
#define RADIO_SCLK                                  PA_5
#define RADIO_MISO                                  PA_6
#define RADIO_MOSI                                  PA_7

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define USB_VCP_DM									PA_11
#define USB_VCP_DP									PA_12

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define Vext										PB_3
/*!
 * Board MCU unusedpins definitions
 */
#define UNUSEDPINPA0                                    PA_0
#define UNUSEDPINPA1                                    PA_1
#define UNUSEDPINPA2                                    PA_2
#define UNUSEDPINPA3                                    PA_3
#define UNUSEDPINPA4                                    PA_4
#define UNUSEDPINPA5                                    PA_5
#define UNUSEDPINPA6                                    PA_6
#define UNUSEDPINPA7                                    PA_7
#define UNUSEDPINPA8                                    PA_8
#define UNUSEDPINPA9                                    PA_9
#define UNUSEDPINPA10                                   PA_10
#define UNUSEDPINPA11                                   PA_11
#define UNUSEDPINPA12                                   PA_12
#define UNUSEDPINPA13                                   PA_13
#define UNUSEDPINPA14                                   PA_14
#define UNUSEDPINPA15                                   PA_15

#define UNUSEDPINPB0                                    PB_0
#define UNUSEDPINPB1                                    PB_1
#define UNUSEDPINPB2                                    PB_2
#define UNUSEDPINPB3                                    PB_3
#define UNUSEDPINPB4                                    PB_4
#define UNUSEDPINPB5                                    PB_5
#define UNUSEDPINPB6                                    PB_6
#define UNUSEDPINPB7                                    PB_7
#define UNUSEDPINPB8                                    PB_8
#define UNUSEDPINPB9                                    PB_9
#define UNUSEDPINPB10                                   PB_10
#define UNUSEDPINPB11                                   PB_11
#define UNUSEDPINPB12                                   PB_12
#define UNUSEDPINPB13                                   PB_13
#define UNUSEDPINPB14                                   PB_14
#define UNUSEDPINPB15                                   PB_15

#define UNUSEDPINPC13                                   PC_13

//extern Gpio_t rftrx1;
//extern Gpio_t rftrx2;

/*!
 * MCU objects
 */
extern Uart_t Uart1;
#if defined( USE_USB_CDC )
extern Uart_t UartUsb;
#endif



/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

uint32_t HexToString(/*IN*/  const char    * pHex,  
                     /*IN*/  uint32_t           hexLen,  
                     /*OUT*/ char          * pByteString);

#endif // __BOARD_H__
