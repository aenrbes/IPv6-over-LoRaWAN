/*
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup sensortag-hdc-sensor
 * @{
 *
 * \file
 *        Driver for the Sensortag HDC1000 sensor.
 * \author
 *        Edvard Pettersen <e.pettersen@ti.com>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include "hdc-1000-sensor.h"
/*---------------------------------------------------------------------------*/
#include <Board.h>

#include <ti/drivers/I2C.h>
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/*
 * Disable the entire file if sensors are disabled, as it could potentially
 * create compile errors with missing defines from either the Board file or
 * configuration defines.
 */
#if BOARD_SENSORS_ENABLE
/*---------------------------------------------------------------------------*/
#ifndef Board_HDC1000_ADDR
#error "Board file doesn't define the I2C address Board_HDC1000_ADDR"
#endif
/* Sensor I2C address */
#define HDC1000_I2C_ADDRESS        Board_HDC1000_ADDR
/*---------------------------------------------------------------------------*/
/* Registers */
#define HDC1000_REG_TEMP           0x00 /* Temperature */
#define HDC1000_REG_HUM            0x01 /* Humidity */
#define HDC1000_REG_CONFIG         0x02 /* Configuration */
#define HDC1000_REG_SERID_H        0xFB /* Serial ID high */
#define HDC1000_REG_SERID_M        0xFC /* Serial ID middle */
#define HDC1000_REG_SERID_L        0xFD /* Serial ID low */
#define HDC1000_REG_MANF_ID        0xFE /* Manufacturer ID */
#define HDC1000_REG_DEV_ID         0xFF /* Device ID */
/*---------------------------------------------------------------------------*/
/* Fixed values */
#define HDC1000_VAL_MANF_ID        0x5449
#define HDC1000_VAL_DEV_ID         0x1000
#define HDC1000_VAL_CONFIG         0x1000 /* 14 bit, acquired in sequence */
/*---------------------------------------------------------------------------*/
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define SWAP16(v) ((LO_UINT16(v) << 8) | HI_UINT16(v))

#define LSB16(v)  (LO_UINT16(v)), (HI_UINT16(v))
/*---------------------------------------------------------------------------*/
static I2C_Handle i2c_handle;
/*---------------------------------------------------------------------------*/
/* Raw data as returned from the sensor (Big Endian) */
typedef struct {
  uint16_t temp;
  uint16_t hum;
} HDC_1000_SensorData;

static HDC_1000_SensorData sensor_data;
/*---------------------------------------------------------------------------*/
static volatile HDC_1000_SENSOR_STATUS sensor_status = HDC_1000_SENSOR_STATUS_DISABLED;
/*---------------------------------------------------------------------------*/
/*
 * Maximum measurement durations in clock ticks. We use 14bit resolution, thus:
 * - Tmp: 6.35ms
 * - RH:  6.5ms
 */
#define MEASUREMENT_DURATION 2

/*
 * Wait SENSOR_STARTUP_DELAY clock ticks between activation and triggering a
 * reading (max 15ms)
 */
#define SENSOR_STARTUP_DELAY 3

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/**
 * \brief         Setup and peform an I2C transaction.
 * \param wbuf    Output buffer during the I2C transation.
 * \param wcount  How many bytes in the wbuf.
 * \param rbuf    Input buffer during the I2C transation.
 * \param rcount  How many bytes to read into rbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
static bool
i2c_write_read(void *wbuf, size_t wcount, void *rbuf, size_t rcount)
{
  I2C_Transaction i2c_transaction = {
    .writeBuf = wbuf,
    .writeCount = wcount,
    .readBuf = rbuf,
    .readCount = rcount,
    .slaveAddress = HDC1000_I2C_ADDRESS,
  };

  return I2C_transfer(i2c_handle, &i2c_transaction);
}
/**
 * \brief         Peform a write only I2C transaction.
 * \param wbuf    Output buffer during the I2C transation.
 * \param wcount  How many bytes in the wbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
static inline bool
i2c_write(void *wbuf, size_t wcount)
{
  return i2c_write_read(wbuf, wcount, NULL, 0);
}
/**
 * \brief         Peform a read only I2C transaction.
 * \param rbuf    Input buffer during the I2C transation.
 * \param rcount  How many bytes to read into rbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
static inline bool
i2c_read(void *rbuf, size_t rcount)
{
  return i2c_write_read(NULL, 0, rbuf, rcount);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief   Initialize the HDC-1000 sensor driver.
 * \return  true if I2C operation successful; else, return false.
 */
static bool
sensor_init(void)
{
  if(i2c_handle) {
    return true;
  }

  I2C_Params i2c_params;
  I2C_Params_init(&i2c_params);

  i2c_params.transferMode = I2C_MODE_BLOCKING;
  i2c_params.bitRate = I2C_400kHz;

  i2c_handle = I2C_open(Board_I2C0, &i2c_params);
  if(i2c_handle == NULL) {
    return false;
  }

  /* Enable reading data in one operation */
  uint8_t config_data[] = { HDC1000_REG_CONFIG, LSB16(HDC1000_VAL_CONFIG) };

  return i2c_write(config_data, sizeof(config_data));
}
/*---------------------------------------------------------------------------*/
/**
 * \brief   Start measurement.
 * \return  true if I2C operation successful; else, return false.
 */
static bool
start(void)
{
  uint8_t temp_reg[] = { HDC1000_REG_TEMP };

  return i2c_write(temp_reg, sizeof(temp_reg));
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Convert raw data to temperature and humidity.
 * \param temp  Output variable to store converted temperature.
 * \param hum   Output variable to store converted humidity.
 */
static void
convert(int32_t *temp, int32_t *hum)
{
  int32_t raw_temp = SWAP16(sensor_data.temp);
  int32_t raw_hum = SWAP16(sensor_data.hum);

  /* Convert temperature to degrees C */
  *temp = raw_temp * 100 * 165 / 65536 - 40000;
  /* Convert relative humidity to a %RH value */
  *hum = raw_hum * 100 * 100 / 65536;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Callback when sensor is ready to read data from.
 */
static void
notify_ready(void *unused)
{
  /* Unused args */
  (void)unused;

  /* Latch readings */
  if(i2c_read(&sensor_data, sizeof(sensor_data))) {
    sensor_status = HDC_1000_SENSOR_STATUS_READINGS_READY;
  } else {
    sensor_status = HDC_1000_SENSOR_STATUS_I2C_ERROR;
  }

  sensors_changed(&hdc_1000_sensor);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns a reading from the sensor.
 * \param type  HDC_1000_SENSOR_TYPE_TEMP or HDC_1000_SENSOR_TYPE_HUMID.
 * \return      Temperature (centi degrees C) or Humidity (centi %RH).
 */
static int
value(int type)
{
  int32_t temp = 0;
  int32_t hum = 0;

  if(sensor_status != HDC_1000_SENSOR_STATUS_READINGS_READY) {
    PRINTF("Sensor disabled or starting up (%d)\n", sensor_status);
    return HDC_1000_READING_ERROR;
  }

  switch(type) {
  case HDC_1000_SENSOR_TYPE_TEMP:
  case HDC_1000_SENSOR_TYPE_HUMID:
    convert(&temp, &hum);
    PRINTF("HDC: t=%d h=%d\n", (int)temp, (int)hum);

    if(type == HDC_1000_SENSOR_TYPE_TEMP) {
      return (int)temp;
    } else if(type == HDC_1000_SENSOR_TYPE_HUMID) {
      return (int)hum;
    } else {
      return HDC_1000_READING_ERROR;
    }

  default:
    PRINTF("Invalid type\n");
    return HDC_1000_READING_ERROR;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Configuration function for the HDC1000 sensor.
 * \param type    Activate, enable or disable the sensor. See below.
 * \param enable  Either enable or disable the sensor.
 *                When type == SENSORS_HW_INIT we turn on the hardware.
 *                When type == SENSORS_ACTIVE and enable==1 we enable the sensor.
 *                When type == SENSORS_ACTIVE and enable==0 we disable the sensor.
 */
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    memset(&sensor_data, 0, sizeof(sensor_data));

    if(sensor_init()) {
      sensor_status = HDC_1000_SENSOR_STATUS_INITIALISED;
    } else {
      sensor_status = HDC_1000_SENSOR_STATUS_I2C_ERROR;
    }
    break;

  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(sensor_status == HDC_1000_SENSOR_STATUS_DISABLED) {
      break;
    }

    if(enable) {
      if(!start()) {
        sensor_status = HDC_1000_SENSOR_STATUS_I2C_ERROR;
        break;
      }
      ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
      sensor_status = HDC_1000_SENSOR_STATUS_TAKING_READINGS;
    } else {
      ctimer_stop(&startup_timer);
      sensor_status = HDC_1000_SENSOR_STATUS_INITIALISED;
    }
    break;

  default:
    break;
  }
  return sensor_status;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns the status of the sensor.
 * \param type  SENSORS_ACTIVE or SENSORS_READY.
 * \return      One of the SENSOR_STATUS_xyz defines.
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return sensor_status;

  default:
    return HDC_1000_SENSOR_STATUS_DISABLED;
  }
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(hdc_1000_sensor, "HDC1000", value, configure, status);
/*---------------------------------------------------------------------------*/
#endif /* BOARD_SENSORS_ENABLE */
/*---------------------------------------------------------------------------*/
/** @} */
