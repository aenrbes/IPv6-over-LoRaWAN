/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
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
 *
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
 * \addtogroup loramac
 * @{
 *
 * \defgroup loramac-clock loramac Clock
 *
 * Implementation of the clock module for the loramac
 *
 * To implement the clock functionality, we use the SysTick peripheral on the
 * cortex-M3. We run the system clock at a configurable speed and set the 
 * SysTick to give us 128 interrupts / sec. However, the Sleep Timer counter 
 * value is used for the number of elapsed ticks in order to avoid a 
 * significant time drift caused by PM1/2. Contrary to the Sleep Timer, the 
 * SysTick peripheral is indeed frozen during PM1/2, so adjusting upon wake-up 
 * a tick counter based on this peripheral would hardly be accurate.
 * @{
 *
 * \file
 * Clock driver implementation for the TI loramac
 */
#include "sys/clock.h"
#include "lora-timer.h"

#include <stdint.h>
/*---------------------------------------------------------------------------*/
/**
 * \brief Arch-specific implementation of clock_init for the loramac
 *
 * We initialise the SysTick to fire 128 interrupts per second, giving us a
 * value of 128 for CLOCK_SECOND
 *
 * We also initialise GPT0:Timer A, which is used by clock_delay_usec().
 * We use 16-bit range (individual), count-down, one-shot, no interrupts.
 * The prescaler is computed according to the system clock in order to get 1
 * tick per usec.
 */
void
clock_init(void)
{
  return;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  TimerTime_t sys_mcu_time;
  sys_mcu_time = TimerGetCurrentTime();
  return sys_mcu_time;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  TimerTime_t sys_mcu_time;
  sys_mcu_time = TimerGetCurrentTime();
  return sys_mcu_time / 1000;
}
/*---------------------------------------------------------------------------*/
void
clock_delay(unsigned int i)
{
  /* Does not do anything. */
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 * @}
 */
