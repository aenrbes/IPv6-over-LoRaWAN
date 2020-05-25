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
 * \addtogroup cc2538-platforms
 * @{
 *
 * \defgroup cc2538dk The cc2538 Development Kit platform
 *
 * The cc2538DK is a platform by Texas Instruments, based on the
 * cc2538 SoC with an ARM Cortex-M3 core.
 * @{
 *
 * \file
 *   Main module for the cc2538dk platform
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"

#include "dev/serial-line.h"

#include "net/netstack.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-debug.h"
#include "net/queuebuf.h"
#include "net/ipv6/uip-ds6.h"
#include "net/linkaddr.h"

#include "sys/int-master.h"

#include "ieee-addr.h"
#include "uart.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "LORAMAC_platform"
#define LOG_LEVEL LOG_LEVEL_MAIN

extern void LowLevelInit(void);
extern void LoRaMacProcessLoopOnes(void);

extern bool LoRaMacPacketEmpty;
extern bool LoRaMacPacketAckReceived;
extern bool LoRaMacRecvPending;

extern Uart_t Uart1;

PROCESS_NAME(loramac_recv_process);
/*---------------------------------------------------------------------------*/
static void
set_lladdr(void)
{
  linkaddr_t addr;

  memset(&addr, 0, sizeof(linkaddr_t));
  ieee_addr_cpy_to(addr.u8, LINKADDR_SIZE);

  linkaddr_set_node_addr(&addr);
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  const uip_ipaddr_t *default_prefix = uip_ds6_default_prefix();

  /* Assign a unique local address (RFC4193,
     http://tools.ietf.org/html/rfc4193). */
  uip_ip6addr_copy(&ipaddr, default_prefix);

  /* Assumes that the uip_lladdr is set */
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  LOG_INFO("Added global IPv6 address ");
  LOG_INFO_6ADDR(&ipaddr);
  LOG_INFO_("\n");

  /* set the PREFIX::1 address to the IF */
  uip_ip6addr_copy(&ipaddr, default_prefix);
  ipaddr.u8[15] = 2;
  uip_ds6_defrt_add(&ipaddr, 0);
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_one(void)
{
  LowLevelInit();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_two()
{
  set_lladdr();
  serial_line_init();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three()
{
  set_global_address();
}
/*---------------------------------------------------------------------------*/
void
platform_main_loop()
{
  printf("---main_loop---\n");
  while(1) {
    uint8_t c;

    process_run();
    LoRaMacProcessLoopOnes();

    if (LoRaMacRecvPending == true) {
      process_poll(&loramac_recv_process);
    }

    if (UartGetChar(&Uart1, &c) == 0) {
      UartPutChar(&Uart1, c);
      serial_line_input_byte(c);
    }

    etimer_request_poll();
  }

  return;
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
  printf("%s%s\n", m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  printf("%s\n", m);
}
