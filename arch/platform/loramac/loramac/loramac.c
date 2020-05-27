/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
*         The 802.15.4 standard LORAMAC protocol (nonbeacon-enabled)
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 */

#include "loramac/loramac.h"
#include "loramac/loramac-output.h"
#include "net/packetbuf.h"
#include "net/netstack.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "LORAMAC"
#define LOG_LEVEL LOG_LEVEL_MAC

static uint8_t default_lorawan_server_lladdr[] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };

PROCESS(loramac_recv_process, "loramac recv process");

static void input_packet(void);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(loramac_recv_process, ev, data)
{
  PROCESS_BEGIN();
  LOG_INFO("loramac_recv_process init...\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    packetbuf_clear();
    memcpy(packetbuf_dataptr(), LoRaMacRecvBuffer, LoRaMacRecvLength);
    LoRaMacRecvPending = false;

    if(LoRaMacRecvLength > 0) {
      packetbuf_set_datalen(LoRaMacRecvLength);
      input_packet();
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  if(packetbuf_totlen() > PACKETBUF_SIZE) {
    LOG_WARN("send failed, too large header\n");
    mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);
  } else {
    loramac_output_packet(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{
  linkaddr_t addr;

  memcpy(addr.u8, default_lorawan_server_lladdr, sizeof(addr.u8));
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &addr);
  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &linkaddr_node_addr);

  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, -40);

  LOG_INFO("received packet from ");
  LOG_INFO_LLADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER));
  LOG_INFO_("len %u\n", packetbuf_datalen());
  NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  process_start(&loramac_recv_process, NULL);
  loramac_output_init();
  on();
}
/*---------------------------------------------------------------------------*/
static int
max_payload(void)
{
  return LORAWAN_APP_DATA_MAX_SIZE;
}
/*---------------------------------------------------------------------------*/
const struct mac_driver loramac_driver = {
  "LORAMAC",
  init,
  send_packet,
  input_packet,
  on,
  off,
  max_payload,
};
/*---------------------------------------------------------------------------*/
