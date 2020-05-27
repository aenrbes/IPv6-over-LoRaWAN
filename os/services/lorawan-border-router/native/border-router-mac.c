/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         A null RDC implementation that uses framer for headers and sends
 *         the packets over slip instead of radio.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "border-router.h"
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "BR-MAC"
#define LOG_LEVEL LOG_LEVEL_INFO

#define MAX_CALLBACKS 16

/* a structure for calling back when packet data is coming back
   from radio... */
struct tx_callback {
  mac_callback_t cback;
  void *ptr;
  struct packetbuf_attr attrs[PACKETBUF_NUM_ATTRS];
  struct packetbuf_addr addrs[PACKETBUF_NUM_ADDRS];
};

static struct tx_callback send_callback;
/*---------------------------------------------------------------------------*/

void
packet_sent(uint8_t status, uint8_t tx)
{
    struct tx_callback *callback;
    callback = &send_callback;
    packetbuf_clear();
    packetbuf_attr_copyfrom(callback->attrs, callback->addrs);
    mac_call_sent_callback(callback->cback, callback->ptr, status, tx);
}
/*---------------------------------------------------------------------------*/
static void
setup_callback(mac_callback_t sent, void *ptr)
{
  struct tx_callback *callback;
  callback = &send_callback;
  callback->cback = sent;
  callback->ptr = ptr;
  packetbuf_attr_copyto(callback->attrs, callback->addrs);

  return;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  uint8_t buf[PACKETBUF_SIZE];
  char devEUI[17] = "\0";

  const linkaddr_t *lladdr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

  sprintf(devEUI, "%02x%02x%02x%02x%02x%02x%02x%02x", 
                                            lladdr->u8[0],
                                            lladdr->u8[1],
                                            lladdr->u8[2],
                                            lladdr->u8[3],
                                            lladdr->u8[4],
                                            lladdr->u8[5],
                                            lladdr->u8[6],
                                            lladdr->u8[7]);

  LOG_INFO("devEUI: %s\n", devEUI);

  LOG_INFO("sending packet (%u bytes)\n", packetbuf_datalen());

  if(packetbuf_totlen() > sizeof(buf)) {
    LOG_WARN("send failed, too large header\n");
    mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);
  } else {
    setup_callback(sent, ptr);

    /* Copy packet data */
    memcpy(&buf[0], packetbuf_dataptr(), packetbuf_datalen());

    write_to_slip(devEUI, buf, packetbuf_datalen());
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
off()
{
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
max_payload()
{
  return 51;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{

}
/*---------------------------------------------------------------------------*/
const struct mac_driver border_router_mac_driver = {
  "br-mac",
  init,
  send_packet,
  packet_input,
  on,
  off,
  max_payload,
};
/*---------------------------------------------------------------------------*/
