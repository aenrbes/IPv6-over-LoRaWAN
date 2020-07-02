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
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-net.h"

#include "net/routing/routing.h"
#include "lorawan-border-router.h"
#include "border-router.h"

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "BR"
#define LOG_LEVEL LOG_LEVEL_INFO

#include <stdlib.h>

extern int contiki_argc;
extern char **contiki_argv;
extern const char *slip_config_ipaddr;

uip_ipaddr_t host_ipaddr;

static uint8_t mac_addr[] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };

PROCESS(border_router_process, "Border router process");
/*---------------------------------------------------------------------------*/

static void
set_global_address(uip_ipaddr_t *prefix, uip_ipaddr_t *iid)
{
  static uip_ipaddr_t root_ipaddr;
  const uip_ipaddr_t *default_prefix;
  int i;

  default_prefix = uip_ds6_default_prefix();

  /* Assign a unique local address (RFC4193,
     http://tools.ietf.org/html/rfc4193). */
  if(prefix == NULL) {
    uip_ip6addr_copy(&root_ipaddr, default_prefix);
  } else {
    memcpy(&root_ipaddr, prefix, 8);
  }
  if(iid == NULL) {
    uip_ds6_set_addr_iid(&root_ipaddr, &uip_lladdr);
  } else {
    memcpy(((uint8_t*)&root_ipaddr) + 8, ((uint8_t*)iid) + 8, 8);
  }

  uip_ds6_addr_add(&root_ipaddr, 0, ADDR_AUTOCONF);

  if(LOG_DBG_ENABLED) {
    uint8_t state;

    LOG_DBG("IPv6 addresses: ");
    for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
      state = uip_ds6_if.addr_list[i].state;
      if(uip_ds6_if.addr_list[i].isused &&
         (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
        LOG_DBG_6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
        LOG_DBG_("\n");
      }
    }
  }
}

static void
border_router_set_mac(const uint8_t *data)
{
  memcpy(uip_lladdr.addr, data, sizeof(uip_lladdr.addr));
  linkaddr_set_node_addr((linkaddr_t *)uip_lladdr.addr);

  /* is this ok - should instead remove all addresses and
     add them back again - a bit messy... ?*/
  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  uip_ds6_init();
  NETSTACK_ROUTING.init();
  PROCESS_CONTEXT_END(&tcpip_process);
}

PROCESS_THREAD(border_router_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  LOG_INFO("LORAWAN-Border router started\n");

  slip_config_handle_arguments(contiki_argc, contiki_argv);

  /* tun init is also responsible for setting up the SLIP connection */
  tun_init();

  border_router_set_mac(mac_addr);

  if(slip_config_ipaddr != NULL) {
    uip_ipaddr_t prefix;

    if(uiplib_ipaddrconv((const char *)slip_config_ipaddr, &prefix)) {
      LOG_INFO("Setting prefix ");
      LOG_INFO_6ADDR(&prefix);
      LOG_INFO_("\n");
      host_ipaddr = prefix;
      set_global_address(&prefix, NULL);
    } else {
      LOG_ERR("Parse error: %s\n", slip_config_ipaddr);
      exit(0);
    }
  }

  print_local_addresses();

  slip_init();

  while(1) {
    etimer_set(&et, CLOCK_SECOND * 2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    /* do anything here??? */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
