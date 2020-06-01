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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc2538dk
 * @{
 *
 * \file
 *  Configuration for the cc2538dk platform
 */
#ifndef CONTIKI_CONF_H_
#define CONTIKI_CONF_H_

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
/*---------------------------------------------------------------------------*/
/* Include Project Specific conf */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
#include "loramac-def.h"
/*---------------------------------------------------------------------------*/
#define NETSTACK_CONF_MAC       loramac_driver
#define NETSTACK_CONF_RADIO     nullradio_driver
#define NETSTACK_CONF_FRAMER    no_framer

#define NETSTACK_CONF_NETWORK   schc_driver

#define UIP_CONF_ROUTER 0
#define UIP_CONF_ND6_SEND_RA 0
#define UIP_CONF_ND6_SEND_NS 0
#define UIP_CONF_ND6_SEND_NA 1
#define UIP_CONF_ND6_AUTOFILL_NBR_CACHE 1

#define PLATFORM_CONF_PROVIDES_MAIN_LOOP 1
#define PLATFORM_CONF_SUPPORTS_STACK_CHECK 0

/* configure network size and density */
#ifndef NETSTACK_MAX_ROUTE_ENTRIES
#define NETSTACK_MAX_ROUTE_ENTRIES   1
#endif /* NETSTACK_MAX_ROUTE_ENTRIES */
#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS 3
#endif /* NBR_TABLE_CONF_MAX_NEIGHBORS */


#define UIP_CONF_WAIT_TIMEOUT 300


#ifndef LOG_CONF_LEVEL_RPL
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_DBG /* Only for rpl-lite */
#endif /* LOG_CONF_LEVEL_RPL */

#ifndef LOG_CONF_LEVEL_TCPIP
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_TCPIP */

#ifndef LOG_CONF_LEVEL_IPV6
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_IPV6 */

#ifndef LOG_CONF_LEVEL_6LOWPAN
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_6LOWPAN */

#ifndef LOG_CONF_LEVEL_NULLNET
#define LOG_CONF_LEVEL_NULLNET                     LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_NULLNET */

#ifndef LOG_CONF_LEVEL_MAC
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_MAC */

#ifndef LOG_CONF_LEVEL_FRAMER
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_FRAMER */

#ifndef LOG_CONF_LEVEL_6TOP
#define LOG_CONF_LEVEL_6TOP                        LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_6TOP */

#ifndef LOG_CONF_LEVEL_COAP
#define LOG_CONF_LEVEL_COAP                        LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_COAP */

#ifndef LOG_CONF_LEVEL_SNMP
#define LOG_CONF_LEVEL_SNMP                        LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_COAP */

#ifndef LOG_CONF_LEVEL_LWM2M
#define LOG_CONF_LEVEL_LWM2M                       LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_LWM2M */

#ifndef LOG_CONF_LEVEL_MAIN
#define LOG_CONF_LEVEL_MAIN                        LOG_LEVEL_DBG
#endif /* LOG_CONF_LEVEL_MAIN */
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H_ */
/*---------------------------------------------------------------------------*/
/** @} */
