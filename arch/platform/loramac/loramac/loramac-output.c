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
 *         The 802.15.4 standard CSMA protocol (nonbeacon-enabled).
 *         Output functions.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 */

#include "loramac/loramac.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "dev/watchdog.h"
#include "sys/ctimer.h"
#include "sys/clock.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/assert.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "LORAMAC"
#define LOG_LEVEL LOG_LEVEL_MAC

/* Constants of the IEEE 802.15.4 standard */

/* macMinBE: Initial backoff exponent. Range 0--CSMA_MAX_BE */
#ifdef CSMA_CONF_MIN_BE
#define CSMA_MIN_BE CSMA_CONF_MIN_BE
#else
#define CSMA_MIN_BE 3
#endif

/* macMaxBE: Maximum backoff exponent. Range 3--8 */
#ifdef CSMA_CONF_MAX_BE
#define CSMA_MAX_BE CSMA_CONF_MAX_BE
#else
#define CSMA_MAX_BE 5
#endif

/* macMaxCSMABackoffs: Maximum number of backoffs in case of channel busy/collision. Range 0--5 */
#ifdef CSMA_CONF_MAX_BACKOFF
#define CSMA_MAX_BACKOFF CSMA_CONF_MAX_BACKOFF
#else
#define CSMA_MAX_BACKOFF 5
#endif

/* macMaxFrameRetries: Maximum number of re-transmissions attampts. Range 0--7 */
#ifdef CSMA_CONF_MAX_FRAME_RETRIES
#define CSMA_MAX_FRAME_RETRIES CSMA_CONF_MAX_FRAME_RETRIES
#else
#define CSMA_MAX_FRAME_RETRIES 7
#endif

/* Packet metadata */
struct qbuf_metadata {
  mac_callback_t sent;
  void *cptr;
  uint8_t max_transmissions;
};

/* Every neighbor has its own packet queue */
struct neighbor_queue {
  struct neighbor_queue *next;
  linkaddr_t addr;
  struct ctimer transmit_timer;
  uint8_t transmissions;
  uint8_t collisions;
  LIST_STRUCT(packet_queue);
};

/* The maximum number of co-existing neighbor queues */
#ifdef CSMA_CONF_MAX_NEIGHBOR_QUEUES
#define CSMA_MAX_NEIGHBOR_QUEUES CSMA_CONF_MAX_NEIGHBOR_QUEUES
#else
#define CSMA_MAX_NEIGHBOR_QUEUES 2
#endif /* CSMA_CONF_MAX_NEIGHBOR_QUEUES */

/* The maximum number of pending packet per neighbor */
#ifdef CSMA_CONF_MAX_PACKET_PER_NEIGHBOR
#define CSMA_MAX_PACKET_PER_NEIGHBOR CSMA_CONF_MAX_PACKET_PER_NEIGHBOR
#else
#define CSMA_MAX_PACKET_PER_NEIGHBOR MAX_QUEUED_PACKETS
#endif /* CSMA_CONF_MAX_PACKET_PER_NEIGHBOR */

#define MAX_QUEUED_PACKETS QUEUEBUF_NUM

/* Neighbor packet queue */
struct packet_queue {
  struct packet_queue *next;
  struct queuebuf *buf;
  void *ptr;
};

MEMB(neighbor_memb, struct neighbor_queue, CSMA_MAX_NEIGHBOR_QUEUES);
MEMB(packet_memb, struct packet_queue, MAX_QUEUED_PACKETS);
MEMB(metadata_memb, struct qbuf_metadata, MAX_QUEUED_PACKETS);
LIST(neighbor_list);

static void packet_sent(struct neighbor_queue *n,
    struct packet_queue *q,
    int status,
    int num_transmissions);
static void transmit_from_queue(void *ptr);
/*---------------------------------------------------------------------------*/
static struct neighbor_queue *
neighbor_queue_from_addr(const linkaddr_t *addr)
{
  struct neighbor_queue *n = list_head(neighbor_list);
  while(n != NULL) {
    if(linkaddr_cmp(&n->addr, addr)) {
      return n;
    }
    n = list_item_next(n);
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
static int
send_one_packet(struct neighbor_queue *n, struct packet_queue *q)
{
  int ret;
  int last_sent_ok = 0;

  memcpy(LoRaMacPacketBufferPtr, packetbuf_dataptr(), packetbuf_datalen());
  *LoRaMacPacketLengthPtr = packetbuf_datalen();
  LoRaMacPacketEmpty = false;

  if(LoRaMacRecvPending == true) {
    /* Currently receiving a packet over air or the radio has
       already received a packet that needs to be read before
       sending with auto ack. */
    ret = MAC_TX_NOACK;
  } else {
    /* for now we polling for ack, this may waste a large time */
    while(LoRaMacPacketEmpty == false) {
      LoRaMacProcessLoopOnes();
    }

    if(LoRaMacPacketAckReceived == false) {
      ret = MAC_TX_NOACK;
    } else {
      ret = MAC_TX_OK;
    }
  }

  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }

  packet_sent(n, q, ret, 1);
  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
transmit_from_queue(void *ptr)
{
  struct neighbor_queue *n = ptr;
  if(n) {
    struct packet_queue *q = list_head(n->packet_queue);
    if(q != NULL) {
      LOG_INFO("preparing packet for ");
      LOG_INFO_LLADDR(&n->addr);
      LOG_INFO_(", tx %u, queue %d\n",
        n->transmissions, list_length(n->packet_queue));
      /* Send first packet in the neighbor queue */
      queuebuf_to_packetbuf(q->buf);
      send_one_packet(n, q);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_transmission(struct neighbor_queue *n)
{
  transmit_from_queue(n);
}
/*---------------------------------------------------------------------------*/
static void
free_packet(struct neighbor_queue *n, struct packet_queue *p, int status)
{
  if(p != NULL) {
    /* Remove packet from queue and deallocate */
    list_remove(n->packet_queue, p);

    queuebuf_free(p->buf);
    memb_free(&metadata_memb, p->ptr);
    memb_free(&packet_memb, p);
    LOG_DBG("free_queued_packet, queue length %d, free packets %d\n",
           list_length(n->packet_queue), memb_numfree(&packet_memb));
    if(list_head(n->packet_queue) != NULL) {
      /* There is a next packet. We reset current tx information */
      n->transmissions = 0;
      n->collisions = 0;
      /* Schedule next transmissions */
      schedule_transmission(n);
    } else {
      /* This was the last packet in the queue, we free the neighbor */
      ctimer_stop(&n->transmit_timer);
      list_remove(neighbor_list, n);
      memb_free(&neighbor_memb, n);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
tx_done(int status, struct packet_queue *q, struct neighbor_queue *n)
{
  mac_callback_t sent;
  struct qbuf_metadata *metadata;
  void *cptr;
  uint8_t ntx;

  metadata = (struct qbuf_metadata *)q->ptr;
  sent = metadata->sent;
  cptr = metadata->cptr;
  ntx = n->transmissions;

  LOG_INFO("packet sent to ");
  LOG_INFO_LLADDR(&n->addr);
  LOG_INFO_(", status %u, tx %u, coll %u\n",
              status, n->transmissions, n->collisions);

  free_packet(n, q, status);
  mac_call_sent_callback(sent, cptr, status, ntx);
}
/*---------------------------------------------------------------------------*/
static void
rexmit(struct packet_queue *q, struct neighbor_queue *n)
{
  schedule_transmission(n);
  /* This is needed to correctly attribute energy that we spent
     transmitting this packet. */
  queuebuf_update_attr_from_packetbuf(q->buf);
}
/*---------------------------------------------------------------------------*/
static void
noack(struct packet_queue *q, struct neighbor_queue *n, int num_transmissions)
{
  struct qbuf_metadata *metadata;

  metadata = (struct qbuf_metadata *)q->ptr;

  n->collisions = 0;
  n->transmissions += num_transmissions;

  if(n->transmissions >= metadata->max_transmissions) {
    tx_done(MAC_TX_NOACK, q, n);
  } else {
    rexmit(q, n);
  }
}
/*---------------------------------------------------------------------------*/
static void
tx_ok(struct packet_queue *q, struct neighbor_queue *n, int num_transmissions)
{
  n->collisions = 0;
  n->transmissions += num_transmissions;
  tx_done(MAC_TX_OK, q, n);
}
/*---------------------------------------------------------------------------*/
static void
packet_sent(struct neighbor_queue *n,
    struct packet_queue *q,
    int status,
    int num_transmissions)
{
  assert(n != NULL);
  assert(q != NULL);

  if(q->ptr == NULL) {
    LOG_WARN("packet sent: no metadata\n");
    return;
  }

  LOG_INFO("tx to ");
  LOG_INFO_LLADDR(&n->addr);
  LOG_INFO_(", status %u, tx %u, coll %u\n",
            status, n->transmissions, n->collisions);

  switch(status) {
  case MAC_TX_OK:
    tx_ok(q, n, num_transmissions);
    break;
  case MAC_TX_NOACK:
    noack(q, n, num_transmissions);
    break;
  case MAC_TX_COLLISION:
    break;
  case MAC_TX_DEFERRED:
    break;
  default:
    tx_done(status, q, n);
    break;
  }
}
/*---------------------------------------------------------------------------*/
void
loramac_output_packet(mac_callback_t sent, void *ptr)
{
  struct packet_queue *q;
  struct neighbor_queue *n;
  const linkaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

  /* Look for the neighbor entry */
  n = neighbor_queue_from_addr(addr);
  if(n == NULL) {
    /* Allocate a new neighbor entry */
    n = memb_alloc(&neighbor_memb);
    if(n != NULL) {
      /* Init neighbor entry */
      linkaddr_copy(&n->addr, addr);
      n->transmissions = 0;
      n->collisions = 0;
      /* Init packet queue for this neighbor */
      LIST_STRUCT_INIT(n, packet_queue);
      /* Add neighbor to the neighbor list */
      list_add(neighbor_list, n);
    }
  }

  if(n != NULL) {
    /* Add packet to the neighbor's queue */
    if(list_length(n->packet_queue) < CSMA_MAX_PACKET_PER_NEIGHBOR) {
      q = memb_alloc(&packet_memb);
      if(q != NULL) {
        q->ptr = memb_alloc(&metadata_memb);
        if(q->ptr != NULL) {
          q->buf = queuebuf_new_from_packetbuf();
          if(q->buf != NULL) {
            struct qbuf_metadata *metadata = (struct qbuf_metadata *)q->ptr;
            /* Neighbor and packet successfully allocated */
            metadata->max_transmissions = packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
            if(metadata->max_transmissions == 0) {
              /* If not set by the application, use the default CSMA value */
              metadata->max_transmissions = 1; // for now let LoRaMac take care retransmitting
            }
            metadata->sent = sent;
            metadata->cptr = ptr;
            list_add(n->packet_queue, q);

            LOG_INFO("sending to ");
            LOG_INFO_LLADDR(addr);
            LOG_INFO_(", len %u, queue length %d, free packets %d\n",
                    packetbuf_datalen(),
                    list_length(n->packet_queue), memb_numfree(&packet_memb));
            /* If q is the first packet in the neighbor's queue, send asap */
            if(list_head(n->packet_queue) == q) {
              schedule_transmission(n);
            }
            return;
          }
          memb_free(&metadata_memb, q->ptr);
          LOG_WARN("could not allocate queuebuf, dropping packet\n");
        }
        memb_free(&packet_memb, q);
        LOG_WARN("could not allocate queuebuf, dropping packet\n");
      }
      /* The packet allocation failed. Remove and free neighbor entry if empty. */
      if(list_length(n->packet_queue) == 0) {
        list_remove(neighbor_list, n);
        memb_free(&neighbor_memb, n);
      }
    } else {
      LOG_WARN("Neighbor queue full\n");
    }
    LOG_WARN("could not allocate packet, dropping packet\n");
  } else {
    LOG_WARN("could not allocate neighbor, dropping packet\n");
  }
  mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1);
}
/*---------------------------------------------------------------------------*/
void
loramac_output_init(void)
{
  memb_init(&packet_memb);
  memb_init(&metadata_memb);
  memb_init(&neighbor_memb);
}
