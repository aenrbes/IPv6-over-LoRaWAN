/*
 * (c) 2018 - idlab - UGent - imec
 *
 * Bart Moons
 *
 * This file is part of the SCHC stack implementation
 *
 * This is a basic example on how to fragment
 * and reassemble a packet
 * The client will send every 10 seconds a fragment
 * to the network gateway, which will reassemble the packet
 *
 */

#include <stdio.h>
#include <stdint.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "contiki.h"
#include "dev/watchdog.h"
#include "net/link-stats.h"
#include "net/ipv6/uipopt.h"
#include "net/ipv6/tcpip.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uipbuf.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/routing/routing.h"
#include "sys/ctimer.h"
#include "lib/memb.h"

#include "schc.h"
#include "compressor.h"
#include "fragmenter.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "SCHC"
#define LOG_LEVEL LOG_LEVEL_6LOWPAN

#define MAX_PACKET_LENGTH		1280

#if UIP_CONF_ROUTER
static linkaddr_t devid_lladdr[SCHC_CONF_RX_CONNS];
#endif

static int TX_RUNNING = 0;
static int counter = 1;

struct cb_t {
    schc_fragmentation_t* conn;
    void (*cb)(schc_fragmentation_t* conn);
    struct cb_t *next;
    struct ctimer timer;
};

static struct cb_t *rx_cb_head = NULL;
static struct cb_t *tx_cb_head = NULL;

// structure to keep track of the transmission
static schc_fragmentation_t tx_conn;

/**
 * A pointer to the packetbuf buffer.
 * We initialize it to the beginning of the packetbuf buffer, then
 * access different fields by updating the offset packetbuf_hdr_len.
 */
static uint8_t *packetbuf_ptr;

/**
 * mac_max_payload is the maimum payload space on the MAC frame.
 */
static int mac_max_payload;

/**
 * the result of the last transmitted fragment
 */
static int last_tx_status;

static struct queuebuf *output_qbuf;

static int last_rssi;

MEMB(timer_cb_memb, struct cb_t, 20);

static void
cleanup(struct cb_t* head)
{
  if(head) {
  	struct cb_t* curr = head;
  	struct cb_t* next = curr->next;
  	struct cb_t* prev = NULL;

  	while(curr != NULL) {
      ctimer_stop(&curr->timer);
  		memb_free(&timer_cb_memb, curr);
  		curr = curr->next;
  	}
  }
}

static void
timer_handler(void* user_data)
{
	struct cb_t* cb_t_ = (struct cb_t*) user_data;
	schc_fragmentation_t* conn = cb_t_->conn;
  LOG_DBG("_____________________timer_handler()________________________\n");
  ctimer_stop(&cb_t_->timer);
	cb_t_->cb(conn);
}

/*
 * The timer used by the SCHC library to schedule the transmission of fragments
 */
static void
set_tx_timer(void (*callback)(void *conn),
		uint32_t device_id, uint32_t delay, void *arg)
{
	struct cb_t* cb_t_= memb_alloc(&timer_cb_memb); // create on heap

	cb_t_->conn = arg;
	cb_t_->cb = callback;
  cb_t_->next = NULL;

	struct cb_t* curr = tx_cb_head;
	if(tx_cb_head == NULL) {
		tx_cb_head = cb_t_;
	} else {
    if(curr->conn == cb_t_->conn) {
      memb_free(&timer_cb_memb, cb_t_);
      ctimer_set(&curr->timer, delay, timer_handler, curr);
      LOG_DBG(
			"set_tx_timer(): reschedule next tx callback %d s \n\r\n", delay / 1000);
      return;
    }
		while(curr->next != NULL) {
      if(curr->conn == cb_t_->conn) {
        memb_free(&timer_cb_memb, cb_t_);
        ctimer_set(&curr->timer, delay, timer_handler, curr);
        LOG_DBG(
				"set_tx_timer(): reschedule next tx callback %d s \n\r\n", delay / 1000);
        return;
      }
			curr = curr->next;
		}
		curr->next = cb_t_;
	}

	counter++;
	LOG_DBG("\n+-------- TX  %02d --------+\r\n", counter);

  ctimer_set(&cb_t_->timer, delay, timer_handler, cb_t_);

	LOG_DBG(
				"set_tx_timer(): schedule next tx state check in %d s \n\r\n", delay / 1000);
}

/*
 * The timer used by the SCHC library to time out the reception of fragments
 * should have multiple timers for a device
 */
static void
set_rx_timer(void (*callback)(void *conn),
		uint32_t device_id, uint32_t delay, void *arg)
{
	struct cb_t* cb_t_= memb_alloc(&timer_cb_memb); // create on heap
	cb_t_->conn = arg;
	cb_t_->cb = callback;
  cb_t_->next = NULL;

	struct cb_t* curr = rx_cb_head;
	if(rx_cb_head == NULL) {
		rx_cb_head = cb_t_;
	} else {
    if(curr->conn == cb_t_->conn) {
      memb_free(&timer_cb_memb, cb_t_);
      ctimer_restart(&curr->timer);
      LOG_DBG(
			"set_rx_timer(): reschedule rx callback %d s \n\r\n", delay / 1000);
      return;
    }
		while(curr->next != NULL) {
      if(curr->conn == cb_t_->conn) {
        memb_free(&timer_cb_memb, cb_t_);
        ctimer_restart(&curr->timer);
        LOG_DBG(
				"set_rx_timer(): reschedule rx callback %d s \n\r\n", delay / 1000);
        return;
      }
			curr = curr->next;
		}
		curr->next = cb_t_;
	}

  ctimer_set(&cb_t_->timer, delay, timer_handler, cb_t_);

	LOG_DBG(
				"set_rx_timer(): schedule rx callback %d s \n\r\n", delay / 1000);
}

/*
 * Callback to remove a timer entry for a device
 * (required by some timer libraries)
 */
static void
remove_timer_entry(uint32_t device_id)
{
	LOG_DBG("remove_timer_entry(): remove timer entry for device with id %d \r\n", device_id);
}

/*
 * Callback to handle the end of a fragmentation sequence
 */
static void
end_tx()
{
	LOG_DBG("end_tx() callback \r\n");
  queuebuf_free(output_qbuf);
  cleanup(tx_cb_head);
  tx_cb_head = NULL;
  TX_RUNNING = 0;
}

/*
 * Callback to handle the end of a fragmentation sequence
 * may be used to forward packet to IP network
 */
static void
end_rx(schc_fragmentation_t *conn)
{
	LOG_DBG("end_rx(): copy mbuf contents to message buffer \r\n");

	uint16_t packetlen = get_mbuf_len(conn); // calculate the length of the original packet
	uint8_t rx_compressed_packet[1280]; // todo pass the mbuf chain to the decompressor
  direction flow_dir;

#if UIP_CONF_ROUTER
  flow_dir = UP;
#else
  flow_dir = DOWN;
#endif

  /* Clear uipbuf and set default attributes */
  uipbuf_clear();

	mbuf_copy(conn, rx_compressed_packet); // copy the packet from the mbuf list

	LOG_DBG("end_rx(): decompress packet \r\n");
	schc_bitarray_t bit_arr;
	bit_arr.ptr = rx_compressed_packet;
	uip_len = schc_decompress(&bit_arr, (uint8_t *)UIP_IP_BUF,
			conn->device_id, packetlen, flow_dir);
	if (uip_len == 0) { // some error occured
		LOG_ERR("decompressed schc packet lenth is 0!\r\n");
	}

	LOG_DBG("end_rx(): forward packet to IP network \r\n");

	schc_reset(conn);
  cleanup(rx_cb_head);
  rx_cb_head = NULL;

  tcpip_input();
}

static void
schc_drv_input(void)
{
  int i;
  uint32_t device_id;
  linkaddr_t *src_lladdr;

	LOG_DBG("\n+-------- RX  %02d --------+\r\n", counter);

  src_lladdr = packetbuf_addr(PACKETBUF_ADDR_SENDER);

  device_id = 1; // device node always use device id 0x01
#if UIP_CONF_ROUTER
  for(i = 0; i < SCHC_CONF_RX_CONNS; i++) {
    device_id = i + 1;
    if(linkaddr_cmp(&devid_lladdr[i], &linkaddr_null)) {
      devid_lladdr[i] = *src_lladdr;
      break;
    } else if (linkaddr_cmp(&devid_lladdr[i], src_lladdr)) {
      break;
    }
  }
#endif

  /* Update link statistics */
  link_stats_input_callback(src_lladdr);

  /* The MAC puts the lorawan payload inside the packetbuf data buffer */
  packetbuf_ptr = packetbuf_dataptr();

  if(packetbuf_datalen() == 0) {
    LOG_WARN("input: empty packet\r\n");
    return;
  }

  /* Save the RSSI of the incoming packet in case the upper layer will
     want to query us for it later. */
  last_rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

	schc_fragmentation_t *conn = schc_input(packetbuf_ptr, packetbuf_datalen(),
			&tx_conn, device_id); // get active connection and set the correct rule for this connection
  if(!conn) {
    LOG_WARN("input: no rx_conn\r\n");
    return;
  }

	if (conn != &tx_conn) { // if returned value is tx_conn: acknowledgement is received, which is handled by the library
		conn->post_timer_task = set_rx_timer;
		conn->dc = 30000; // retransmission timer: used for timeouts

		if (conn->schc_rule->mode == NOT_FRAGMENTED) { // packet was not fragmented
			end_rx(conn);
		} else {
			int ret = schc_reassemble(conn);
			if(ret && conn->schc_rule->mode == NO_ACK){ // use the connection to reassemble
				end_rx(conn); // final packet arrived
			}
		}
	}
}

/**
 * Callback function for the MAC packet sent callback
 */
static void
packet_sent(void *ptr, int status, int transmissions)
{
  const linkaddr_t *dest;

  last_tx_status = status;

  /* What follows only applies to unicast */
  dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }

  /* Update neighbor link statistics */
  link_stats_packet_sent(dest, status, transmissions);

  /* Call routing protocol link callback */
  NETSTACK_ROUTING.link_callback(dest, status, transmissions);

  /* DS6 callback, used for UIP_DS6_LL_NUD */
  uip_ds6_link_callback(status, transmissions);
}

/*
 * Callback to handle transmission of fragments
 *
 * should return 1 or 0 to inform the fragmenter
 * whether the network driver is busy or not
 * @ret		0				the packet was not sent
 * 			  1				the packet was transmitted
 */
static uint8_t
tx_send_callback(uint8_t* data, uint16_t length, uint32_t device_id)
{
	LOG_DBG("tx_send_callback(): transmitting packet with length %d for device %d \r\n", length, device_id);
  uint8_t ret;

  /* Restore packetbuf from queuebuf */
  queuebuf_to_packetbuf(output_qbuf);
  queuebuf_free(output_qbuf);

  /* Now copy fragment payload from uip_buf */
  memcpy(packetbuf_ptr, data, length);
  packetbuf_set_datalen(length);

  /* Provide a callback function to receive the result of
     a packet transmission. */
  NETSTACK_MAC.send(&packet_sent, NULL);

  /* If we are sending multiple packets in a row, we need to let the
     watchdog know that we are still alive. */
  watchdog_periodic();

  output_qbuf = queuebuf_new_from_packetbuf();
  if(output_qbuf == NULL) {
    LOG_WARN("output: could not allocate queuebuf, dropping fragment\r\n");
    return 0;
  }

	return (last_tx_status == MAC_TX_OK) ? 1 : 0;
}

static uint8_t
schc_drv_output(const linkaddr_t *localdest)
{
  /* The MAC address of the destination of the packet */
  linkaddr_t dest;

	static uint8_t tx_compressed_packet[MAX_PACKET_LENGTH];
	static struct schc_rule_t* schc_rule;
	static schc_bitarray_t bit_arr;
  direction flow_dir = UP;
  uint32_t device_id;
  int i;

  if(TX_RUNNING) {
    LOG_INFO("a ip packet is in transmitting, drop incoming packet!\r\n");
    return 0;
  }

  device_id = 1; // device node always use device id 0x01
#if UIP_CONF_ROUTER
  flow_dir = DOWN;
  for(i = 0; i < SCHC_CONF_RX_CONNS; i++) {
    device_id = i + 1;
    if(linkaddr_cmp(&devid_lladdr[i], &linkaddr_null)) {
      devid_lladdr[i] = *localdest;
      break;
    } else if (linkaddr_cmp(&devid_lladdr[i], localdest)) {
      break;
    }
  }
#endif

    /* reset packetbuf buffer */
  packetbuf_clear();
  packetbuf_ptr = packetbuf_dataptr();

  /*
   * The destination address will be tagged to each outbound
   * packet. If the argument localdest is NULL, we are sending a
   * broadcast packet.
   */
  if(localdest == NULL) {
    linkaddr_copy(&dest, &linkaddr_null);
  } else {
    linkaddr_copy(&dest, localdest);
  }

  LOG_INFO("output: sending IPv6 packet with len %d\r\n", uip_len);

  /* copy over the retransmission count from uipbuf attributes */
  packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS,
                     uipbuf_get_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS));

  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &dest);

  mac_max_payload = NETSTACK_MAC.max_payload();
  if(mac_max_payload <= 0) {
  /* Framing failed, drop packet */
    LOG_WARN("output: failed to calculate payload size - dropping packet\r\n");
    return 0;
  }

	// compress packet
	bit_arr.ptr = (uint8_t *)tx_compressed_packet;
	schc_rule = schc_compress((uint8_t *)UIP_IP_BUF, uip_len, &bit_arr, device_id, flow_dir);

	tx_conn.mtu = mac_max_payload; // network driver MTU
	tx_conn.dc = 10000; // 10 seconds duty cycle
	tx_conn.device_id = device_id; // the device id of the connection

	tx_conn.bit_arr = &bit_arr;
	tx_conn.send = &tx_send_callback;
	tx_conn.end_tx = &end_tx;

	tx_conn.schc_rule = schc_rule;
	tx_conn.RULE_SIZE = RULE_SIZE_BITS;

#if UIP_CONF_ROUTER
	tx_conn.MODE = ACK_ALWAYS;
#else
  tx_conn.MODE = ACK_ALWAYS;
#endif

	tx_conn.post_timer_task = &set_tx_timer;

	if (schc_rule == NULL) {
    LOG_ERR("NO schc rule found!\r\n");
		cleanup(tx_cb_head);
		return 0;
	}

  /* Reset last tx status -- MAC layers most often call packet_sent asynchrously */
  last_tx_status = MAC_TX_OK;
  counter = 1;
	// start fragmentation loop
	LOG_DBG("+-------- TX  %02d --------+\r\n", counter);

  /* Backup packetbuf to queuebuf. Enables preserving attributes for all framgnets */
  output_qbuf = queuebuf_new_from_packetbuf();
  if(output_qbuf == NULL) {
    LOG_WARN("output: could not allocate queuebuf, dropping fragment\r\n");
    cleanup(tx_cb_head);
    return 0;
  }

  TX_RUNNING = 1;

	if(schc_fragment(&tx_conn) == SCHC_NO_FRAGMENTATION) {
    if(last_tx_status != MAC_TX_OK){
      LOG_INFO("TX is busy, drop incoming packet!\r\n");
      return 0;
    }
  }

  return 1;
}

static void
schc_drv_init()
{
	uint8_t src[16] = { 0 };

  TX_RUNNING = 0;

#if UIP_CONF_ROUTER
  memset(devid_lladdr, 0, sizeof(devid_lladdr));
#endif

  memb_init(&timer_cb_memb);

	schc_compressor_init(src);

	// initialize fragmenter for constrained device
	schc_fragmenter_init(&tx_conn, &tx_send_callback, &end_rx, &remove_timer_entry);
}

const struct network_driver schc_driver = {
  "schc",
  schc_drv_init,
  schc_drv_input,
  schc_drv_output
};
