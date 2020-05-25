/*
 * Copyright (c) 2001, Adam Dunkels.
 * Copyright (c) 2009, 2010 Joakim Eriksson, Niclas Finne, Dogan Yazar.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "contiki.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mosquitto.h>

#include "net/netstack.h"
#include "net/packetbuf.h"

#include "base64.h"
#include "parson.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "BR_dev"
#define LOG_LEVEL LOG_LEVEL_INFO

void packet_sent(uint8_t status, uint8_t tx);

extern const char *slip_config_appid;

struct mosquitto *mosq;

static char sub_topic[128] = "\0";

static struct userdata__callback cb_userdata;

static uint8_t recv_buf[128];
static uint8_t recv_len;
static linkaddr_t sender_lladdr;
bool lorawan_recv_pending;

struct userdata__callback {
  const char *topic;
  int (*callback)(struct mosquitto *, void *, const struct mosquitto_message *);
  void *userdata;
  int qos;
};

PROCESS(lorawan_recv_process, "lorawan recv process");

static int
devEUI_to_lladdr(const char *addrstr, linkaddr_t *lladdr)
{
  uint8_t value;
  int tmp;
  unsigned int len;
  char c = 0;  //gcc warning if not initialized

  value = 0;

  for(len = 0; len < sizeof(linkaddr_t); len++) {
    c = *addrstr;
    if(c >= '0' && c <= '9') {
      tmp = c - '0';
    } else if(c >= 'a' && c <= 'f') {
      tmp = c - 'a' + 10;
    } else if(c >= 'A' && c <= 'F') {
      tmp = c - 'A' + 10;
    } else {
      LOG_ERR("illegal char: '%c'\n", c);
      return 0;
    }
    value = (value << 4) + (tmp & 0xf);

    addrstr++;
    c = *addrstr;
    if(c >= '0' && c <= '9') {
      tmp = c - '0';
    } else if(c >= 'a' && c <= 'f') {
      tmp = c - 'a' + 10;
    } else if(c >= 'A' && c <= 'F') {
      tmp = c - 'A' + 10;
    } else {
      LOG_ERR("illegal char: '%c'\n", c);
      return 0;
    }
    addrstr++;
    value = (value << 4) + (tmp & 0xf);
    lladdr->u8[len] = value;
  }
  if(*addrstr != '\0') {
    LOG_ERR("too large address\n");
    return 0;
  }
  return 1;
}

static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
  struct userdata__callback *userdata = obj;

  (void)rc;
  printf("--------------on_connect 01---------------\n");
    printf("userdata->topic : %s", userdata->topic);
  mosquitto_subscribe(mosq, NULL, userdata->topic, userdata->qos);
  printf("--------------on_connect 02---------------\n");
}

static void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
  int rc;
  struct userdata__callback *userdata = obj;

  rc = userdata->callback(mosq, userdata->userdata, message);
  if(rc){
    mosquitto_disconnect(mosq);
  }
}

//-------------------------------------------------------------------------------------------------
static uint32_t appHexToString(/*IN*/  const char    * pHex,  
                     /*IN*/  uint32_t           hexLen,  
                     /*OUT*/ char          * pByteString)  
{  
  unsigned long i;  
  
  if (pHex==NULL)  
    return 1;  
  
  if(hexLen <= 0)  
    return 2;  
  
  for(i=0;i<hexLen;i++)  
  {  
    if(((pHex[i]&0xf0)>>4)>=0 && ((pHex[i]&0xf0)>>4)<=9)  
      pByteString[2*i]=((pHex[i]&0xf0)>>4)+0x30;  
    else if(((pHex[i]&0xf0)>>4)>=10 && ((pHex[i]&0xf0)>>4)<=16)  
      pByteString[2*i]=((pHex[i]&0xf0)>>4)+0x37;  
    
    if((pHex[i]&0x0f)>=0 && (pHex[i]&0x0f)<=9)  
      pByteString[2*i+1]=(pHex[i]&0x0f)+0x30;  
    else if((pHex[i]&0x0f)>=10 && (pHex[i]&0x0f)<=16)  
      pByteString[2*i+1]=(pHex[i]&0x0f)+0x37;  
  }  
  return 0;  
} 
//-------------------------------------------------------------------------------------------------

static int on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg)
{
  uint8_t json_up[1024] = "\0";
  JSON_Value *root_val = NULL;
  JSON_Object *root_obj = NULL;
  JSON_Value *val = NULL;
  const char *str; /* pointer to sub-strings in the JSON data */
  int i,j;

uint8_t temp1[200]={0};


  if(lorawan_recv_pending == true) {
    return 0;
  }

  memcpy(json_up, msg->payload, msg->payloadlen);
  printf("[ JSON_UP ] %s %s (%d)\n", msg->topic, (const char *)msg->payload, msg->payloadlen);
  json_up[msg->payloadlen] = 0; /* add string terminator, just to be safe */

  root_val = json_parse_string_with_comments((const char *)json_up); /* JSON offset */
  if (root_val == NULL) {
    printf("[ JSON_UP ] WARNING: [down] invalid JSON\n");
    return 0;
  }
  root_obj = json_value_get_object(root_val);

  str = json_object_get_string(root_obj, "devEUI");
  if (str == NULL) {
    printf("[ JSON_UP ]: [down] no \"devEUI\" object in JSON\n");
    json_value_free(root_val);
      return 0;
  }
  printf("[ JSON_UP ]: devEUI:%s\n", str);
  i = devEUI_to_lladdr(str, &sender_lladdr);
  printf("[ JSON_UP ]: dev lladdr:");
  for(j = 0; j < sizeof sender_lladdr; j++) {
    printf("%02x", sender_lladdr.u8[j]);
  }
  printf("\n");
  if(i == 0) {
    json_value_free(root_val);
    return 0;
  }

  val = json_object_get_value(root_obj, "fPort");
  if (val == NULL) {
    printf("[ JSON_UP ]: \"fPort\" object in JSON\n");
    json_value_free(root_val);
    return 0;
  }
  if((uint8_t)json_value_get_number(val) != 2 ) {
    printf("[ JSON_UP ]: \"fPort\" != 2, not for us\n");
    json_value_free(root_val);
    return 0;
  }

  str = json_object_get_string(root_obj, "data");
  if (str == NULL) {
      printf("[ JSON_UP ]: [down] no mandatory \"data\" object in JSON\n");
      json_value_free(root_val);
      return 0;
  }
  i = b64_to_bin(str, strlen(str), recv_buf, sizeof recv_buf);
  if (i > 128) {
      printf("[ JSON_UP ]: [down] too large data obj\n");
      json_value_free(root_val);
      return 0;
  }
  recv_len = i;

memset(temp1,0,200);
appHexToString((const char *)recv_buf, recv_len,(char *)(temp1));
temp1[recv_len * 2]='\0';   
printf("[ JSON_UP ]: recv_buf:%s", (char *)temp1);

  json_value_free(root_val);

  lorawan_recv_pending = true;
  return 0;
}

PROCESS_THREAD(lorawan_recv_process, ev, data)
{
  PROCESS_BEGIN();
  LOG_INFO("lorawan_recv_process init...\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    packetbuf_clear();
    memcpy(packetbuf_dataptr(), recv_buf, recv_len);
    lorawan_recv_pending = false;

    if(recv_len > 0) {
      packetbuf_set_datalen(recv_len);
    }

    packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &sender_lladdr);
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &linkaddr_node_addr);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, -40);

    LOG_INFO("received packet from ");
    LOG_INFO_LLADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER));
    LOG_INFO_("len %u\n", packetbuf_datalen());

    NETSTACK_MAC.input();
  }
  PROCESS_END();
}

void
write_to_slip(char *devEUI, const uint8_t *buf, int len)
{
  int j = 0;
  int buff_index = 0;
  char pub_topic[256] = "\0";
  uint8_t json_down[1024];

  sprintf((char *)json_down, "{\"confirmed\":true,\"fPort\":2,\"data\":\"");
  buff_index += strlen((char *)json_down);
  j = bin_to_b64(buf, len, (char *)(json_down + buff_index), 341); /* 255 bytes = 340 chars in b64 + null char */
  if (j >= 0) {
    buff_index += j;
  } else {
    printf("ERROR: [up] bin_to_b64 failed\n");
    exit(EXIT_FAILURE);
  }

  json_down[buff_index] = '"';
  ++buff_index;
  json_down[buff_index] = '}';
  ++buff_index;
  json_down[buff_index] = 0; /* add string terminator, for safety */

  printf("[ JSON_DOWN ]: %s\n", json_down);

  sprintf(pub_topic, "application/%s/device/%s/tx", slip_config_appid, devEUI);

  printf("[ PUB_TOPIC ]: %s\n", pub_topic);

  j = mosquitto_publish(mosq, NULL, pub_topic, buff_index, json_down, 2, false);
  if (j != MOSQ_ERR_SUCCESS) {
    printf("[ ERORR! ] mosquitto_publish failed ret=%d", j);
    packet_sent(MAC_TX_ERR, 1);
  }
  packet_sent(MAC_TX_OK, 1);
}

int
slip_init(void)
{
  int rc = -1;

  process_start(&lorawan_recv_process, NULL);

  sprintf(sub_topic, "application/%s/device/+/rx", slip_config_appid);
  printf("[MQTT CLIENT] subscribe:%s\n", sub_topic);

  mosquitto_lib_init();

  cb_userdata.topic = sub_topic;
  cb_userdata.qos = 0;
  cb_userdata.userdata = NULL;
  cb_userdata.callback = on_message;

  mosq = mosquitto_new(NULL, true, &cb_userdata);
  if(!mosq){
    rc = MOSQ_ERR_NOMEM;
    goto cleanup;
  }

  mosquitto_connect_callback_set(mosq, on_connect);
  mosquitto_message_callback_set(mosq, on_message_callback);

  rc = mosquitto_connect(mosq, "localhost", 1883, 300);
  if(rc){
    goto destroy;
  }

  rc = mosquitto_loop_start(mosq);
  if(rc) {
    printf("mosquitto loop start failed!\n");
    goto destroy;
  }

  printf("[MQTT CLIENT] slip_init success!\n");
  return 0;

destroy:
  mosquitto_destroy(mosq);
cleanup:
  mosquitto_lib_cleanup();

  return rc;
}
/*---------------------------------------------------------------------------*/
