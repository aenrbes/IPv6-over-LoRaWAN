/**
 *    Example Contiki 3.0 Firmware
 *    Blinks the LEDs @ 1Hz.

 *    If we see the LED blinking, we know
 *    the bootloader loaded our code!
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "coap-engine.h"
#include "coap-blocking-api.h"

#include "ota-download.h"


/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_APP
/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */
extern coap_resource_t res_hello,res_ota;

PROCESS_NAME(ota_download_th);

PROCESS(blinker_test_loop, "GPIO Blinker Lifecycle");
AUTOSTART_PROCESSES(&blinker_test_loop);



PROCESS_THREAD(blinker_test_loop, ev, data)
{
  static struct etimer timer;
  PROCESS_BEGIN();
  //	(1)	UART Output
  printf("OTA Image Example: Starting\n");
  // coap_activate_resource(&res_hello, "test/hello");
  // coap_activate_resource(&res_ota, "test/ota");

  // (3) Get metadata about the current firmware version
  OTAMetadata_t current_firmware;
  get_current_metadata( &current_firmware );
  printf("\nCurrent Firmware\n");
  print_metadata( &current_firmware );

  int ota_slot;
  OTAMetadata_t ota_metadata;

  printf("\nNewest Firmware:\n");
  ota_slot = find_newest_ota_image();
  while( get_ota_slot_metadata( ota_slot, &ota_metadata ) );
  print_metadata( &ota_metadata );

  printf("\nOldest Firmware:\n");
  ota_slot = find_oldest_ota_image();
  while( get_ota_slot_metadata( ota_slot, &ota_metadata ) );
  print_metadata( &ota_metadata );

  int empty_slot = find_empty_ota_slot();
  printf("\nEmpty OTA slot: #%u\n", empty_slot);
  //  (4) OTA Download!

  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 15);

  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

  process_start(&ota_download_th, NULL);
  printf("\nblinker_test_loop end\n");
  PROCESS_END();
}
