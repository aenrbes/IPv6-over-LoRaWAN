CONTIKI_PROJECT = ota-image-example
all: $(CONTIKI_PROJECT)

TARGET = loramac

# Include CoAP resources
# MODULES_REL += ./resources
MODULES_REL += ../apps/ota
CONTIKI=../../..

CFLAGS += -DOTA_DEBUG=1 -g -DUSE_BOOTLOADER
OTA = 1
# Include the CoAP implementation
include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/coap
# MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell

include $(CONTIKI)/Makefile.include
