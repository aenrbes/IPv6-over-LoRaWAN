CONTIKI_PROJECT = mqtt-client
all: $(CONTIKI_PROJECT)

TARGET = loramac
CONTIKI = ../..

include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/mqtt

-include $(CONTIKI)/Makefile.identify-target

MODULES_REL += arch/platform/$(TARGET)

PLATFORMS_ONLY = loramac

include $(CONTIKI)/Makefile.include
