
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-cpp
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-gcc
TRACE_CC = @echo "  CC       " $<
TRACE_LD = @echo "  LD       " $@
TRACE_AR = @echo "  AR       " $@
TRACE_AS = @echo "  AS       " $<
Q=@

OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM      = arm-none-eabi-nm
SIZE    = arm-none-eabi-size
SREC_CAT = srec_cat

CONTIKI = ../../..
OTA_SOURCE = ../apps/ota

CPU_ABS_PATH = ./loramac
TARGET=lora-node-151
OBJECTDIR=obj_$(TARGET)

CFLAGS += -DUSE_HAL_DRIVER -DSTM32L151xC -DUSE_BAND_470 -DUSE_DEBUGGER -DUSE_NO_TIMER
# Optimize for code size
CFLAGS += -O0

LDSCRIPT = ota-bootloader.ld
LDFLAGS += --specs=nosys.specs -static --specs=nano.specs -Wall
LDFLAGS += -mcpu=cortex-m3 -mthumb -mlittle-endian
LDFLAGS += -T $(LDSCRIPT)
#LDFLAGS += -g
LDFLAGS += -Wl,--gc-sections,--sort-section=alignment
LDFLAGS += -Wl,-Map=$(@:.elf=-$(TARGET).map),--cref
OBJCOPY_FLAGS += -O binary --gap-fill 0xff
OBJDUMP_FLAGS += --disassemble --source --disassembler-options=force-thumb

STM32_HAL = $(CPU_ABS_PATH)/STM32_HAL_Drivers

CFLAGS += -I$(CPU_ABS_PATH)/mac/Inc
CFLAGS += -I$(OTA_SOURCE)
CFLAGS += -I$(STM32_HAL)/CMSIS/Include
CFLAGS += -I$(STM32_HAL)/STM32L1xx_HAL_Driver/Inc
CFLAGS += -I$(STM32_HAL)/CMSIS/Device/ST/STM32L1xx/Include
CFLAGS += -I$(STM32_HAL)/STM32L1xx_HAL_Driver/Inc/Legacy

SOURCEDIRS += .
SOURCEDIRS += ./loramac/STM32_HAL_Drivers/STM32L1xx_HAL_Driver/Src
SOURCEDIRS += ./loramac/mac
vpath %.c $(SOURCEDIRS)
SOURCE_FILES = $(foreach dir, $(SOURCEDIRS), $(notdir $(wildcard $(dir)/*.c)))
SOURCE_FILES += startup_stm32l151ccux.s
SOURCE_FILES += ota.c
vpath ota.c $(OTA_SOURCE)
vpath %.s $(SOURCEDIRS)


CFLAGS += -mcpu=cortex-m3
CFLAGS += -mthumb -mabi=aapcs -mlittle-endian
CFLAGS += -Wall
CFLAGS += -std=gnu11
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fomit-frame-pointer -fno-builtin
CFLAGS += -fno-unroll-loops -ffast-math -ftree-vectorize

### Compilation rules
### Don't treat the .elf as intermediate
.PRECIOUS: %.elf %.hex %.bin

oname = ${patsubst %.c,%.o,${patsubst %.s,%.o,$(1)}}
OBJECTFILES = ${addprefix $(OBJECTDIR)/,${call oname, $(SOURCE_FILES)}}

all: $(OBJECTFILES)

$(OBJECTDIR)/%.o: %.c | $(OBJECTDIR)
	$(TRACE_CC)
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

### Pass CFLAGS along to assembly files in the build
ASFLAGS += $(CFLAGS)
### Specify '-c' option to assemble only and not link
ASFLAGS += -c 

$(OBJECTDIR)/%.o: %.s | $(OBJECTDIR)
	$(TRACE_AS)
	$(Q)$(AS) $(ASFLAGS) -o $@ $<

$(OBJECTDIR):
	mkdir $@

%.elf: $(OBJECTFILES) $(LDSCRIPT)
	$(TRACE_LD)
	$(Q)$(LD) $(LDFLAGS) -Wl,--start-group ${filter-out $(LDSCRIPT) %.a,$^} ${filter %.a,$^} -Wl,--end-group -lm -o $@

%.i16hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.hex: %.i16hex
	$(SREC_CAT) $< -intel -o $@ -intel

%.bin: %.elf
	$(OBJCOPY) $(OBJCOPY_FLAGS) $< $@

%.lst: %.elf
	$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@

clean:
	-rm -f *~ *core core *.srec \
	*.lst *.map \
	*.cprg *.bin *.elf *.hex *.data contiki*.a *.firmware core-labels.S *.ihex *.ini \
	*.ce *.co
	rm -rf $(CLEAN)
	-rm -rf $(OBJECTDIR)
