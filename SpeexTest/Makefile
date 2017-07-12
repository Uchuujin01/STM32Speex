######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = SpeexTest

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
C_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
  Src/main.c \
  Src/stm32f1xx_hal_msp.c \
  Src/stm32f1xx_it.c \
  STM32F10x_Speex_Lib/libspeex/bits.c \
  STM32F10x_Speex_Lib/libspeex/buffer.c \
  STM32F10x_Speex_Lib/libspeex/exc_10_16_table.c \
  STM32F10x_Speex_Lib/libspeex/exc_10_32_table.c \
  STM32F10x_Speex_Lib/libspeex/exc_20_32_table.c \
  STM32F10x_Speex_Lib/libspeex/exc_5_256_table.c \
  STM32F10x_Speex_Lib/libspeex/exc_5_64_table.c \
  STM32F10x_Speex_Lib/libspeex/exc_8_128_table.c \
  STM32F10x_Speex_Lib/libspeex/fftwrap.c \
  STM32F10x_Speex_Lib/libspeex/filterbank.c \
  STM32F10x_Speex_Lib/libspeex/gain_table.c \
  STM32F10x_Speex_Lib/libspeex/gain_table_lbr.c \
  STM32F10x_Speex_Lib/libspeex/hexc_10_32_table.c \
  STM32F10x_Speex_Lib/libspeex/hexc_table.c \
  STM32F10x_Speex_Lib/libspeex/high_lsp_tables.c \
  STM32F10x_Speex_Lib/libspeex/jitter.c \
  STM32F10x_Speex_Lib/libspeex/kiss_fft.c \
  STM32F10x_Speex_Lib/libspeex/kiss_fftr.c \
  STM32F10x_Speex_Lib/libspeex/lpc.c \
  STM32F10x_Speex_Lib/libspeex/lsp.c \
  STM32F10x_Speex_Lib/libspeex/lsp_tables_nb.c \
  STM32F10x_Speex_Lib/libspeex/mdf.c \
  STM32F10x_Speex_Lib/libspeex/modes_wb.c \
  STM32F10x_Speex_Lib/libspeex/preprocess.c \
  STM32F10x_Speex_Lib/libspeex/resample.c \
  STM32F10x_Speex_Lib/libspeex/sb_celp.c \
  STM32F10x_Speex_Lib/libspeex/smallft.c \
  STM32F10x_Speex_Lib/libspeex/speex.c \
  STM32F10x_Speex_Lib/libspeex/speex_callbacks.c \
  STM32F10x_Speex_Lib/libspeex/speex_header.c \
  STM32F10x_Speex_Lib/libspeex/stereo.c \
  STM32F10x_Speex_Lib/libspeex/testdenoise.c \
  STM32F10x_Speex_Lib/libspeex/testecho.c \
  STM32F10x_Speex_Lib/libspeex/testenc.c \
  STM32F10x_Speex_Lib/libspeex/testenc_uwb.c \
  STM32F10x_Speex_Lib/libspeex/testenc_wb.c \
  STM32F10x_Speex_Lib/libspeex/vbr.c \
  STM32F10x_Speex_Lib/libspeex/window.c \
  STM32F10x_Speex_Lib/STM32/libspeex/cb_search.c \
  STM32F10x_Speex_Lib/STM32/libspeex/filters.c \
  STM32F10x_Speex_Lib/STM32/libspeex/ltp.c \
  STM32F10x_Speex_Lib/STM32/libspeex/modes.c \
  STM32F10x_Speex_Lib/STM32/libspeex/nb_celp.c \
  STM32F10x_Speex_Lib/STM32/libspeex/quant_lsp.c \
  STM32F10x_Speex_Lib/STM32/libspeex/vq.c  
ASM_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s \
  STM32F10x_Speex_Lib/STM32/libspeex/gcc/filters_cortexM3.s \
  STM32F10x_Speex_Lib/STM32/libspeex/gcc/ltp_cortexM3.s \
  STM32F10x_Speex_Lib/STM32/libspeex/gcc/vq_cortexm3.s \

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F103xB
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IDrivers/CMSIS/Device/ST/STM32F1xx/Include
C_INCLUDES += -IDrivers/CMSIS/Include
C_INCLUDES += -IDrivers/STM32F1xx_HAL_Driver/Inc
C_INCLUDES += -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy
C_INCLUDES += -IInc
C_INCLUDES += -ISTM32F10x_Speex_Lib/include/speex
C_INCLUDES += -ISTM32F10x_Speex_Lib/libspeex
C_INCLUDES += -ISTM32F10x_Speex_Lib/STM32
C_INCLUDES += -ISTM32F10x_Speex_Lib/STM32/include/speex
C_INCLUDES += -ISTM32F10x_Speex_Lib/STM32/libspeex
C_INCLUDES += -ISTM32F10x_Speex_Lib/STM32/libspeex/gcc
# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m3 $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m3 $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -std=c99 -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103CBTx_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = -mthumb -mcpu=cortex-m3 -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean all

# *** EOF ***
