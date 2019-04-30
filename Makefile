######################################
# target
######################################
TARGET = STM32_PD

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES := 
-include HAL_Driver/subdir.mk
C_SOURCES += \
src/error_handler.c \
src/main.c \
src/pd_cc.c \
src/pd_phy.c \
src/pd_policy.c \
src/pd_protocol.c \
src/pd_source.c \
src/pd_tcpc.c \
src/platform.c \
src/stm32f0xx_hal_msp.c \
src/stm32f0xx_it.c \
src/syscalls.c \
src/system_stm32f0xx.c \
src/tcpm.c 

# ASM sources
ASM_SOURCES =  \
startup/startup_stm32.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
BINPATH =
PREFIX = arm-none-eabi-
CC = $(BINPATH)$(PREFIX)gcc
AS = $(BINPATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)$(PREFIX)objcopy
AR = $(BINPATH)$(PREFIX)ar
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32F030x6 \
-DDEBUG

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-Iinc \
-IHAL_Driver/Inc \
-ICMSIS/core \
-ICMSIS/device


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = LinkerScript.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

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
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***