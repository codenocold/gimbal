######################################
# target
######################################
TARGET ?= dmtp_boot
BUILD_DIR ?= build

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O3

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32g4xx_it.c \
Core/Src/system_stm32g4xx.c \
Core/Src/sysmem.c \
Core/Src/syscalls.c \
Core/Src/stm32g4xx_hal_msp.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_lpuart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
Source/rtt/SEGGER_RTT.c \
Source/rtt/SEGGER_RTT_printf.c \
Source/eeprom/core/eeprom_emul.c \
Source/eeprom/port/flash_interface.c \
Source/flash.c \
Source/soc.c \
Source/util.c \
Source/com_485.c \
Source/com_can.c \
Source/aes.c \
Source/dfu.c \
Source/com.c \
Source/mpu6050.c \
Source/imu.c \
Source/Fusion/FusionAhrs.c \
Source/Fusion/FusionBias.c \
Source/Fusion/FusionCompass.c \


# ASM sources
ASM_SOURCES =  \
startup_stm32g431xx.s

# ASM sources
ASMM_SOURCES =

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS :=
C_DEFS += -DSTM32G431xx
C_DEFS += -DARM_MATH_CM4
C_DEFS += -DUSE_HAL_DRIVER
C_DEFS += -DUSE_FULL_LL_DRIVER
ifeq ($(DEBUG), 1)
C_DEFS += -D__DEBUG__
endif

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include \
-IMiddlewares/ST/ARM/DSP/Inc \
-ISource \
-ISource/RTT \
-ISource/eeprom/core \
-ISource/eeprom/port \
-ISource/Fusion \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = stm32g431cbux_flash.ld

# libraries
LIBS = -lc -lm -lnosys  \
-l:libarm_cortexM4lf_math.a
LIBDIR =  \
-LMiddlewares/ST/ARM/DSP/Lib
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

LDFLAGS += -Wl,--no-warn-rwx-segments

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
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	@echo compile: $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@
	@echo compile: $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@
	@echo compile: $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@echo making elf: $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(HEX) $< $@
	@echo making hex: $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(BIN) $< $@
	@echo making bin: $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
