# 命令行编译（可选）：需要 arm-none-eabi-gcc（STM32CubeIDE 自带，或单独安装 Arm GNU Toolchain）
#   make            → build/SmartCar_F407.elf / .hex / .bin
# 一般直接用 STM32CubeIDE 导入本工程编译即可，不需要这个文件。
PREFIX  ?= arm-none-eabi-
CC       = $(PREFIX)gcc
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size
TARGET   = SmartCar_F407
BUILD    = build

MCU      = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS     = -DUSE_HAL_DRIVER -DSTM32F407xx
INCS     = -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
           -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include
CFLAGS   = $(MCU) $(DEFS) $(INCS) -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -ffunction-sections -fdata-sections
LDFLAGS  = $(MCU) -TSTM32F407ZGTX_FLASH.ld --specs=nano.specs --specs=nosys.specs -u _printf_float \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -lm

SRCS     = $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
ASMS     = Core/Startup/startup_stm32f407zgtx.s
OBJS     = $(addprefix $(BUILD)/,$(SRCS:.c=.o)) $(addprefix $(BUILD)/,$(ASMS:.s=.o))

all: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex   $< $(BUILD)/$(TARGET).hex
	$(OBJCOPY) -O binary $< $(BUILD)/$(TARGET).bin
	$(SIZE) $<

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
