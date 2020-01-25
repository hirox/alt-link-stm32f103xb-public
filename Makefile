include mk/toolchain.mk

TARGET_NAME     ?= stm32f1
OUTPUT_PATH      = ./build
GDB_ADDR        ?= 192.168.0.7:3333
STM32SDK         = STM32Cube_FW_F1_V1.4.0

DAPLINK          = ./Src/DAPLink/source
CMSISDAP         = $(DAPLINK)/daplink/cmsis-dap

SRCS             = Src/main.c Src/stm32f1xx_it.c Src/stm32f1xx_hal_msp.c Src/system_stm32f1xx.c Src/usbd_conf.c Src/usbd_desc.c
SRCS            += Src/usb/usbd.c Src/usb/hid/usbd_hid.c Src/usb/hid/usbd_hid_if.c Src/stm32f1xx_hal_pcd.c
SRCS            += Src/usb/cdc/usbd_cdc_interface.c Src/usb/cdc/usbd_cdc.c Src/usb/i2c.c Src/usbd_ctlreq.c
SRCS            += Src/mcp2515.cpp
SRCS            += Src/usb/cdc/slcan.cpp
SRCS            += $(CMSISDAP)/DAP.c $(CMSISDAP)/JTAG_DP.c $(CMSISDAP)/SW_DP.c

SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c

SRCS            += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
SRCS            += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
SRCS            += $(DAPLINK)/hic_hal/stm32/stm32f103xb/usb_config.c

# Separate .s files to link weak symbols first
SRCS_AS          = $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

LINK_SCRIPT      = $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld

ELF              = $(OUTPUT_PATH)/$(TARGET_NAME).elf
FW_BIN           = $(OUTPUT_PATH)/$(TARGET_NAME).bin
MAP              = $(OUTPUT_PATH)/$(TARGET_NAME).map

OBJS             = $(addprefix $(OUTPUT_PATH)/, $(SRCS:%=%.o))
OBJS_AS          = $(addprefix $(OUTPUT_PATH)/, $(SRCS_AS:%=%.o))
DEPS             = $(addprefix $(OUTPUT_PATH)/, $(SRCS:%=%.d))
DEPS_AS          = $(addprefix $(OUTPUT_PATH)/, $(SRCS_AS:%=%.d))

INC_PATH         = ./
INC_PATH         = ./Inc
INC_PATH        += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
INC_PATH        += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Inc
INC_PATH        += $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Include
INC_PATH        += $(STM32SDK)/Drivers/CMSIS/Include
INC_PATH        += $(DAPLINK)/usb $(DAPLINK)/rtos

CPUFLAGS         = -mthumb -mcpu=cortex-m3

COMMON_FLAGS     = -Wall -Wextra
COMMON_FLAGS    += -DSTM32F103xB -DHAL_TIM_MODULE_ENABLED
COMMON_FLAGS    += -Wno-attributes
COMMON_FLAGS    += -MD -g -Os -ffunction-sections -fdata-sections -fshort-wchar
COMMON_FLAGS    += -flto
COMMON_FLAGS    += -c $(CPUFLAGS)
COMMON_FLAGS    += $(addprefix -I,$(INC_PATH))

CFLAGS           = $(COMMON_FLAGS)
CFLAGS          += -std=gnu99

CXXFLAGS         = $(COMMON_FLAGS)
CXXFLAGS        += -std=c++14

LDFLAGS          = -ggdb -Xlinker -Map=$(MAP) -T $(LINK_SCRIPT)
LDFLAGS         += $(CPUFLAGS)
#LDFLAGS         += --specs=nano.specs
#LDFLAGS         += --specs=nosys.specs
LDFLAGS         += -Wl,--no-wchar-size-warning
LDFLAGS         += -Wl,--gc-sections

all: $(FW_BIN) size

rebuild:
	make clean
	make all

$(ELF): $(OBJS) $(OBJS_AS)
	@echo "LD            $<"
	@$(LD) $(LDFLAGS) $(OBJS_AS) $(OBJS) -o $(ELF)

$(FW_BIN): $(ELF)
	@echo "OBJCOPY (bin) $@"
	#@$(OBJCOPY) -j .text -j .data -j .ARM.extab -j .ARM.exidx -O binary $(ELF) $(FW_BIN)
	@$(OBJCOPY) -O binary $(ELF) $(FW_BIN)

$(OUTPUT_PATH)/%.c.o: %.c Makefile
	@[ -d $(dir $@) ] || mkdir -p $(dir $@)
	@echo "CC            $<"
	@$(CC) $(CFLAGS) $< -o $@

$(OUTPUT_PATH)/%.cpp.o: %.cpp Makefile
	@[ -d $(dir $@) ] || mkdir -p $(dir $@)
	@echo "CXX            $<"
	@$(CXX) $(CXXFLAGS) $< -o $@

$(OUTPUT_PATH)/%.s.o: %.s Makefile
	@[ -d $(dir $@) ] || mkdir -p $(dir $@)
	@echo "AS            $<"
	@$(AS) $< -o $@

size: $(FLASH_IMAGE)
	@echo "SIZE          $(ELF)"
	@$(SIZE) $(ELF)

clean:
	$(RM) -fr $(OUTPUT_PATH)

clean-full: clean
	$(RM) -fr GPATH GRTAGS GTAGS

-include $(DEPS) $(DEPS_AS)


.PHONY: all clean clean-full
