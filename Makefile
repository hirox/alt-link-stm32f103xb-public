include mk/toolchain.mk

TARGET_NAME     ?= stm32f1
OUTPUT_PATH      = ./build
GDB_ADDR        ?= 192.168.0.7:3333
STM32SDK         = STM32Cube_FW_F1_V1.4.0

DAPLINK          = ./Src/DAPLink/source
CMSISDAP         = $(DAPLINK)/daplink/cmsis-dap

SRCS             = Src/main.c Src/stm32f1xx_it.c Src/stm32f1xx_hal_msp.c Src/system_stm32f1xx.c Src/usbd_conf.c Src/usbd_desc.c
SRCS            += Src/usb/hid/usbd_hid.c Src/usb/hid/usbd_hid_if.c
SRCS            += Src/usb/cdc/usbd_cdc_interface.c Src/usb/cdc/usbd_cdc.c Src/usbd_ctlreq.c Src/stm32f1xx_hal_uart.c
SRCS            += $(CMSISDAP)/DAP.c $(CMSISDAP)/JTAG_DP.c $(CMSISDAP)/SW_DP.c

SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
SRCS            += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
SRCS            += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
SRCS            += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
SRCS            += $(DAPLINK)/hic_hal/stm32/stm32f103xb/usb_config.c

SRCS_AS          = $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

LINK_SCRIPT      = $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld

ELF              = $(OUTPUT_PATH)/$(TARGET_NAME).elf
FW_BIN           = $(OUTPUT_PATH)/$(TARGET_NAME).bin
MAP              = $(OUTPUT_PATH)/$(TARGET_NAME).map

OBJS             = $(addprefix $(OUTPUT_PATH)/, $(SRCS:.c=.o))
OBJS_AS          = $(addprefix $(OUTPUT_PATH)/, $(SRCS_AS:.s=.o))
DEPS             = $(addprefix $(OUTPUT_PATH)/, $(SRCS:.c=.d))
DEPS_AS          = $(addprefix $(OUTPUT_PATH)/, $(SRCS_AS:.s=.d))

INC_PATH         = ./
INC_PATH         = ./Inc
INC_PATH        += $(STM32SDK)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
INC_PATH        += $(STM32SDK)/Drivers/STM32F1xx_HAL_Driver/Inc
INC_PATH        += $(STM32SDK)/Drivers/CMSIS/Device/ST/STM32F1xx/Include
INC_PATH        += $(STM32SDK)/Drivers/CMSIS/Include
INC_PATH        += $(DAPLINK)/usb $(DAPLINK)/rtos

CPUFLAGS         = -mthumb -mcpu=cortex-m3

CFLAGS           = -Wall -Wextra
CFLAGS          += -DSTM32F103xB -DHAL_TIM_MODULE_ENABLED
CFLAGS          += -Wno-attributes
CFLAGS          += -MD -g -Os -ffunction-sections -fdata-sections -fshort-wchar
CFLAGS          += -flto
CFLAGS          += -c $(CPUFLAGS)
CFLAGS          += -std=gnu99
CFLAGS          += -ffunction-sections -fdata-sections
CFLAGS          += $(addprefix -I,$(INC_PATH))

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

$(ELF): $(OBJS) $(OBJS_CPP) $(OBJS_AS)
	@echo "LD            $<"
	@$(LD) $(LDFLAGS) $(OBJS_AS) $(OBJS_CPP) $(OBJS) -o $(ELF)

$(FW_BIN): $(ELF)
	@echo "OBJCOPY (bin) $@"
	#@$(OBJCOPY) -j .text -j .data -j .ARM.extab -j .ARM.exidx -O binary $(ELF) $(FW_BIN)
	@$(OBJCOPY) -O binary $(ELF) $(FW_BIN)

$(OUTPUT_PATH)/%.o: %.c Makefile
	@[ -d $(dir $@) ] || mkdir -p $(dir $@)
	@echo "CC            $<"
	@$(CC) $(CFLAGS) $< -o $@

$(OUTPUT_PATH)/%.o: %.s Makefile
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
