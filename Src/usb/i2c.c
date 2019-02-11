#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc.h"
#include "usbd_cdc_io.h"

#define I2Cx                I2C1

#define CMD_ECHO            0
#define CMD_GET_FUNC        1
#define CMD_SET_DELAY       2
#define CMD_GET_STATUS      3

#define CMD_I2C_IO          4
#define CMD_I2C_IO_BEGIN    (1<<0)
#define CMD_I2C_IO_END      (1<<1)

#define FLAG_TEN            0x0010
#define FLAG_RD             0x0001
#define FLAG_NOSTART        0x4000
#define FLAG_REV_DIR_ADDR   0x2000
#define FLAG_IGNORE_NAK     0x1000
#define FLAG_NO_RD_ACK      0x0800


#define STATUS_IDLE          0
#define STATUS_ADDRESS_ACK   1
#define STATUS_ADDRESS_NAK   2
#define STATUS_BUSY          127

static uint8_t i2c_status = STATUS_IDLE;
static uint32_t i2c_clock_speed = 400 * 1000;

#define FUNC_I2C            0x00000001
#define FUNC_SMBUS_EMUL     0x0EFF0008

I2C_HandleTypeDef i2c_handle;

static const uint32_t i2c_func = (FUNC_I2C | FUNC_SMBUS_EMUL);

static uint32_t i2c_expected_length = 0;
static uint32_t i2c_actual_length = 0;
static uint8_t i2c_addr = 0;
static uint8_t i2c_is_begin = 0;
static uint8_t i2c_is_end = 0;
static uint8_t i2c_read = 0;

// Maximum length of Linux is 8192, but we support 1024 bytes to save RAM
static uint8_t i2c_buffer[1024];

static uint8_t run_send_data = 0;
static uint8_t run_send_status = 0;
static uint8_t run_i2c_init = 1;

static USBD_HandleTypeDef *usbd_handle;

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Add your own code here */
    while(1) {};
}

static uint32_t GetXferOption() {
    if (i2c_is_begin && i2c_is_end)
        return I2C_FIRST_AND_LAST_FRAME;
    else if (i2c_is_begin)
        return I2C_FIRST_FRAME;
    else if (i2c_is_end)
        return I2C_LAST_FRAME;

    return I2C_NEXT_FRAME;
}

uint8_t I2C_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    const uint8_t cmd = req->bRequest;
    const uint16_t value = req->wValue;
    const uint16_t index = req->wIndex;
    const uint16_t length = req->wLength;

    if ((cmd & CMD_I2C_IO) != 0) {
        const uint16_t flags = value;

        i2c_addr = (index << 1);

        i2c_is_begin = cmd & CMD_I2C_IO_BEGIN;
        i2c_is_end = cmd & CMD_I2C_IO_END;

        if (flags & FLAG_RD)
            i2c_read = 1;
        else
            i2c_read = 0;

        i2c_expected_length = length;
        i2c_actual_length =
            sizeof(i2c_buffer) < i2c_expected_length ? sizeof(i2c_buffer) : i2c_expected_length;

        usbd_handle = pdev;

        if (i2c_read) {
            HAL_StatusTypeDef ret = HAL_I2C_Master_Sequential_Receive_IT(
                &i2c_handle, i2c_addr, i2c_buffer, i2c_actual_length, GetXferOption());
            
            if (ret == HAL_OK)
                i2c_status = STATUS_BUSY;
            else
                i2c_status = STATUS_ADDRESS_NAK;
        } else {
            if (length == 0) {
                // 0 bytes write is used to check existence of device
                HAL_StatusTypeDef ret = HAL_I2C_Master_Sequential_Transmit_IT(
                    &i2c_handle, i2c_addr, i2c_buffer, 0, GetXferOption());

                if (ret == HAL_OK)
                    i2c_status = STATUS_BUSY;
                else
                    i2c_status = STATUS_ADDRESS_NAK;
            } else {
                i2c_status = STATUS_BUSY;

                // Receive OUT packets and call Transmit
                USBD_CtlPrepareRx(pdev, i2c_buffer, i2c_actual_length);
            }
        }
    }
    else
    {
        switch (cmd)
        {
            case CMD_ECHO:
                USBD_CtlSendData(pdev, (uint8_t *)&req->wValue, 2);
                break;
            case CMD_GET_FUNC:
                // adapter functionality
                USBD_CtlSendData(pdev, (uint8_t *)&i2c_func, 4);
                break;
            case CMD_SET_DELAY:
                {
                    /* 10us -> 100kHz */
                    const uint32_t delay_us = value;
                    i2c_clock_speed = 1000 * 1000 / delay_us;
                    // maximum clock is 400kHz
                    if (i2c_clock_speed > 400 * 1000)
                        i2c_clock_speed = 400 * 1000;
                    run_i2c_init = 1;
                }
                break;
            case CMD_GET_STATUS:
                run_send_status = 1;
                break;
        }

    }
    return USBD_OK;
}

uint8_t I2C_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    (void)pdev;
    if (i2c_read == 0 && i2c_actual_length > 0) {
        HAL_I2C_Master_Sequential_Transmit_IT(
            &i2c_handle, i2c_addr, i2c_buffer, i2c_actual_length, GetXferOption());
    }
    return USBD_OK;
}

#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2
#define I2C_ADDRESS      0x3C

static void I2C_Init() {
    i2c_handle.Instance             = I2Cx;
    i2c_handle.Init.ClockSpeed      = i2c_clock_speed;
    i2c_handle.Init.DutyCycle       = I2C_DUTYCYCLE;
    i2c_handle.Init.OwnAddress1     = I2C_ADDRESS;
    i2c_handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle.Init.OwnAddress2     = 0xFE;
    i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  

    if(HAL_I2C_Init(&i2c_handle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

static void I2C_DeInit() {
    i2c_handle.Instance             = I2Cx;

    if(HAL_I2C_DeInit(&i2c_handle) != HAL_OK)
        Error_Handler();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c_handle) {
    (void)i2c_handle;
    i2c_status = STATUS_ADDRESS_ACK;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *i2c_handle) {
    (void)i2c_handle;
    i2c_status = STATUS_ADDRESS_ACK;
    run_send_data = 1;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *i2c_handle) {
    (void)i2c_handle;
    i2c_status = STATUS_IDLE;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c_handle) {
    (void)i2c_handle;
    i2c_status = STATUS_ADDRESS_NAK;

    if (i2c_read) {
        for (uint32_t i = 0; i < i2c_actual_length; i++) {
            i2c_buffer[i] = 0;
        }
        run_send_data = 1;
    }
}

void I2C_Run_In_Thread_Mode() {
    if (run_i2c_init == 1) {
        run_i2c_init = 0;
        I2C_DeInit();
        I2C_Init();
    }

    if (run_send_status == 1 && i2c_status != STATUS_BUSY) {
        run_send_status = 0;

        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
        USBD_CtlSendData(usbd_handle, (uint8_t *)&i2c_status, 1);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }

    if (run_send_data == 1) {
        run_send_data = 0;

        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
        USBD_CtlSendData(usbd_handle, i2c_buffer, i2c_actual_length);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}