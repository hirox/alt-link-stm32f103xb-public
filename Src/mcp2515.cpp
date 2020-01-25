#include <cstdint>
#include <type_traits>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_hal_dma.h>
#include <stm32f1xx_hal_spi.h>
#include "usbd_cdc_interface.h"
#include "mcp2515.h"

extern "C" {

SPI_HandleTypeDef SpiHandle;
uint32_t IsMcp2515Exist;
void SPI_Init();

}

MCP mcp;
bool IsSPIBusy = false;

enum class MCP_ADDR : std::uint8_t {
    RXF0SIDH    = 0x00,
    RXF0SIDL    = 0x01,
    RXF0EID8    = 0x02,
    RXF0EID0    = 0x03,
    BFPCTRL     = 0x0C,
    TXRTSCTRL   = 0x0D,
    CANSTAT     = 0x0E,
    CANCTRL     = 0x0F,
    TEC         = 0x1C,
    REC         = 0x1D,
    CNF3        = 0x28,
    CNF2        = 0x29,
    CNF1        = 0x2A,
    CANINTE     = 0x2B,
    CANINTF     = 0x2C,
    EFLG        = 0x2D,

    TXB0CTRL    = 0x30,
    TXB0SIDH    = 0x31,
    TXB0SIDL    = 0x32,
    TXB0EID8    = 0x33,
    TXB0EID0    = 0x34,
    TXB0DLC     = 0x35,
    TXB0D0      = 0x36,

    TXB1CTRL    = 0x40,
    TXB1SIDH    = 0x41,
    TXB1SIDL    = 0x42,
    TXB1EID8    = 0x43,
    TXB1EID0    = 0x44,
    TXB1DLC     = 0x45,
    TXB1D0      = 0x46,

    TXB2CTRL    = 0x50,
    TXB2SIDH    = 0x51,
    TXB2SIDL    = 0x52,
    TXB2EID8    = 0x53,
    TXB2EID0    = 0x54,
    TXB2DLC     = 0x55,
    TXB2D0      = 0x56,

    RXB0CTRL    = 0x60,
    RXB1CTRL    = 0x70,
};

template<typename T>
static std::uint8_t to_uint8(T cmd) {
    return static_cast<typename std::underlying_type<T>::type>(cmd);
}

static void Error_Handler(void)
{
    /* Infinite loop */
    while(1) {};
}

static void SPI_CS(bool select) {
    HAL_GPIO_WritePin(MCP2515_CS_PORT, MCP2515_CS_PIN,
                      select ? GPIO_PIN_RESET : GPIO_PIN_SET);
    IsSPIBusy = select;
}

static void SPI_TX(const std::uint8_t *buffer, std::uint16_t size) {
    if (HAL_SPI_Transmit(&SpiHandle, const_cast<std::uint8_t*>(buffer), size, 10) != HAL_OK)
        Error_Handler();
}

static void SPI_TX_DMA(const std::uint8_t *buffer, std::uint16_t size) {
    SPI_CS(true);
    if (HAL_SPI_Transmit_DMA(&SpiHandle, const_cast<std::uint8_t*>(buffer), size) != HAL_OK)
        Error_Handler();
}

static void SPI_RX(std::uint8_t *buffer, std::uint16_t size) {
    if (HAL_SPI_Receive(&SpiHandle, buffer, size, 10) != HAL_OK)
        Error_Handler();
}

static void SPI_TX_RX_DMA(const std::uint8_t *tx_buffer, std::uint8_t *rx_buffer, std::uint16_t size) {
    if (HAL_SPI_TransmitReceive_IT(&SpiHandle, const_cast<uint8_t*>(tx_buffer), rx_buffer, size) != HAL_OK)
        Error_Handler();
}

static void MCP_Reset() {
    std::uint8_t buffer = to_uint8(MCP_CMD::RESET);

    SPI_CS(true);
    SPI_TX(&buffer, sizeof(buffer));
    SPI_CS(false);
}

static void MCP_Read(MCP_ADDR addr, std::uint32_t size, std::uint8_t *value) {
    std::uint8_t buffer[2];
    
    buffer[0] = to_uint8<MCP_CMD>(MCP_CMD::READ);
    buffer[1] = to_uint8<MCP_ADDR>(addr);

    SPI_CS(true);
    SPI_TX(buffer, sizeof(buffer));
    SPI_RX(value, size);
    SPI_CS(false);
}

#if 0
static void MCP_Write(MCP_ADDR addr, std::uint32_t size, std::uint8_t *value) {
    std::uint8_t buffer[2];
    
    buffer[0] = to_uint8<MCP_CMD>(MCP_CMD::WRITE);
    buffer[1] = to_uint8<MCP_ADDR>(addr);

    SPI_CS(true);
    SPI_TX(buffer, sizeof(buffer));
    SPI_TX(value, size);
    SPI_CS(false);
}
#endif

static void MCP_CheckExistence() {
    std::uint8_t buffer[2];

    MCP_Reset();

    // Check 3 times to wait reset of MCP2515
    for (auto i = 0; i < 3; i++) {
        MCP_Read(MCP_ADDR::CANSTAT, 2, buffer);

        // Check reset value of CANSTAT and CANCTRL
        if ((buffer[0] & 0xEE) == 0x80 &&
            (buffer[1] == 0x87)) {
            IsMcp2515Exist = 1;
            break;
        } else {
            IsMcp2515Exist = 0;
        }
    }
}

void MCP::ControlChannel(SEQ_EVENT ev) {
    if (ev == SEQ_EVENT::OPEN)
        WriteBit_DMA(MCP_ADDR::CANCTRL, 0xE0, 0x00);
    else if (ev == SEQ_EVENT::LISTEN)
        WriteBit_DMA(MCP_ADDR::CANCTRL, 0xE0, 0x60);
    else if (ev == SEQ_EVENT::OPEN_LOOPBACK)
        WriteBit_DMA(MCP_ADDR::CANCTRL, 0xE0, 0x40);
    else if (ev == SEQ_EVENT::CLOSE)
        WriteBit_DMA(MCP_ADDR::CANCTRL, 0xE0, 0x80);
}

static void SPI_CS_INT_Init() {
    SPIx_CS_GPIO_CLK_ENABLE();
    SPIx_INT_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin       = MCP2515_CS_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MCP2515_CS_PORT, &GPIO_InitStruct);

    SPI_CS(false);

    GPIO_InitStruct.Pin       = MCP2515_INT_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(MCP2515_INT_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void SPI_Init() {
    SPI_CS_INT_Init();

    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;

    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
        Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
    __HAL_SPI_ENABLE(&SpiHandle);

    MCP_CheckExistence();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // MCP2515 Sent INT
    if (GPIO_Pin == MCP2515_INT_PIN)
        mcp.Interrupt();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    SPI_CS(false);
}

static constexpr std::uint32_t OSC_HZ = 8 * 1000 * 1000;

bool MCP::SetBitrate(std::uint32_t bitrate) {
    if (seq_event != SEQ_EVENT::NONE)
        return false;

    const std::uint32_t tqbrp = OSC_HZ / 2 / bitrate;
    const std::uint32_t min_tq = 5;
    const std::uint32_t max_tq = 25;
    std::uint32_t brp = (tqbrp + max_tq - 1) / max_tq;

    // min brp is 1
    if (brp == 0)
        brp = 1;

    std::uint32_t tq = (tqbrp - 1) / brp + 1;

    if (tq > max_tq)
        tq = max_tq;

    if (tq < min_tq)
        tq = min_tq;

    std::uint32_t best_brp = brp;
    std::uint32_t best_tq = tq;

    if (tq * brp != tqbrp) {
        // search better brp and tq
        std::uint32_t diff = tq * brp - tqbrp;
        for (std::uint32_t i = 1; i < 10; i++) {
            auto tmpbrp = brp + i;
            auto tmptq = (tqbrp - 1) / tmpbrp + 1;
            if (tmptq < min_tq || tmpbrp > 64)
                break;
            auto tmpdiff = tmpbrp * tmptq - tqbrp;
            if (tmpdiff < diff) {
                best_brp = tmpbrp;
                best_tq = tmptq;
                diff = tmpdiff;
            }
            if (diff == 0)
                break;
        }
    }

    const std::uint32_t sjw = 1;  // usually 1 is enough
    const std::uint32_t sync = 1; // fixed
    const std::uint32_t min_prop = 1;
    std::uint32_t ps2 = (best_tq - sync - min_prop + 1) / 2;
    if (ps2 > 8)
        ps2 = 8;
    if (ps2 < 2)
        ps2 = 2;
    
    std::uint32_t ps1 = (best_tq - sync - ps2 + 1) / 2;
    if (ps1 > 8)
        ps1 = 8;
    if (ps1 < 1)
        ps1 = 1;

    std::uint32_t prop = best_tq - sync - ps1 - ps2;
    if (prop > 8)
        prop = 8;
    if (prop < 1)
        prop = 1;

    bitrate_.cmd = MCP_CMD::WRITE;
    bitrate_.addr = to_uint8(MCP_ADDR::CNF3);
    bitrate_.cnf1.SJW10 = sjw;
    bitrate_.cnf1.BRP5to0 = best_brp;
    bitrate_.cnf2.BTLMODE = 1;
    bitrate_.cnf2.SAM = 1;
    bitrate_.cnf2.PHSEG1 = ps1;
    bitrate_.cnf2.PRSEG = prop;
    bitrate_.cnf3.SOF = 0;
    bitrate_.cnf3.WAKFIL = 0;
    bitrate_.cnf3.PHSEG2 = ps2;
    seq_event = SEQ_EVENT::SETBITRATE;
    return true;
}

void MCP::ReadEFLG_DMA() {
    std::uint32_t size = 0;
    read_write_buffer_[0] = to_uint8<MCP_CMD>(MCP_CMD::READ);   size++;
    read_write_buffer_[1] = to_uint8<MCP_ADDR>(MCP_ADDR::EFLG); size++;
    read_write_buffer_[2] = 0;                                  size++;

    SPI_CS(true);
    SPI_TX_RX_DMA(read_write_buffer_, read_write_buffer_, size);
}

void MCP::ReadStatus_DMA() {
    SPI_CS(true);
    mcp_status.cmd = static_cast<std::uint8_t>(MCP_CMD::STATUS);
    SPI_TX_RX_DMA(mcp_status.raw, mcp_status.raw, sizeof(mcp_status));
}

void MCP::ReadRXStatus_DMA() {
    SPI_CS(true);
    rx_status.cmd = static_cast<std::uint8_t>(MCP_CMD::RX_STATUS);
    SPI_TX_RX_DMA(rx_status.raw, rx_status.raw, sizeof(rx_status));
}

void MCP::ReadBuffer_DMA(std::uint32_t index) {
    SPI_CS(true);
    if (index > 1)
        index = 1;
    rx_data[index].cmd = static_cast<std::uint8_t>(MCP_CMD::RX_READ) + (index == 0U ? 0x00 : 0x04);
    SPI_TX_RX_DMA(rx_data[index].raw, rx_data[index].raw, sizeof(RX));
    // [NOTE] RXnIF will be cleard when !CS will be high
}

bool MCP::CanReadTxBuffer() {
    if (tx_write_index - tx_read_index > 0)
        return true;
    else
        return false;
}

void MCP::WriteBuffer_DMA(std::uint32_t index) {
    if (!CanReadTxBuffer())
        return;

    auto tx_index = (tx_read_index & 0x03);
    tx_data[tx_index].cmd = static_cast<std::uint8_t>(MCP_CMD::TX_WRITE) + ((1 << index) >> 1);
    SPI_TX_DMA(tx_data[tx_index].raw, sizeof(TX));

    if (tx_data[tx_index].sidl.EXIDE)
        notification_.TransmitAck29();
    else
        notification_.TransmitAck11();

    tx_read_index++;
}

void MCP::WriteByte_DMA(MCP_ADDR addr, std::uint8_t data) {
    std::uint32_t size = 0;
    read_write_buffer_[0] = to_uint8<MCP_CMD>(MCP_CMD::WRITE);  size++;
    read_write_buffer_[1] = to_uint8<MCP_ADDR>(addr);           size++;
    read_write_buffer_[3] = data;                               size++;

    SPI_TX_DMA(read_write_buffer_, size);
}

void MCP::WriteBit_DMA(MCP_ADDR addr, std::uint8_t mask, std::uint8_t data) {
    std::uint32_t size = 0;
    read_write_buffer_[0] = to_uint8<MCP_CMD>(MCP_CMD::WRITE_BIT);  size++;
    read_write_buffer_[1] = to_uint8<MCP_ADDR>(addr);               size++;
    read_write_buffer_[2] = mask;                                   size++;
    read_write_buffer_[3] = data;                                   size++;

    SPI_TX_DMA(read_write_buffer_, size);
}

void MCP::RTS_DMA() {
    static std::uint8_t rts_msg;
    rts_msg = 0x80 | (rts_.raw & 0x7);

    SPI_TX_DMA(&rts_msg, 1);

    rts_.raw = 0x80;
}

void ToHex(std::uint8_t data, std::uint8_t* buffer) {
    auto val = (data & 0x0F);
    if (val >= 10)
        *buffer = ((val - 10) + '0');
    else
        *buffer = (val + '0');
}

void ToHex(std::uint8_t data, std::uint8_t* buffer1, std::uint8_t* buffer2) {
    ToHex(data >> 4, buffer1);
    ToHex(data, buffer2);
}

void MCP::ComposeForSLCAN(std::uint32_t index, std::uint8_t* buffer, std::size_t* size) {
    if (index > 1)
        index = 1;

    const RX& rx = rx_data[index];
    bool ext = rx.sidl.IDE ? true : false;
    std::uint32_t id = (rx.sidh.SID10to3 << 3) | rx.sidl.SID210;
    std::uint32_t received = rx.dlc.DLC;
    std::uint8_t* p = buffer;
    bool rtr = false;

    if (ext) {
        rtr = rx.dlc.RTR ? true : false;
        if (rtr) {
            *p = 'R'; p++;
        } else {
            *p = 'T'; p++;
        }

        id |= (rx.sidl.EID1716 << (11 + 16));
        id |= (rx.eid8.EID15to8 << (11 + 8));
        id |= (rx.eid8.EID15to8 << 11);
        ToHex((id >> 24) & 0xFF, p, p + 1); p += 2;
        ToHex((id >> 16) & 0xFF, p, p + 1); p += 2;
        ToHex((id >> 8) & 0xFF, p, p + 1); p += 2;
        ToHex(id & 0xFF, p, p + 1); p += 2;
        ToHex(received, p); p++;
    } else {
        rtr = rx.sidl.SRR ? true : false;
        if (rtr) {
            *p = 'r'; p++;
        } else {
            *p = 't'; p++;
        }

        ToHex(id >> 8, p); p++;
        ToHex(id & 0xFF, p, p + 1); p += 2;
        ToHex(received, p); p++;
    }

    if (!rtr) {
        for (auto i = 0U; i < received; i++) {
            ToHex(rx.data[i], p, p + 1); p += 2;
        }
    }
    if (timestamp_) {
        auto t = SysTick->VAL % 60000;  // 60sec
        ToHex(t >> 8, p, p + 1); p += 2;
        ToHex(t & 0xFF, p, p + 1); p += 2;
    }

    // [TODO] Support loopback flag

    *p = '\r'; p++;
    *size = p - buffer;
}

void MCP::Run_In_Thread_Mode() {
    if (IsSPIBusy)
        return;

    if (event_.raw) {
        std::uint8_t buffer[64];
        std::size_t size;
        if (event_.READ_RX0) {
            event_.READ_RX0 = 0;
            ComposeForSLCAN(0, buffer, &size);
            notification_.Write(buffer, size);
        }

        if (event_.READ_RX1) {
            event_.READ_RX1 = 0;
            ComposeForSLCAN(1, buffer, &size);
            notification_.Write(buffer, size);
        }
    }

    if (seq_event != SEQ_EVENT::NONE) {
        auto ev = seq_event;
        seq_event = SEQ_EVENT::NONE;
        switch (ev) {
            case SEQ_EVENT::NONE:
                break;
            case SEQ_EVENT::SETBITRATE:
                SPI_TX_DMA(bitrate_.raw, sizeof(bitrate_));
                // [TODO] check status
                notification_.Ack();
                return;
            case SEQ_EVENT::OPEN:
            case SEQ_EVENT::LISTEN:
            case SEQ_EVENT::OPEN_LOOPBACK:
            case SEQ_EVENT::CLOSE:
                ControlChannel(ev);
                // [TODO] send nack when status is error
                notification_.Ack();
                break;
            case SEQ_EVENT::VERSION:
                notification_.Version();
                break;
            case SEQ_EVENT::MINOR:
                notification_.Minor();
                break;
            case SEQ_EVENT::SERIAL:
                notification_.Serial();
                break;
            case SEQ_EVENT::TIMESTAMP_ON:
                timestamp_ = true;
                notification_.Ack();
                break;
            case SEQ_EVENT::TIMESTAMP_OFF:
                timestamp_ = false;
                notification_.Ack();
                break;
            case SEQ_EVENT::FLAGS:
                ReadEFLG_DMA();
                seq_event = SEQ_EVENT::FLAGS_STEP2;
                return;
            case SEQ_EVENT::FLAGS_STEP2:
                {
                    EFLG ef_mcp;
                    ef_mcp.raw = read_write_buffer_[2];

                    std::uint8_t ef_sl = 0;
                    if (ef_mcp.RX1OVR || ef_mcp.RX0OVR)
                        ef_sl &= 0x08;

                    if (ef_mcp.TXEP || ef_mcp.RXEP)
                        ef_sl &= 0x20;

                    if (ef_mcp.TXBO)
                        ef_sl &= 0x80;

                    std::uint8_t buffer[3];
                    buffer[0] = 'F';
                    buffer[1] = ef_sl;
                    buffer[2] = '\r';
                    notification_.Write(buffer, sizeof(buffer));
                }
                break;
            case SEQ_EVENT::INIT:
                // Enter configuration mode
                ControlChannel(SEQ_EVENT::CLOSE);
                seq_event = SEQ_EVENT::INIT_STEP2;
                return;
            case SEQ_EVENT::INIT_STEP2:
                {
                    // Enable RX0/RX1 interrupts
                    CANINTE data = {};
                    data.RX1IE = 1;
                    data.RX1IE = 0;
                    WriteByte_DMA(MCP_ADDR::CANINTE, data.raw);
                    seq_event = SEQ_EVENT::INIT_STEP3;
                }
                return;
            case SEQ_EVENT::INIT_STEP3:
                {
                    // Use RXB1 when RXB0 is full
                    RXB0CTRL data = {};
                    data.BUKT = 1;
                    WriteByte_DMA(MCP_ADDR::RXB0CTRL, data.raw);
                }
                return;
            case SEQ_EVENT::UNKNOWN:
                notification_.NAck();
                break;
        }
    }

    if (mcp_status.data.RX1IF) {
        mcp_status.data.RX1IF = 0;
        ReadBuffer_DMA(1U);
        event_.READ_RX1 = 1;
    } else if (mcp_status.data.RX0IF) {
        mcp_status.data.RX0IF = 0;
        ReadBuffer_DMA(0U);
        event_.READ_RX0 = 1;
    } else if (event_.READ_STATUS) {
        event_.READ_STATUS = 0;
        ReadStatus_DMA();
    } else if (CanReadTxBuffer()) {
        if (mcp_status.data.TX2REQ == 1) {
            if (mcp_status.data.TX1REQ == 1) {
                if (mcp_status.data.TX0REQ == 0) {
                    // write to tx0 when tx2 and tx1 are full
                    mcp_status.data.TX0REQ = 1;
                    WriteBuffer_DMA(0);
                    rts_.TXB0 = 1;
                }
            } else if (mcp_status.data.TX0REQ == 0) {
                // write to tx1 when only tx2 is full
                mcp_status.data.TX1REQ = 1;
                WriteBuffer_DMA(1);
                rts_.TXB1 = 1;
            }
        } else if (mcp_status.data.TX1REQ == 0 && mcp_status.data.TX0REQ == 0) {
            // write to tx2 when tx2,1,0 are empty
            mcp_status.data.TX2REQ = 1;
            WriteBuffer_DMA(2);
            rts_.TXB2 = 1;
        }
    } else if (rts_.raw) {
        RTS_DMA();
    }
}

void MCP::Initialize() {
    seq_event = SEQ_EVENT::INIT;
}

bool MCP::SetEvent(SEQ_EVENT event) {
    if (seq_event == SEQ_EVENT::NONE) {
        seq_event = event;
        return true;
    }
    return false;
}

bool MCP::CanTransmit() {
    if (tx_write_index - tx_read_index < 3)
        return true;
    else
        return false;
}

bool MCP::Transmit(bool ext, bool rtr, std::uint32_t id, std::uint8_t *buffer, std::uint32_t size) {
    if (!CanTransmit())
        return false;

    if (size > 8)
        return false;

    auto &tx = tx_data[tx_write_index & 0x03];
    tx.sidh.SID10to3 = (id >> 3) & 0xFF;
    tx.sidl.SID210 = (id & 0x07);
    if (ext) {
        tx.sidl.EXIDE = 1;
        tx.sidl.EID1716 = (id >> (11 + 16)) & 0x03;
        tx.eid8.EID15to8 = (id >> (11 + 8)) & 0xFF;
        tx.eid0.EID7to0  = (id >> 11) & 0xFF;
    } else {
        tx.sidl.EXIDE = 0;
    }

    if (rtr) {
        tx.dlc.RTR = 1;
    } else {
        tx.dlc.RTR = 0;
    }

    tx.dlc.DLC = size;
    if (!rtr)
        memcpy(tx.data, buffer, size);

    tx_write_index++;

    return true;
}