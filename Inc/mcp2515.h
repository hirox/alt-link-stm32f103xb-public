#pragma once
#pragma pack(1)

#include <cstdint>
#include <type_traits>
#include "notification_buffer.h"
#include "mcp2515_registers.h"

// MCP2515 SPI instruction set
enum class MCP_CMD : std::uint8_t {
    RESET       = 0xC0,
    READ        = 0x03,
    RX_READ     = 0x90, // 0b0nm0
    WRITE       = 0x02,
    TX_WRITE    = 0x40, // 0b0abc
    RTS         = 0x80, // 0b0nnn
    STATUS      = 0xA0,
    RX_STATUS   = 0xB0,
    WRITE_BIT   = 0x05
};

enum class SEQ_EVENT : std::uint8_t {
    NONE            = 0x00,
    SETBITRATE      = 0x01,
    OPEN            = 0x02,
    LISTEN          = 0x03,
    OPEN_LOOPBACK   = 0x04,
    CLOSE           = 0x05,

    VERSION         = 0x06,
    MINOR           = 0x07,
    SERIAL          = 0x08,

    TIMESTAMP_ON    = 0x09,
    TIMESTAMP_OFF   = 0x0A,

    FLAGS           = 0x0B,
    FLAGS_STEP2     = 0x0C,

    INIT            = 0xF0,
    INIT_STEP2      = 0xF1,
    INIT_STEP3      = 0xF2,
    UNKNOWN         = 0xFF
};

enum class MCP_ADDR : std::uint8_t;

class MCP {
    bool timestamp_ = false;
    SPIRTS rts_ = {};
    NotificationBuffer notification_;

    union {
        struct {
            std::uint8_t cmd;
            SPISTAT data;
        };
        std::uint8_t raw[sizeof(SPISTAT) + 1];
     } mcp_status = {};
    static_assert(sizeof(mcp_status) == sizeof(SPISTAT) + 1);

    void ReadStatus_DMA();

    union {
        struct {
            std::uint8_t cmd;
            SPIRXSTAT data;
        };
        std::uint8_t raw[sizeof(SPIRXSTAT) + 1];
     } rx_status;
    static_assert(sizeof(rx_status) == sizeof(SPIRXSTAT) + 1);

    void ReadRXStatus_DMA();

    void ReadEFLG_DMA();

    union RX {
        struct {
            std::uint8_t cmd;
            RXBnSIDH sidh;
            RXBnSIDL sidl;
            RXBnEID8 eid8;
            RXBnEID0 eid0;
            RXBnDLC  dlc;
            std::uint8_t data[8];
        };
        std::uint8_t raw[29];
    };
    static_assert(sizeof(RX) == 29);

    union TX {
        struct {
            std::uint8_t cmd;
            TXBnSIDH sidh;
            TXBnSIDL sidl;
            TXBnEID8 eid8;
            TXBnEID0 eid0;
            TXBnDLC  dlc;
            std::uint8_t data[8];
        };
        std::uint8_t raw[29];
    };
    static_assert(sizeof(TX) == 29);

    RX rx_data[2];
    TX tx_data[3];
    std::uint32_t tx_write_index = 0;
    std::uint32_t tx_read_index = 0;

    std::uint8_t read_write_buffer_[4];

    void ControlChannel(SEQ_EVENT ev);

    void Write_DMA(std::uint8_t* buffer, std::uint16_t size);
    void ReadBuffer_DMA(std::uint32_t index);
    void WriteBuffer_DMA(std::uint32_t index);
    void WriteByte_DMA(MCP_ADDR addr, std::uint8_t data);
    void WriteBit_DMA(MCP_ADDR addr, std::uint8_t mask, std::uint8_t data);

    void RTS_DMA();

    bool CanReadTxBuffer();

    union EVENT {
        struct {
            std::uint32_t READ_STATUS   : 1;
            std::uint32_t READ_RX0      : 1;
            std::uint32_t READ_RX1      : 1;
        };
        std::uint8_t raw;
    } event_;
    static_assert(CONFIRM_UINT8(EVENT));

    SEQ_EVENT seq_event = SEQ_EVENT::NONE;

    union {
        struct {
            MCP_CMD cmd;
            std::uint8_t addr;
            CNF3 cnf3;
            CNF2 cnf2;
            CNF1 cnf1;
        };
        std::uint8_t raw[5];
    } bitrate_;
    static_assert(sizeof(bitrate_) == 5);

    void ComposeForSLCAN(std::uint32_t index, std::uint8_t* buffer, std::size_t* size);

  public:
    void Initialize();
    bool SetBitrate(std::uint32_t bitrate);
    bool SetEvent(SEQ_EVENT event);
    bool Transmit(bool ext, bool rtr, std::uint32_t id, std::uint8_t *buffer, std::uint32_t size);
    bool CanTransmit();

    void Interrupt() {
        event_.READ_STATUS = 1;
    }

    void SetNotificationBuffer(std::uint8_t* buffer, std::uint32_t size, std::uint32_t write_pos = 0U) {
        notification_.Set(buffer, size, write_pos);
    }

    std::uint32_t GetNotificationWritePos() {
        return notification_.GetWritePos();
    }

    void Run_In_Thread_Mode();
};