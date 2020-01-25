#include "usbd_cdc.h"
#include "mcp2515.h"
#include "stm32f1xx_hal_cortex.h"

extern CDC_WorkMemory CDC;
extern MCP mcp;

class SLCan {
  private:
    std::size_t overwrite_size_;

    char GetChar(std::size_t read_idx) {
        const uint32_t i = SLCAN_INDEX;

        std::uint8_t data;
        if (read_idx < APP_RX_DATA_SIZE + overwrite_size_) {
            data = CDC.d[i].UsbOutBuffer[read_idx];
        } else {
            data = CDC.d[i].UsbOutBuffer[read_idx & (APP_RX_DATA_SIZE - 1)];
        }
        return static_cast<char>(data);
    }

    void GetChars(char buf[], std::size_t size, std::size_t read_idx) {
        for (std::uint32_t i = 0; i < size; i++)
            buf[i] = GetChar(read_idx + i);
    }

    std::uint8_t HexToInt(const char c) {
        if ('0' <= c && c <= '9') {
            return static_cast<std::uint32_t>(c - '0');
        } else if ('A' <= c && c <= 'F') {
            return static_cast<std::uint32_t>(c - 'A' + 10);
        } else if ('a' <= c && c <= 'f') {
            return static_cast<std::uint32_t>(c - 'A' + 10);
        } else {
            return 0;
        }
    }

    std::uint8_t GetBin(std::size_t read_idx) {
        auto char1 = GetChar(read_idx);
        auto char2 = GetChar(read_idx + 1);
        return static_cast<std::uint8_t>((HexToInt(char1) << 4) + HexToInt(char2));
    }

    void GetBins(std::uint8_t buf[], std::size_t bytes, std::size_t read_idx) {
        for (uint32_t i = 0; i < bytes; i++)
            buf[i] = GetBin(read_idx + i * 2);
    }

    std::uint32_t IdToInt(const char id[], std::uint32_t length) {
        std::uint32_t int_id = 0;
        for (std::uint32_t i = 0; i < length; i++) {
            int_id = (int_id << 4) + HexToInt(id[i]);
        }
        return int_id;
    }

    std::uint32_t GetStdId(std::size_t read_idx) {
        char id[3];
        GetChars(id, 3, read_idx);
        return IdToInt(id, 3);
    }

    std::uint32_t GetExtId(std::size_t read_idx) {
        char id[8];
        GetChars(id, 8, read_idx);
        return IdToInt(id, 8);
    }

    std::uint32_t GetLen(std::size_t read_idx) {
        char len_hex = GetChar(read_idx);
        return HexToInt(len_hex);
    }

    void HexToBuf(const char* hex, std::uint8_t* buf, std::size_t buf_bytes) {
        for (std::uint32_t i = 0; i < buf_bytes; i++) {
            buf[i] = (HexToInt(hex[i * 2]) << 8) + HexToInt(hex[i * 2 + 1]);
        }
    }

  public:
    bool ParseCommand(std::size_t read_idx/*, std::uint32_t size*/) {
        char command = GetChar(read_idx);

        switch(command) {
            case 'S':
                {
                    char bitrate_hex = GetChar(read_idx + 1);
                    std::uint32_t bitrate = 100 * 1000; // 100Kbps
                    switch (bitrate_hex) {
                        case '0': bitrate =   10 * 1000; break;
                        case '1': bitrate =   20 * 1000; break;
                        case '2': bitrate =   50 * 1000; break;
                        case '3': bitrate =  100 * 1000; break;
                        case '4': bitrate =  125 * 1000; break;
                        case '5': bitrate =  250 * 1000; break;
                        case '6': bitrate =  500 * 1000; break;
                        case '7': bitrate =  800 * 1000; break;
                        case '8': bitrate = 1000 * 1000; break;
                    }
                    return mcp.SetBitrate(bitrate);
                }
            case 'O': // Open
                return mcp.SetEvent(SEQ_EVENT::OPEN);
            case 'L': // Listen (silent)
                return mcp.SetEvent(SEQ_EVENT::LISTEN);
            case 'l': // Open + Loopback
                return mcp.SetEvent(SEQ_EVENT::OPEN_LOOPBACK);
            case 'C': // Close
                return mcp.SetEvent(SEQ_EVENT::CLOSE);
            case 'M': // Acceptance mask
                // Not supported
                return mcp.SetEvent(SEQ_EVENT::UNKNOWN);
            case 'm': // Acceptance value
                // Not supported
                return mcp.SetEvent(SEQ_EVENT::UNKNOWN);
            case 't': // 11bit ID frame
            case 'T': // 29bit ID frame
            case 'r': // 11bit ID RTR frame
            case 'R': // 29bit ID RTR frame
                if (mcp.CanTransmit()) {
                    bool rtr = (command == 'r' || command == 'R');
                    bool ext = (command == 'T' || command == 'R');
                    std::uint32_t id;
                    if (ext)
                        id = GetStdId(read_idx + 1);
                    else
                        id = GetExtId(read_idx + 1);

                    std::size_t len_offset = 1 + (ext ? 8 : 3);
                    std::uint32_t len = GetLen(read_idx + len_offset);
                    if (len > 8)
                        len = 8;

                    if (!rtr) {
                        std::uint8_t buf[8];
                        GetBins(buf, len, read_idx + 5);
                        return mcp.Transmit(ext, rtr, id, buf, len);
                    } else {
                        return mcp.Transmit(ext, rtr, id, nullptr, len);
                    }
                } else {
                    return false;
                }
            case 'Z': // timestamp
                if (GetChar(read_idx + 1) == '1')
                    return mcp.SetEvent(SEQ_EVENT::TIMESTAMP_ON);
                else
                    return mcp.SetEvent(SEQ_EVENT::TIMESTAMP_OFF);
            case 'F': // Read FIFO information
                return mcp.SetEvent(SEQ_EVENT::FLAGS);
            case 'V': // Version
                return mcp.SetEvent(SEQ_EVENT::VERSION);
            case 'v': // Minor version
                return mcp.SetEvent(SEQ_EVENT::MINOR);
            case 'N': // Serial number
                return mcp.SetEvent(SEQ_EVENT::SERIAL);
            default:
                return mcp.SetEvent(SEQ_EVENT::UNKNOWN);
        }
    }

    std::int32_t Parse(std::size_t read_idx, std::size_t overwrite_size, std::uint32_t size) {
        overwrite_size_ = overwrite_size;

        std::int32_t last_pos = -1;
        for (auto i = 0U; i < size; i++) {
            // Parse data when delimiter('\r') was found.
            if (GetChar(read_idx + i) == '\r') {
                if (ParseCommand(last_pos + 1/*, i - last_pos - 1*/) == false)
                    break;
                last_pos = i;
            }
        }
        return last_pos + 1;
    }
};

SLCan slcan;

extern "C" void SLCAN_Init();
extern "C" void SLCAN_Run_In_Thread_Mode();
extern "C" uint32_t SLCAN_GetWritePos();

void SLCAN_Init() {
    const std::uint32_t i = SLCAN_INDEX;

    mcp.Initialize();
    mcp.SetNotificationBuffer(CDC.d[i].UserTxBuffer, sizeof(CDC.d[i].UserTxBuffer));
}

static void SLCAN_Parse() {
    const std::uint32_t i = SLCAN_INDEX;

    if (CDC.d[i].UsbOutBufReadPos == CDC.d[i].UsbOutBufWritePos)
        return;

    std::uint32_t read_idx = CDC.d[i].UsbOutBufReadPos & (APP_RX_DATA_SIZE - 1);
    std::uint32_t buffsize;
    std::size_t overwritesize;

    // Disable USB OUT interrupt to get consistent value
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    buffsize = CDC.d[i].UsbOutBufWritePos - CDC.d[i].UsbOutBufReadPos;
    overwritesize = CDC.d[i].UsbOutOverWriteSize;
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // When previous read position was in overwrite area, add APP_RX_DATA_SIZE.
    if (CDC.d[i].UsbOutBufReadPosInOverwrite)
        read_idx += APP_RX_DATA_SIZE;

    auto parsed_size = slcan.Parse(read_idx, overwritesize, buffsize);

    if (parsed_size > 0) {
        if (read_idx + parsed_size >= APP_RX_DATA_SIZE + overwritesize) {
            CDC.d[i].UsbOutBufReadPosInOverwrite = 0;
            // [NOTE] It is safe write UsbOutOverWriteSize here because USB OUT
            //        callback does not write before updating UsbOutBufReadPos.
            CDC.d[i].UsbOutOverWriteSize = 0;
        } else if (read_idx + parsed_size >= APP_RX_DATA_SIZE) {
            CDC.d[i].UsbOutBufReadPosInOverwrite = 1;
        }
        CDC.d[i].UsbOutBufReadPos += parsed_size;
    }
}

void SLCAN_Run_In_Thread_Mode() {
    SLCAN_Parse();
    mcp.Run_In_Thread_Mode();
}

uint32_t SLCAN_GetWritePos() {
    return mcp.GetNotificationWritePos();
}