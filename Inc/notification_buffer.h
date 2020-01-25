#pragma once

class NotificationBuffer {
    std::uint8_t* buffer_;
    std::uint32_t size_;
    std::uint32_t write_pos_;

  public:
    const std::uint8_t ACK = '\r';
    const char ACK11[3] = "z\r";
    const char ACK29[3] = "Z\r";
    const char VERSION[7] = "V1010\r";
    const char MINOR[7] = "v0000\r";
    const char SERIAL[7] = "N0000\r";
    const std::uint8_t NACK = '\a';

    void Set(std::uint8_t* buffer, std::uint32_t size, std::uint32_t write_pos = 0U) {
        buffer_ = buffer;
        size_ = size;
        write_pos_ = write_pos;
    }

    std::uint32_t GetWritePos() {
        return write_pos_;
    }

    void Write(const std::uint8_t* buffer, std::size_t size) {
        for (auto i = 0U; i < size; i++) {
            buffer_[write_pos_] = buffer[i];
            write_pos_ = (write_pos_ + 1) & (size_ - 1);
        }
    }

    void Write(const char* buffer, std::size_t size) {
        Write(reinterpret_cast<const std::uint8_t*>(buffer), size);
    }

    void Ack() {
        Write(&ACK, 1);
    }

    void TransmitAck11() {
        Write(ACK11, sizeof(ACK11) - 1);
    }

    void TransmitAck29() {
        Write(ACK29, sizeof(ACK29) - 1);
    }

    void Version() {
        Write(VERSION, sizeof(VERSION) - 1);
    }

    void Minor() {
        Write(MINOR, sizeof(MINOR) - 1);
    }

    void Serial() {
        Write(SERIAL, sizeof(SERIAL) - 1);
    }

    void NAck() {
        Write(&NACK, 1);
    }
};