#pragma once
#pragma pack(1)

#define CONFIRM_SIZE(typeA, typeB) sizeof(typeA) == sizeof(typeB), "sizeof(" #typeA ") should be same as sizeof(" #typeB ")"
#define CONFIRM_UINT8(type) CONFIRM_SIZE(type, std::uint8_t)
#define CONFIRM_UINT16(type) CONFIRM_SIZE(type, std::uint16_t)

// TX
union TXRTSCTRL {
    struct {
        std::uint32_t Blank  : 2;
        std::uint32_t B2RTS  : 1;   // !TX2RTS pin
        std::uint32_t B1RTS  : 1;   // !TX1RTS pin
        std::uint32_t B0RTS  : 1;   // !TX1RTS pin
        std::uint32_t B2RTSM : 1;   // TX2RTS pin mode (1: TXB2 send request, 0: digital in)
        std::uint32_t B1RTSM : 1;   // TX1RTS pin mode (1: TXB1 send request, 0: digital in)
        std::uint32_t B0RTSM : 1;   // TX0RTS pin mode (1: TXB0 send request, 0: digital in)
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXRTSCTRL));

union TXBnCTRL {
    struct {
        std::uint32_t Blank1 : 1;
        std::uint32_t ABTF   : 1;   // Messag aborted
        std::uint32_t MLOA   : 1;   // Message lost arbitration
        std::uint32_t TXERR  : 1;   // Tx error detected
        std::uint32_t TXREQ  : 1;   // Tx request
        std::uint32_t Blank2 : 1;
        std::uint32_t TXP1   : 1;   // Tx buffer priority
        std::uint32_t TXP0   : 1;
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnCTRL));

union TXBnSIDH {
    struct {
        std::uint32_t SID10to3 : 8; // Standard ID Bit 10 to Bit 3
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnSIDH));

union TXBnSIDL {
    struct {
        std::uint32_t SID210  : 3;  // Standard ID Bit 2 to Bit 0
        std::uint32_t Blank1  : 1;
        std::uint32_t EXIDE   : 1;  // Extnded ID Enable
        std::uint32_t Blank2  : 1;
        std::uint32_t EID1716 : 2;  // Extended ID Bit 17 to 16
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnSIDL));

union TXBnEID8 {
    struct {
        std::uint32_t EID15to8 : 8; // Extended ID Bit 15 to 8
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnEID8));

union TXBnEID0 {
    struct {
        std::uint32_t EID7to0 : 8;  // Extended ID Bit 7 to 0
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnEID0));

union TXBnDLC {
    struct {
        std::uint32_t Blank1 : 1;
        std::uint32_t RTR    : 1;   // Remote transmit request
        std::uint32_t Blank2 : 2;
        std::uint32_t DLC    : 4;   // Data length (0-8 bytes)
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnDLC));


// RX
union BFPCTRL {
    struct {
        std::uint32_t Blank : 2;
        std::uint32_t B1BFS : 1;    // !RX1BF pin output bit
        std::uint32_t B0BFS : 1;    // !RX0BF pin output bit
        std::uint32_t B1BFE : 1;    // RX1BF pin enable
        std::uint32_t B0BFE : 1;    // RX0BF pin enable
        std::uint32_t B1BFM : 1;    // RXB1 pin mode (1: RXB1 receive interrupt, 0: digital out)
        std::uint32_t B0BFM : 1;    // RXB0 pin mode (1: RXB0 receive interrupt, 0: digital out)
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(BFPCTRL));

union RXB0CTRL {
    enum RX_MODE {
        FILTER          = 0,
        EXTEND_FILTER   = 1,
        STANDARD_FILTER = 2,
        ALL             = 3
    };
    enum FILTER_HIT {
        RXF0 = 0,
        RXF1 = 1
    };
    struct {
        std::uint32_t Blank1  : 1;
        RX_MODE RXM           : 2;  // RX Mode
        std::uint32_t Blank2  : 1;
        std::uint32_t RXRTR   : 1;  // Received RTR bit
        std::uint32_t BUKT    : 1;  // Allow switch to RXB1
        std::uint32_t BUKT1   : 1;  // MCP2515 internal use bit
        FILTER_HIT FILHIT     : 1;  // Filter hit
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXB0CTRL));

union RXB1CTRL {
    enum RX_MODE {
        FILTER          = 0,
        EXTEND_FILTER   = 1,
        STANDARD_FILTER = 2,
        ALL             = 3
    };
    enum FILTER_HIT {
        RXF0 = 0,
        RXF1 = 1,
        RXF2 = 2,
        RXF3 = 3,
        RXF4 = 4,
        RXF5 = 5
    };
    struct {
        std::uint32_t Blank1  : 1;
        RX_MODE RXM           : 2;  // RX Mode
        std::uint32_t Blank2  : 1;
        std::uint32_t RXRTR   : 1;  // Received RTR bit
        FILTER_HIT FILHIT     : 3;  // Filter hit
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXB1CTRL));

union RXBnSIDH {
    struct {
        std::uint32_t SID10to3 : 8; // Standard ID Bit 10 to Bit 3
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXBnSIDH));

union RXBnSIDL {
    struct {
        std::uint32_t SID210  : 3;  // Standard ID Bit 2 to Bit 0
        std::uint32_t SRR     : 1;  // Standard frame remote transmit request
        std::uint32_t IDE     : 1;  // Extnded ID Enabled
        std::uint32_t Blank   : 1;
        std::uint32_t EID1716 : 2;  // Extended ID Bit 17 to 16
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXBnSIDL));

union RXBnEID8 {
    struct {
        std::uint32_t EID15to8 : 8; // Extended ID Bit 15 to 8
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXBnEID8));

union RXBnEID0 {
    struct {
        std::uint32_t EID7to0 : 8;  // Extended ID Bit 7 to 0
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXBnEID0));

union RXBnDLC {
    struct {
        std::uint32_t Blank1 : 1;
        std::uint32_t RTR    : 1;   // Extended frame remote transmit request
        std::uint32_t RB     : 2;   // Reserved bit
        std::uint32_t DLC    : 4;   // Data length (0-8 bytes)
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TXBnDLC));


// RX filter
union RXFnSIDH {
    struct {
        std::uint32_t SID10to3 : 8; // RX Filter Standard ID Bit 10 to Bit 3
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXFnSIDH));

union RXFnSIDL {
    struct {
        std::uint32_t SID210  : 3;  // Standard ID Bit 2 to Bit 0
        std::uint32_t Blank1  : 1;
        std::uint32_t EXIDE   : 1;  // Extnded ID Enable
        std::uint32_t Blank2  : 1;
        std::uint32_t EID1716 : 2;  // Extended ID Bit 17 to 16
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXFnSIDL));

union RXFnEID8 {
    struct {
        std::uint32_t EID15to8 : 8; // Extended ID Bit 15 to 8
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXFnEID8));

union RXFnEID0 {
    struct {
        std::uint32_t EID7to0 : 8;  // Extended ID Bit 7 to 0
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXFnEID0));

union RXMnSIDH {
    struct {
        std::uint32_t SID10to3 : 8;  // RX Mask Standard ID Bit 10 to 3
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXMnSIDH));

union RXMnSIDL {
    struct {
        std::uint32_t SID210  : 3;  // Standard ID Bit 2 to Bit 0
        std::uint32_t Blank   : 3;
        std::uint32_t EID1716 : 2;  // Extended ID Bit 17 to 16
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXMnSIDL));

union RXMnEID8 {
    struct {
        std::uint32_t EID15to8 : 8; // Extended ID Bit 15 to 8
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXMnEID8));

union RXMnEID0 {
    struct {
        std::uint32_t EID7to0 : 8;  // Extended ID Bit 7 to 0
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(RXMnEID0));


// Configuration
union CNF1 {
    struct {
        std::uint32_t SJW10   : 2;  // Synchronize Jump Width
        std::uint32_t BRP5to0 : 6;  // Baudrate prescaler
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CNF1));

union CNF2 {
    struct {
        std::uint32_t BTLMODE : 1;  // PS2 bit time length mode
        std::uint32_t SAM     : 1;  // Sampling point configuration bit
        std::uint32_t PHSEG1  : 3;  // PS1 length
        std::uint32_t PRSEG   : 3;  // Propagate segment length
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CNF2));

union CNF3 {
    struct {
        std::uint32_t SOF    : 1;   // Use CLKOUT as start of frame signal when CANCTRL.CLKEN = 1
        std::uint32_t WAKFIL : 1;   // Wakeup filter
        std::uint32_t Blank  : 3;
        std::uint32_t PHSEG2 : 3;   // PS2 length
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CNF3));


// Status
union TEC {
    struct {
        std::uint32_t TEC7to0 : 8;  // TX error count
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(TEC));

union REC {
    struct {
        std::uint32_t REC7to0 : 8;  // RX error count
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(REC));

union EFLG {
    struct {
        std::uint32_t RX1OVR : 1;   // RX1 overflow
        std::uint32_t RX0OVR : 1;   // RX0 overflow
        std::uint32_t TXBO   : 1;   // Bus off error (TEC > 255)
        std::uint32_t TXEP   : 1;   // TEC >= 128
        std::uint32_t RXEP   : 1;   // REC >= 128
        std::uint32_t TXWAR  : 1;   // TEC >= 96
        std::uint32_t RXWAR  : 1;   // REC >= 96
        std::uint32_t EWARN  : 1;   // TXWAR || RXWAR
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(EFLG));

// Interrupt
union CANINTE {
    struct {
        std::uint32_t MERRE : 1;    // Message TX/RX error interrupt
        std::uint32_t WAKIE : 1;    // Wakeup (CAN bus activity) interrupt
        std::uint32_t ERRIE : 1;    // Error interrupt enable (EFLG was changed)
        std::uint32_t TX2IE : 1;    // TX2 buffer empty interrupt enable
        std::uint32_t TX1IE : 1;    // TX1 buffer empty interrupt enable
        std::uint32_t TX0IE : 1;    // TX0 buffer empty interrupt enable
        std::uint32_t RX1IE : 1;    // RX1 buffer full interrupt enable
        std::uint32_t RX0IE : 1;    // RX0 buffer full interrupt enable
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CANINTE));

union CANINTF {
    struct {
        std::uint32_t MERRF : 1;    // Message TX/RX error interrupt
        std::uint32_t WAKIF : 1;    // Wakeup (CAN bus activity) interrupt
        std::uint32_t ERRIF : 1;    // Error interrupt (EFLG was changed)
        std::uint32_t TX2IF : 1;    // TX2 buffer empty interrupt
        std::uint32_t TX1IF : 1;    // TX1 buffer empty interrupt
        std::uint32_t TX0IF : 1;    // TX0 buffer empty interrupt
        std::uint32_t RX1IF : 1;    // RX1 buffer full interrupt
        std::uint32_t RX0IF : 1;    // RX0 buffer full interrupt
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CANINTE));

// Control / Stat
union CANCTRL {
    enum OP_MODE {
        NORMAL      = 0,
        SLEEP       = 1,
        LOOP_BACK   = 2,
        LISTEN_ONLY = 3,
        CONFIG      = 4
    };

    struct {
        OP_MODE REQOP        : 3;   // Operation mode
        std::uint32_t ABAT   : 1;   // Abort all TX
        std::uint32_t OSM    : 1;   // One shot mode
        std::uint32_t CLKEN  : 1;   // CLKOUT pin enable
        std::uint32_t CLKPRE : 2;   // CLKOUT pin prescaler
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CANCTRL));

union CANSTAT {
    enum OP_MODE {
        NORMAL      = 0,
        SLEEP       = 1,
        LOOP_BACK   = 2,
        LISTEN_ONLY = 3,
        CONFIG      = 4
    };

    struct {
        OP_MODE OPMOD        : 3;   // Operation mode
        std::uint32_t Blank1 : 1;
        std::uint32_t ICOD   : 3;   // Interrupt flag code
        std::uint32_t Blank2 : 1;
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(CANSTAT));

union SPISTAT {
    struct {
        std::uint32_t TX2IF  : 1;   // [CONINTF.TX2IF] TX2 buffer empty interrupt
        std::uint32_t TX2REQ : 1;   // [TXB2CNTRL.TXREQ] TX2 request (TX2 buffer full)
        std::uint32_t TX1IF  : 1;   // [CONINTF.TX1IF] TX1 buffer empty interrupt
        std::uint32_t TX1REQ : 1;   // [TXB1CNTRL.TXREQ] TX1 request (TX1 buffer full)
        std::uint32_t TX0IF  : 1;   // [CONINTF.TX0IF] TX0 buffer empty interrupt
        std::uint32_t TX0REQ : 1;   // [TXB0CNTRL.TXREQ] TX0 request (TX0 buffer full)
        std::uint32_t RX1IF  : 1;   // [CONINTF.RX1IF] RX1 buffer full interrupt
        std::uint32_t RX0IF  : 1;   // [CONINTF.RX0IF] RX0 buffer full interrupt
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(SPISTAT));

union SPIRXSTAT {
    enum RX_FILTER {
        RXF0      = 0,
        RXF1      = 1,
        RXF2      = 2,
        RXF3      = 3,
        RXF4      = 4,
        RXF5      = 5,
        RXF0_RXB1 = 6,
        RXF1_RXB1 = 7,
    };

    struct {
        std::uint32_t RXB1_MSG : 1; // Message exists in RXB1
        std::uint32_t RXB0_MSG : 1; // Message exists in RXB1
        std::uint32_t EX_FRAME : 1; // Extended frame
        std::uint32_t RT_FRAME : 1; // Remote frame
        RX_FILTER FILTER       : 3; // Matched filter
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(SPIRXSTAT));

union SPIRTS {
    struct {
        std::uint32_t BLANK     : 5; // RTS command bit (0b10000) will be set in RTS_DMA()
        std::uint32_t TXB2      : 1; // Request to send TXB2
        std::uint32_t TXB1      : 1; // Request to send TXB1
        std::uint32_t TXB0      : 1; // Request to send TXB0
    };
    std::uint8_t raw;
};
static_assert(CONFIRM_UINT8(SPIRTS));
