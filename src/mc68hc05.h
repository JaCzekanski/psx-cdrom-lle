#pragma once

#include <cstdint>
#include <cstddef>

struct mc68hc05 {
    union ccr_t {
        struct {
            uint8_t carry: 1;
            uint8_t zero: 1;
            uint8_t negative: 1;
            uint8_t interrupt_mask: 1; // 1 - disabled, 0 - enabled
            uint8_t half_carry: 1;
            uint8_t : 3;
        };
        uint8_t reg;

        ccr_t() : reg(0) {}
    };

    // Core Registers
    uint8_t A = 0;
    uint8_t X = 0;
    uint8_t SP = 0xff; // 11xxxxxx
    uint16_t PC = 0;
    ccr_t CCR;

    // IO Registers
    const static inline uint16_t RAM_BASE = 0x40;
    const static inline uint16_t RAM_SIZE = 0x200;

    const static inline uint16_t ROM_BASE = 0x1000;
    const static inline uint16_t ROM_SIZE = 0x4000;

    const static inline uint16_t BOOTSRAP_BASE = 0xfe00;
    const static inline uint16_t BOOTSRAP_SIZE = 0x200;

    uint8_t ram[RAM_SIZE]; // ?? - ??
    uint8_t rom[ROM_SIZE]; // 0x1000-0x4fff
    uint8_t bootstrap[BOOTSRAP_SIZE]; // 0xfe00 - 0xffff

    uint8_t PORTA = 0; // 0:0x00
    uint8_t PORTB = 0; // 0:0x01
    uint8_t PORTC = 0; // 0:0x02
    uint8_t PORTD = 0; // 0:0x03
    uint8_t PORTE = 0; // 0:0x04
    uint8_t PORTF = 0; // 0:0x05
    uint8_t INTCR = 0; // 0:0x08 Interrupt Control Register
    uint8_t INTSR = 0; // 0:0x09 Interrupt Status Register

    uint8_t SPCR1 = 0; // 0:0x0A

    uint8_t DDRA = 0;  // 1:0x00
    uint8_t DDRC = 0;  // 1:0x02
    uint8_t DDRD = 0;  // 1:0x03
    uint8_t DDRE = 0;  // 1:0x04
    uint8_t DDRF = 0;  // 1:0x05

    uint8_t RCR1 = 0;  // 1:0x08 - Resistor Control Register 1
    uint8_t RCR2 = 0;  // 1:0x09 - Resistor Control Register 2

    uint8_t WOM1 = 0;  // 1:0x0a - Open Drain output control Register 1
    uint8_t WOM2 = 0;  // 1:0x0b - Open Drain output control Register 2

    uint8_t KWIE = 0;  // 1:0x0e - Key Wakeup Interrupt enable Register

    uint8_t TBCR1 = 0;  // 0x10 - Time Base Control Register 1
    uint8_t TBCR2 = 0;  // 0x11 - Time Base Control Register 2
    uint8_t TCR = 0;    // 0x12 - Timer 1 Control Register
    uint8_t TCR2 = 0;   // 0x1C - Timer 2 Control Register
    uint8_t TSR2 = 0;   // 0x1D - Timer 2 Status Register
    uint8_t OC2 = 0;    // 0x1E - Timer 2 Output Compare Register

    uint8_t LCDCR = 0; // 0x20 - LCD Control Register

    uint8_t MISC = 1 << 7; // 0x3e  bit7 - OSC stable

    // Emulator related
    int32_t vaddr = 0; // Used as instruction data pointer
    bool running = true;
    uint16_t currentPC = 0;
    uint64_t ticks = 0;

    bool log_io = false;
    bool trace = true;

    // Debug
    char __ADDR[64] = {0};

    void reset() {
        A = 0;
        X = 0;
        SP = 0xff;
        CCR.reg = 0;

        PC = read16(0xfffe);
    }

    uint8_t read8(uint16_t address, bool peek = false);
    uint16_t read16(uint16_t address);

    void write8(uint16_t address, uint8_t data);
    void write16(uint16_t address, uint16_t data);
    bool step();

    void push8(uint8_t data) {
        write8(SP--, data);
    }
    void push16(uint16_t data) {
        write8(SP--, data&0xff);
        write8(SP--, (data>>8) & 0xff);
    }

    uint8_t pop8() {
        return read8(++SP);
    }

    uint16_t pop16() {
        uint16_t value = 0;
        value |= read8(++SP) << 8;
        value |= read8(++SP);
        return value;
    }

    void irq(uint16_t vector);

    void onPortRead(int port);
    void onPortWrite(int port, uint8_t data);

private:
    void op_brbitset(uint8_t addr, uint8_t bit, int8_t i);
    void op_brbitclr(uint8_t addr, uint8_t bit, int8_t i);
    void op_bitset(uint8_t addr, uint8_t bit);
    void op_bitclr(uint8_t addr, uint8_t bit);

    bool op_control(uint8_t opcode);
    bool op_memory(uint8_t op);
    bool op_rmw(uint8_t op);
    bool op_branch(uint8_t op, uint16_t dst);

    // Addressing helpers
    uint16_t getv() const;
    uint8_t readv();
    void writev(uint8_t);

};
