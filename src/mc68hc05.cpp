#include "mc68hc05.h"
#include <cstdio>
#include <string>

uint8_t mc68hc05::read8(uint16_t address, bool peek) {
    if (address >= ROM_BASE && address < ROM_BASE + ROM_SIZE) return rom[address - ROM_BASE];
    else if (address >= BOOTSRAP_BASE && address <= BOOTSRAP_BASE + BOOTSRAP_SIZE - 1)
        return bootstrap[address - BOOTSRAP_BASE];
    else if (address >= RAM_BASE && address < RAM_BASE + ROM_SIZE) return ram[address - RAM_BASE];

    if (address < 0x10) {
        if (log_io && !peek) printf("mc68hc05: read  @ %d:0x%02x\n", MISC & 1, address);

        if (address >= 0 && address <= 5 && !peek) {
            onPortRead(address);
        }
        if (address == 9 && !peek) {
            printf("mc68hc05: read  INTSR = 0x%02x\n", INTSR);
        }
        switch (address) {
            case 0x00:
                return PORTA;
            case 0x01:
                return PORTB;
            case 0x02:
                return PORTC;
            case 0x03:
                return PORTD;
            case 0x04:
                return PORTE;
            case 0x09:
                return INTSR;
            default:
                break;
        }
        fprintf(stderr, "mc68hc05: Read from unmapped banked IO address %d:0x%02x\n", MISC & 1, address);
    } else {
        if (log_io && !peek) printf("mc68hc05: read  @ 0x%04x\n", address);
        switch (address) {
            case 0x1d:
                return TSR2;
            case 0x3e:
                return MISC;

            default:
                break;
        }
        fprintf(stderr, "mc68hc05: Read from unmapped address 0x%04x\n", address);
    }
    running = false;
    return 0;
}

uint16_t mc68hc05::read16(uint16_t address) {
    uint16_t value = 0;
    value |= read8(address) << 8;
    value |= read8(address + 1);
    return value;
}


void mc68hc05::write8(uint16_t address, uint8_t data) {
    if (address >= RAM_BASE && address < RAM_BASE + RAM_SIZE) {
        ram[address - RAM_BASE] = data;
        return;
    }

    if (address < 0x10) {
        if (log_io) {
            printf("mc68hc05: write @ %d:0x%02x: 0x%02x\n", MISC & 1, address, data);
        }
        if ((MISC & 1) == 0) {
            if (address >= 0 && address <= 5) {
                onPortWrite(address, data); // TODO: Write first, then call
            }
            switch (address) {
                case 0x00:
                    PORTA = data;
                    return;
                case 0x01:
                    PORTB = data;
                    return;
                case 0x02:
                    PORTC = data;
                    return;
                case 0x03:
                    PORTD = data;
                    return;
                case 0x04:
                    PORTE = data;
                    return;
                case 0x05:
                    PORTF = data;
                    return;
                case 0x08:
                    INTCR = data;
                    return;
                case 0x09: // INTSR
                    if (data & (1 << 0)) INTSR &= ~(1 << 4); // Reset Key Wakeup Interrupt Flag
                    if (data & (1 << 2)) INTSR &= ~(1 << 6); // Reset IRQ2 flag
                    if (data & (1 << 3)) INTSR &= ~(1 << 7); // Reset IRQ1 flag
                    return;
                case 0x0a:
                    SPCR1 = data;
                    return;
            }
        } else {
            switch (address) {
                case 0x00:
                    DDRA = data;
                    return;
                case 0x02:
                    DDRC = data;
                    return;
                case 0x03:
                    DDRD = data;
                    return;
                case 0x04:
                    DDRE = data;
                    return;
                case 0x05:
                    DDRF = data;
                    return;
                case 0x08:
                    RCR1 = data;
                    return;
                case 0x09:
                    RCR2 = data;
                    return;
                case 0x0A:
                    WOM1 = data;
                    return;
                case 0x0B:
                    WOM2 = data;
                    return;
                case 0x0e:
                    KWIE = data;
                    return;
            }
        }
        fprintf(stderr, "mc68hc05: Write to unmapped banked IO address %d:0x%02x (data=0x%02x) \n", MISC & 1, address,
                data);
    } else {
        if (log_io) printf("mc68hc05: write @ 0x%04x: 0x%02x\n", address, data);
        switch (address) {
            case 0x10:
                TBCR1 = data;
                return;
            case 0x11:
                TBCR2 = data;
                return;
            case 0x12:
                TCR = data;
                return;
            case 0x1c:
                TCR2 = data;
                return;
            case 0x1d:
                // Reset Output Compare 2 flag
                if (data & (1 << 2)) TSR2 &= ~(1 << 6);

                // Reset Timer Input 2 flag
                if (data & (1 << 3)) TSR2 &= ~(1 << 7);
                return;
            case 0x1e:
                OC2 = data;
                return;
            case 0x1f: // TCNT2
                OC2 = 1;
                return;
            case 0x20:
                LCDCR = data;
                return;
            case 0x3e:
                MISC &= 0b11000000;
                MISC |= data & 0b00111111;
                return;

            default:
                break;
        }
        fprintf(stderr, "mc68hc05: Write to unmapped IO address 0x%02x (data=0x%02x) \n", address, data);
    }
    running = false;
}

void mc68hc05::write16(uint16_t address, uint16_t data) {
    write8(address, (data >> 8) & 0xff);
    write8(address + 1, data & 0xff);
}

std::string parseFlags(mc68hc05::ccr_t CCR) {
    std::string o;

    o += CCR.half_carry ? 'H' : '.';
    o += CCR.interrupt_mask ? 'I' : '.';
    o += CCR.negative ? 'N' : '.';
    o += CCR.zero ? 'Z' : '.';
    o += CCR.carry ? 'C' : '.';
    return o;
}

#define ADDR_CLEAR() __ADDR[0] = 0
#define ADDR(format, ...) sprintf(__ADDR, format, __VA_ARGS__)
#define TRACE(op) do { \
    if (trace) fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5llu    %02x    %s %s\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), op, __ADDR); \
} while(0)

#define TRACE_RAW(format, ...) do { \
    if (trace) fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5llu    %02x    " format "\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), __VA_ARGS__); \
} while(0)

#define HALT(op) do { \
    running = false; \
    fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5llu    %02x    Unknown opcode (%s)\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), op); \
    return false;     \
} while(0)

bool mc68hc05::step() {
    currentPC = PC;
    uint8_t opcode = read8(PC++);

    uint8_t hi = (opcode & 0xf0) >> 4;
    uint8_t lo = (opcode & 0x0f);

    ADDR_CLEAR(); // Debug
    switch (hi) {
        // Bit manipulation
        case 0: { // Bit test and branch
            uint8_t addr = read8(PC++);
            int8_t dst = read8(PC++);
            if (lo % 2 == 0) op_brbitset(addr, lo >> 1, dst);
            else op_brbitclr(addr, lo >> 1, dst);
            return true;
        }

        case 1: { // Bit set/clear
            uint8_t addr = read8(PC++);
            if (lo % 2 == 0) op_bitset(addr, lo >> 1);
            else op_bitclr(addr, lo >> 1);
            return true;
        }

            // Branch
        case 2: { // Branch
            int8_t rel = read8(PC++);
            uint16_t dst = PC + rel;
            if (op_branch(lo, dst)) return true;
            HALT("Branch");
        }

            // Read/modify/write
        case 3: // Direct
            vaddr = read8(PC++);
            ADDR("$%02x", vaddr);

            if (op_rmw(lo)) return true;

            HALT("Read/modify/write direct");
        case 4: // A
            vaddr = -1;
            ADDR("%c", 'A');

            if (op_rmw(lo)) return true;

            HALT("Read/modify/write Inherent A");
        case 5: // X
            ADDR("%c", 'X');
            vaddr = -2;
            if (op_rmw(lo)) return true;

            HALT("Read/modify/write Inherent X");
        case 6: // Indexed 8 bit
            HALT("Read/modify/write Indexed 8 bit");
        case 7: // Indexed (no offset)
            HALT("Read/modify/write Indexed (no offset)");

            // Control
        case 8: // Inherent
        case 9: // Inherent
            if (op_control(opcode)) return true;
            HALT("Control");

            // Register/memory
        case 10: { // Immediate
            vaddr = PC++;
            ADDR("#$%02x", read8(vaddr));

            if (opcode == 0xad) {// CALL/BSR
                int8_t rel = read8(vaddr);
                uint16_t dst = PC + rel;

                TRACE_RAW("BSR $%04x", dst);
                push16(PC);
                PC = dst;

                return true;
            }
            if (op_memory(lo)) return true;

            HALT("Register/memory Immediate");
        }
        case 11: {// Direct
            vaddr = read8(PC++);
            ADDR("$%02x", vaddr);
            if (op_memory(lo)) return true;

            HALT("Register/memory Direct");
        }
        case 12: {// Extended
            vaddr = read16(PC);
            PC += 2;

            ADDR("$%04x", vaddr);

            if (op_memory(lo)) return true;

            HALT("Register/memory Extended");
        }
        case 13: {// Indexed 16 bit
            uint16_t index = read16(PC);
            PC += 2;
            vaddr = X + index;
            ADDR("%c+$%04x", 'X', index);

            if (op_memory(lo)) return true;
            HALT("Register/memory Indexed 16 bit");
        }
        case 14: {// Indexed 8 bit
            uint8_t index = read8(PC++);
            vaddr = X + index;
            ADDR("%c+$%02x", 'X', index);

            if (op_memory(lo)) return true;

            HALT("Register/memory Indexed 8 bit");
        }
        case 15: // Indexed (no offset) (X)
            vaddr = X;
            ADDR("%c", 'X');
            if (op_memory(lo)) return true;

            HALT("Register/memory Indexed (no offset)");
    }

    return false;
}

void mc68hc05::op_bitset(uint8_t addr, uint8_t bit) {
    TRACE_RAW("BSET $%02x.%d", addr, bit);
    uint8_t value = read8(addr, true);
    value |= 1 << bit;
    write8(addr, value);
}

void mc68hc05::op_bitclr(uint8_t addr, uint8_t bit) {
    TRACE_RAW("BCLR $%02x.%d", addr, bit);
    uint8_t value = read8(addr, true);
    value &= ~(1 << bit);
    write8(addr, value);
}

void mc68hc05::op_brbitset(uint8_t addr, uint8_t bit, int8_t dst) {
    TRACE_RAW("BRSET%d $%02x, %d", bit, addr, dst);

    CCR.carry = (read8(addr) & (1 << bit)) != 0;
    if (CCR.carry == 1) {
        PC += dst;
    }
}

void mc68hc05::op_brbitclr(uint8_t addr, uint8_t bit, int8_t dst) {
    TRACE_RAW("BRCLR%d $%02x, %d", bit, addr, dst);
    CCR.carry = (read8(addr) & (1 << bit)) != 0;
    if (CCR.carry == 0) {
        PC += dst;
    }
}


bool mc68hc05::op_branch(uint8_t op, uint16_t dst) {
    switch (op) {
        case 0x0:
            TRACE_RAW("BRA $%04x", dst);
            PC = dst;
            return true;
        case 0x4:
            TRACE_RAW("BCC $%04x", dst);
            if (!CCR.carry) {
                PC = dst;
            }
            return true;
        case 0x5:
            TRACE_RAW("BCS $%04x", dst);
            if (CCR.carry) {
                PC = dst;
            }
            return true;
        case 0x6:
            TRACE_RAW("BNE $%04x", dst);
            if (!CCR.zero) {
                PC = dst;
            }
            return true;
        case 0x7:
            TRACE_RAW("BEQ $%04x", dst);
            if (CCR.zero) {
                PC = dst;
            }
            return true;
        case 0xa:
            TRACE_RAW("BPL $%04x", dst);
            if (!CCR.negative) {
                PC = dst;
            }
            return true;
        case 0xb:
            TRACE_RAW("BMI $%04x", dst);
            if (CCR.negative) {
                PC = dst;
            }
            return true;
    }
    return false;
}

bool mc68hc05::op_rmw(uint8_t op) {
    switch (op) {
        case 0x2: {
            TRACE("MUL");
            uint16_t result = X * A;
            X = (result >> 8) & 0xff;
            A = result & 0xff;

            CCR.half_carry = 0;
            CCR.carry = 0;
            return true;
        }
        case 0x4: {
            TRACE("LSR");
            uint8_t value = readv();
            bool lsb = (value & 1) != 0;
            value >>= 1;
            writev(value);

            CCR.carry = lsb;
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0x6: {
            TRACE("ROR");
            uint8_t value = readv();
            bool lsb = (value & 1) != 0;
            value = (CCR.carry << 7) | (value >> 1);
            writev(value);

            CCR.carry = lsb;
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0x8: {
            TRACE("LSL");
            uint8_t value = readv();
            bool msb = (value & 0x80) != 0;
            value <<= 1;
            writev(value);

            CCR.carry = msb;
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0x9: {
            TRACE("ROL");
            uint8_t value = readv();
            bool msb = (value & 0x80) != 0;
            value = (value << 1) | (CCR.carry << 0);
            writev(value);

            CCR.carry = msb;
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0xa: {
            TRACE("DEC");
            uint8_t value = readv() - 1;
            writev(value);
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0xc: {
            TRACE("INC");
            uint8_t value = readv() + 1;
            writev(value);
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            return true;
        }
        case 0xf:
            TRACE("CLR");
            writev(0);
            CCR.negative = 0;
            CCR.zero = 1;
            return true;

        default:
            return false;
    }
}

bool mc68hc05::op_control(uint8_t opcode) {
    switch (opcode) {
        case 0x80:
            TRACE("RTI");
            CCR.reg = pop8();
            A = pop8();
            X = pop8();
            PC = pop16();
            return true;
        case 0x81:
            TRACE("RTS");
            PC = pop16();
            return true;
        case 0x97:
            TRACE("TAX");
            X = A;
            return true;
        case 0x98:
            TRACE("CLC");
            CCR.carry = 0;
            return true;
        case 0x99:
            TRACE("SEC");
            CCR.carry = 1;
            return true;
        case 0x9a:
            TRACE("CLI");
            CCR.interrupt_mask = 0;
            return true;
        case 0x9b:
            TRACE("SEI");
            CCR.interrupt_mask = 1;
            return true;
        case 0x9c:
            TRACE("RSP");
            SP = 0x00ff;
            return true;
        case 0x9d:
            TRACE("NOP");
            return true;

        case 0x9f:
            TRACE("TXA");
            A = X;
            return true;

        default:
            return false;
    }
}

bool mc68hc05::op_memory(uint8_t op) {
    switch (op) {
        case 0x0: {
            TRACE("SUB");
            int16_t value = A - readv();
            A = value & 0xff;
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            CCR.carry = value < 0;
            return true;
        }
        case 0x1: {
            TRACE("CMP");
            int16_t value = A - readv();
            CCR.negative = (value & 0x80) != 0;
            CCR.zero = value == 0;
            CCR.carry = value < 0;
            return true;
        }

        case 0x4:
            TRACE("AND");
            A &= readv();

            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0x6:
            TRACE("LDA");
            A = readv();
            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0x7:
            TRACE("STA");
            writev(A);

            // TODO: Are flags set here?
            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0x8:
            TRACE("EOR");
            A ^= readv();

            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0x9:
            TRACE("ADC");
            A += readv() + CCR.carry;

            // TODO: Calculate half carry!
            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            // TODO: Calculate carry!
            return true;

        case 0xa:
            TRACE("ORA");
            A |= readv();

            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0xb:
            TRACE("ADD");
            A += readv();

            // TODO: Calculate half carry!
            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            // TODO: Calculate carry!
            return true;

        case 0xc:
            TRACE("JMP");
            PC = getv();
            return true;

        case 0xd:
            TRACE("JSR");
            push16(PC);
            PC = getv();
            return true;

        case 0xe:
            TRACE("LDX");
            X = readv();

            CCR.negative = (X & 0x80) != 0;
            CCR.zero = X == 0;
            return true;

        case 0xf:
            TRACE("STX");
            writev(X);

            CCR.negative = (X & 0x80) != 0;
            CCR.zero = X == 0;
            return true;

        default:
            return false;
    }
}


uint16_t mc68hc05::getv() const {
    return vaddr;
}

uint8_t mc68hc05::readv() {
    if (vaddr == -1) return A;
    else if (vaddr == -2) return X;
    else return read8(vaddr);
}

void mc68hc05::writev(uint8_t data) {
    if (vaddr == -1) A = data;
    else if (vaddr == -2) X = data;
    else write8(vaddr, data);
}

void mc68hc05::irq(uint16_t vector) {
    push16(PC);
    push8(X);
    push8(A);
    push8(CCR.reg);
    CCR.interrupt_mask = 1;
    PC = read16(vector);
}

std::string toBin(uint64_t n, size_t length) {
    std::string output;
    int group = 0;
    for (int i = length - 1; i >= 0; i--) {
        bool bit = n & (1 << i);

        output += bit ? '1' : '0';
        if ((i % 4) == 0) output += ' ';

        group++;
    }
    return output;
}


int CXD2510Q_length = 0;
bool CXD2510Q_prevCLOK = 1;
bool CXD2510Q_prevXLAT = 1;
uint64_t CXD2510Q_reg = 0;

uint8_t CXD1815Q_readdata = 0;
uint8_t CXD1815Q_index = 0;
uint8_t CXD1815Q_data = 0;
bool CXD1815Q_prevCS = 0;
bool CXD1815Q_prevWR = 0;
bool CXD1815Q_prevRD = 0;

bool prevLDON = 0;

// PORTB
bool DOOR_OPEN = 0;
bool CXD2510Q_sens = 1;
int CXD2510Q_sens_cnt = 0;

void mc68hc05::onPortRead(int port) {
    char P = 'A' + port;
    if (P == 'A') { // PORTA - CXD1815Q.Data
        PORTA = CXD1815Q_readdata;
    } else if (P == 'B') { // PORTB - all input
        PORTB = 0;

        CXD2510Q_sens_cnt++;
        CXD2510Q_sens = CXD2510Q_sens_cnt % 64 == 0;

        PORTB |= (DOOR_OPEN << 3);
        PORTB |= (CXD2510Q_sens<<7);
//        CXD2510Q_sens = !CXD2510Q_sens;

        printf("PORT%c read <- 0x%02x\n", 'A' + port, PORTB);
    } else if (P == 'D') { // PORTD write only, ignore reads
    } else if (P == 'E') { // PORTE write only, ignore reads
    } else {
        printf("PORT%c read\n", 'A' + port);
    }

    // PORTB - checks bit7 (CXD2510Q.SENS)
}

const char *CXD1815Q_REGS_W[0x20] = {
        "DRVIF",    // 0x00 - drive interface
        "CONFIG1",  // 0x01 - configuration 1
        "CONFIG2",  // 0x02 - configuration 2
        "DECCTL",   // 0x03 - decoder control
        "DLADR-L",  // 0x04
        "DLADR-M",  // 0x05
        "DLADR-H",  // 0x06
        "CHPCTL",   // 0x07 - chip control
        "",         // 0x08
        "INTMSK",   // 0x09 - interrupt mask
        "CLRCTL",   // 0x0A - clear control
        "CLRINT",   // 0x0B - clear interrupt status
        "HXRF-L",   // 0x0C - host transfer-low
        "HXRF-H",   // 0x0D - host transfer-high
        "HADR-L",   // 0x0E - host address-low
        "HADR-M",   // 0x0F - host address-middle
        "DADRC-L",  // 0x10
        "DADRC-M",  // 0x11
        "DADRC-H",  // 0x12
        "",         // 0x13
        "",         // 0x14
        "",         // 0x15
        "HIFCTL",   // 0x16 - host interface control
        "RESULT",   // 0x17
        "",         // 0x18
        "ADPMNT",   // 0x19
        "",         // 0x1A
        "RTCI",     // 0x1B - real-time coding information

        // Absent from docs
        "???",      // 0x1C
        "???",      // 0x1D
        "???",      // 0x1E
        "???",      // 0x1F
};

const char *CXD1815Q_REGS_R[0x20] = {
        "ECCSTS",  // 0x00 - ECC status
        "DECSTS",  // 0x01 - decoder status
        "HDRFLG",  // 0x02 - header flag
        "HDR",     // 0x03 - header
        "SHDR",    // 0x04 - sub header
        "CMADR",   // 0x05 - current minute address
        "???",     // 0x06 Absent
        "INTSTS",  // 0x07 - interrupt status
        "ADPCI",   // 0x08 - ADPCM coding information
        "",        // 0x09
        "HXFRC-L", // 0x0A - host transfer counter-low
        "HXFRC-H", // 0x0B - host transfer counter-high
        "HADRC-L", // 0x0C - host address counter-low
        "HADRC-M", // 0x0D - host address counter-middle
        "DADRC-L", // 0x0E - drive address counter-low
        "DADRC-M", // 0x0F - drive address counter-middle
        "",        // 0x10
        "HIFSTS",  // 0x11 - host interface status
        "HSTPRM",  // 0x12 - host parameter
        "HSTCMD",  // 0x13 - host command

        // Absent from docs
        "???",     // 0x14
        "???",     // 0x15
        "???",     // 0x16
        "???",     // 0x17
        "???",     // 0x18
        "???",     // 0x19
        "???",     // 0x1A
        "???",     // 0x1B
        "???",     // 0x1C
        "???",     // 0x1D
        "???",     // 0x1E
        "???",     // 0x1F
};

uint8_t swapBits(uint8_t i) {
    uint8_t o = 0;

    const int BITS = 3;
    for (int b = 0; b<=BITS; b++) {
        bool bit = (i & (1 << b)) != 0;

        o |= bit << (BITS-b);
    }
    return o;
}
void mc68hc05::onPortWrite(int port, uint8_t data) {
    char P = 'A' + port;
    if (P == 'D') { // PORTD
        // bit0 - always 1

        // CXD2510Q DATA, XLAT, CLOK
        {
            bool DATA = (data & (1 << 1)) != 0;
            bool XLAT = (data & (1 << 2)) != 0;
            bool CLOK = (data & (1 << 3)) != 0;

            if (CXD2510Q_prevCLOK == 0 && CLOK == 1) {
                CXD2510Q_reg <<= 1;
                CXD2510Q_reg |= DATA;
                CXD2510Q_length++;
            }
            CXD2510Q_prevCLOK = CLOK;

            if (CXD2510Q_prevXLAT == 1 && XLAT == 0) {
                uint8_t cxd_data[4];
                uint8_t cxd_address = swapBits(CXD2510Q_reg & 0xf);
                for (int i = 1; i< CXD2510Q_length / 4; i++) {
                    cxd_data[i-1] = swapBits((CXD2510Q_reg >> i*4) & 0xf);
                }

                printf("W CXD2510Q addr: 0x%x data: ", cxd_address);
                for (int i = 1; i< CXD2510Q_length / 4; i++) {
                    printf("0x%x ", cxd_data[i-1]);
                }
//                printf("%32s", toBin(CXD2510Q_reg, CXD2510Q_length).c_str());
//                printf("W CXD2510Q WRITE(%2d): 0x%0*llx %32s\n", CXD2510Q_length, CXD2510Q_length/4, CXD2510Q_reg,
//                       toBin(CXD2510Q_reg, CXD2510Q_length).c_str());
                printf("\n");
                CXD2510Q_reg = 0;
                CXD2510Q_length = 0;
            }
            CXD2510Q_prevXLAT = XLAT;
        }

        // CXD1815Q CS, WR, RD
        {
            bool CS = (data & (1 << 4)) != 0;
            bool WR = (data & (1 << 5)) != 0;
            bool RD = (data & (1 << 6)) != 0;

            if (CS != CXD1815Q_prevCS || WR != CXD1815Q_prevWR || RD != CXD1815Q_prevRD) {
                if (!CS) { // Active low
                    if (!WR) {
                        printf("W CXD1815Q.%02x %-8s = 0x%02x\n", CXD1815Q_index, CXD1815Q_REGS_W[CXD1815Q_index],
                               CXD1815Q_data);
                    }
                    if (!RD) {
                        printf("R CXD1815Q.%02x %-8s = ??\n", CXD1815Q_index, CXD1815Q_REGS_R[CXD1815Q_index]);
                    }
                }
            }

            CXD1815Q_prevCS = CS;
            CXD1815Q_prevWR = WR;
            CXD1815Q_prevRD = RD;
        }

        {
            bool LDON = (data & (1 << 7)) != 0;
            if (LDON != prevLDON) {
                printf("LDON: %d\n", LDON);
                prevLDON = LDON;
            }
        }
    } else if (P == 'A') {
//        printf("W CXD1815Q.DATA:  0x%02x\n", data);
        CXD1815Q_data = data;
    } else if (P == 'E') {
//        printf("W CXD1815Q.INDEX: 0x%02x\n", data & 0x1f);
        CXD1815Q_index = data & 0x1f;
    } else {
        printf("PORT%c write: 0x%02x\n", 'A' + port, data);
    }
}


