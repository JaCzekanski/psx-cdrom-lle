#include "mc68hc05.h"
#include <cstdio>
#include <string>

uint8_t mc68hc05::read8(uint16_t address) {
    if (address >= ROM_BASE && address < ROM_BASE + ROM_SIZE) return rom[address - ROM_BASE];
    else if (address >= BOOTSRAP_BASE && address < BOOTSRAP_BASE + BOOTSRAP_SIZE)
        return bootstrap[address - BOOTSRAP_BASE];
    else if (address >= RAM_BASE && address < RAM_BASE + ROM_SIZE) return ram[address - RAM_BASE];

    if (address < 0x10) {
        if (log_io) printf("mc68hc05: read  @ %d:0x%02x\n", MISC & 1, address);

        switch (address) {
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
        if (log_io) printf("mc68hc05: read  @ 0x%04x\n", address);
        switch (address) {
            case 0x1d:
                return TSR2;
            case 0x3e: // MISC
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
        if (log_io) printf("mc68hc05: write @ %d:0x%02x: 0x%02x\n", MISC & 1, address, data);
        if ((MISC & 1) == 0) {
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
                case 0x09:
                    if (data & (1<<0)) INTSR &= ~(1<<4); // Reset Key Wakeup Interrupt Flag
                    if (data & (1<<2)) INTSR &= ~(1<<6); // Reset IRQ2 flag
                    if (data & (1<<3)) INTSR &= ~(1<<7); // Reset IRQ1 flag
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
            case 0x1f:
                OC2 = 1;
                return;
            case 0x20:
                LCDCR = data;
                return;
            case 0x3e: // MISC
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
    fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5d    %02x    %s %s\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), op, __ADDR); \
} while(0)

#define TRACE_RAW(format, ...) do { \
    fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5d    %02x    " format "\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), __VA_ARGS__); \
} while(0)

#define HALT(op) do { \
    running = false; \
    fprintf(stdout, "mc68hc05: PC=%04x, A=%02x, X=%02x, SP=%02x, CCR=%s  tick=%-5d    %02x    Unknown opcode (%s)\n", currentPC, A, X, SP, parseFlags(CCR).c_str(), ticks++, read8(currentPC), op); \
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
            break;
        }

        case 1: { // Bit set/clear
            uint8_t addr = read8(PC++);
            if (lo % 2 == 0) op_bitset(addr, lo >> 1);
            else op_bitclr(addr, lo >> 1);
            return true;
        }

            // Branch
        case 2: { // Branch
            int8_t dst = read8(PC++);
            if (op_branch(lo, dst)) return true;
            HALT("Branch");
            break;
        }

            // Read/modify/write
        case 3: // Direct
            vaddr = read8(PC++);
            ADDR("$%02x", vaddr);

            if (op_rmw(lo)) return true;

            HALT("Read/modify/write direct");
            break;
        case 4: // A
            vaddr = -1;
            ADDR("%c", 'A');

            if (op_rmw(lo)) return true;

            HALT("Read/modify/write Inherent A");
            break;
        case 5: // X
            ADDR("%c", 'X');
            vaddr = -2;
            if (op_rmw(lo)) return true;

            HALT("Read/modify/write Inherent X");
            break;
        case 6: // Indexed 8 bit
            HALT("Read/modify/write Indexed 8 bit");
            break;
        case 7: // Indexed (no offset)
            HALT("Read/modify/write Indexed (no offset)");
            break;

            // Control
        case 8: // Inherent
        case 9: // Inherent
            if (op_control(opcode)) return true;
            HALT("Control");
            break;

            // Register/memory
        case 10: { // Immediate
            vaddr = PC++;
            ADDR("#$%02x", read8(vaddr));

            if (opcode == 0xad) {// CALL/BSR
                int8_t dst = vaddr;

                TRACE_RAW("BSR %d", dst);
                push16(PC);
                PC += dst;

                return true;
            }
            if (op_memory(lo)) return true;

            HALT("Register/memory Immediate");
            break;
        }
        case 11: {// Direct
            vaddr = read8(PC++);
            ADDR("$%02x", vaddr);
            if (op_memory(lo)) return true;

            HALT("Register/memory Direct");
            break;
        }
        case 12: {// Extended
            vaddr = read16(PC);
            PC += 2;

            ADDR("$%04x", vaddr);

            if (op_memory(lo)) return true;

            HALT("Register/memory Extended");
            break;
        }
        case 13: {// Indexed 16 bit
            uint16_t index = read16(PC);
            PC += 2;
            vaddr = X + index;
            ADDR("%c+$%04x", 'X', index);

            if (op_memory(lo)) return true;
            HALT("Register/memory Indexed 16 bit");
            break;
        }
        case 14: {// Indexed 8 bit
            uint8_t index = read8(PC++);
            vaddr = X + index;
            ADDR("%c+$%02x", 'X', index);

            if (op_memory(lo)) return true;

            HALT("Register/memory Indexed 8 bit");
            break;
        }
        case 15: // Indexed (no offset) (X)
            vaddr = X;
            ADDR("%c", 'X');
            if (op_memory(lo)) return true;

            HALT("Register/memory Indexed (no offset)");
            break;
    }

    return false;
}

void mc68hc05::op_bitset(uint8_t addr, uint8_t bit) {
    TRACE_RAW("BSET $%02x.%d", addr, bit);
    uint8_t value = read8(addr);
    value |= 1 << bit;
    write8(addr, value);
}

void mc68hc05::op_bitclr(uint8_t addr, uint8_t bit) {
    TRACE_RAW("BCLR $%02x.%d", addr, bit);
    uint8_t value = read8(addr);
    value &= ~(1 << bit);
    write8(addr, value);
}

void mc68hc05::op_brbitset(uint8_t addr, uint8_t bit, int8_t dst) {
    TRACE_RAW("BRSET $%02x.%d, %d", addr, bit, dst);

    CCR.carry = (read8(addr) & (1 << bit)) != 0;
    if (CCR.carry == 1) {
        PC += dst;
    }
}

void mc68hc05::op_brbitclr(uint8_t addr, uint8_t bit, int8_t dst) {
    TRACE_RAW("BRCLR $%02x.%d, %d", addr, bit, dst);
    CCR.carry = (read8(addr) & (1 << bit)) != 0;
    if (CCR.carry == 0) {
        PC += dst;
    }
}


bool mc68hc05::op_branch(uint8_t op, int8_t dst) {
    switch (op) {
        case 0x0:
            TRACE_RAW("BRA %d", dst);
            PC += dst;
            return true;
        case 0x4:
            TRACE_RAW("BCC %d", dst);
            if (!CCR.carry) {
                PC += dst;
            }
            return true;
        case 0x5:
            TRACE_RAW("BCS %d", dst);
            if (CCR.carry) {
                PC += dst;
            }
            return true;
        case 0x6:
            TRACE_RAW("BNE %d", dst);
            if (!CCR.zero) {
                PC += dst;
            }
            return true;
        case 0x7:
            TRACE_RAW("BEQ %d", dst);
            if (CCR.zero) {
                PC += dst;
            }
            return true;
        case 0xb:
            TRACE_RAW("BMI %d", dst); // TODO: Use hex with sign
            if (CCR.negative) {
                PC += dst;
            }
            return true;
    }
    return false;
}

bool mc68hc05::op_rmw(uint8_t op) {
    switch (op) {
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
        case 0x6: // LDA
            TRACE("LDA");
            A = readv();
            CCR.negative = (A & 0x80) != 0;
            CCR.zero = A == 0;
            return true;

        case 0x7: // STA
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

        case 0xd: //JSR
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


