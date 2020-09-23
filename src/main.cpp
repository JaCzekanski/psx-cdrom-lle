#include <iostream>
#include "mc68hc05.h"

uint8_t toHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    else if (c >= 'a' && c <= 'f') return (c - 'a') + 10;
    else if (c >= 'A' && c <= 'F') return (c - 'A') + 10;
    else return 0;
}

bool loadBin(const char *path, mc68hc05 *cpu) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size != mc68hc05::ROM_SIZE + mc68hc05::BOOTSRAP_SIZE) {
        fprintf(stderr, ".bin file must be %d in size.", mc68hc05::ROM_SIZE + mc68hc05::BOOTSRAP_SIZE);
        fclose(f);
        return false;
    }

    fread(cpu->rom, mc68hc05::ROM_SIZE, 1, f);
    fread(cpu->bootstrap, mc68hc05::BOOTSRAP_SIZE, 1, f);
    fclose(f);

    return true;
}
bool loadSrec(const char *path, mc68hc05 *cpu) {
    char buffer[256];
    int ptr = 0;

    FILE *f = fopen(path, "rb");
    if (!f) return false;


    auto read8 = [&]() -> uint8_t {
        uint8_t value = 0;
        value |= toHex(buffer[ptr++]) << 4u;
        value |= toHex(buffer[ptr++]) << 0u;
        return value;
    };

    auto read16 = [&]() -> uint16_t {
        uint16_t value = 0;
        value |= toHex(buffer[ptr++]) << 12u;
        value |= toHex(buffer[ptr++]) << 8u;
        value |= toHex(buffer[ptr++]) << 4u;
        value |= toHex(buffer[ptr++]) << 0u;
        return value;
    };

    int line = 1;
    while (fgets(buffer, sizeof(buffer), f)) {
        ptr = 0;
        if (buffer[ptr++] != 'S') {
            fprintf(stderr, "srec - invalid header at line %d\n", line);
            fclose(f);
            return false;
        }
        // Record type
        uint8_t recordType = buffer[ptr++];
        switch (recordType) {
            case '0': { // Header
                uint8_t size = read8();
                uint16_t address = read16();
                break;
            }
            case '1': { // Data, 16bit address
                uint8_t size = read8();
                uint16_t address = read16();

                for (int i = 0; i < size - 3; i++) {
                    uint8_t byte = read8();
                    uint16_t ptr = address + i;
                    if (ptr >= mc68hc05::ROM_BASE && ptr < mc68hc05::ROM_BASE + mc68hc05::ROM_SIZE) {
                        cpu->rom[ptr - mc68hc05::ROM_BASE] = byte;
                    } else if (address >= mc68hc05::BOOTSRAP_BASE &&
                               address <= mc68hc05::BOOTSRAP_BASE + mc68hc05::BOOTSRAP_SIZE - 1) {
                        cpu->bootstrap[ptr - mc68hc05::BOOTSRAP_BASE] = byte;
                    } else {
                        fprintf(stderr, "srec - unmapped address space at 0x%04x (line %d), breaking.\n", ptr,
                                line);
                        break;
                    }
                }
                break;
            }
            case '5': { // Count of S1/S2/S3 records , 16bit count, ignore
                uint8_t size = read8();
                uint16_t count = read16();
                break;
            }
            case '9': { // Start Address (Termination), 16bit address, ignore
                uint8_t count = read8();
                uint16_t address = read16();
                break;
            }
            default:
                fprintf(stderr, "srec - invalid record type %c at line %d\n", recordType, line);
                fclose(f);
                return false;
        }

        line++;
    }

    fclose(f);
    return true;
}

void dumpIo(mc68hc05 *cpu) {
#define D(reg) printf("%-6s: 0x%02x\n", #reg, cpu->reg)

    D(PORTA);
    D(PORTB);
    D(PORTC);
    D(PORTD);
    D(PORTE);
    D(PORTF);

    D(SPCR1);

    D(DDRA);
    D(DDRC);
    D(DDRD);
    D(DDRE);
    D(DDRF);

    D(RCR1);
    D(RCR2);

    D(WOM1);
    D(WOM2);

    D(KWIE);

    D(TBCR1);
    D(TBCR2);
    D(TCR);
    D(TCR2);
    D(OC2);

    D(LCDCR);

    D(MISC);
}

int main() {
    const char *romPath = "/Users/jakub/Desktop/SC430930/SC430930.S19";

    mc68hc05 subcpu;
    if (!loadSrec(romPath, &subcpu)) {
        fprintf(stderr, "Unable to load %s\n", romPath);
        return 1;
    }

    subcpu.reset();
    for (int i = 0; i < 1000000; i++) {
        if (!subcpu.step()) break;
        if (!subcpu.running) break;

        // Notes SC430930:
        // JSR $408a - Wait for 0x64 Timer2 OC IRQs


        // Main loop is waiting for 64 timer2 IRQs
        // $4b.7 - Timer2 IRQ
        if (i > 2000 && (i % 50) == 0) {
//            FFF2h  6    SSPI Vector (SPI bus)     (SPI1 and SPI2)
//            FFF4h  5    Timer 2 Interrupt Vector  (Timer 2 Input/Compare)
//            FFF6h  4    Timer 1 Interrupt Vector  (Timer 1 Input/Compare/Overflow)
//            FFF8h  3    KWI Vector (Key Wakeup)   (KWI0..7 pins)
//            FFFAh  2    External Interrupt Vector (/IRQ1 and /IRQ2 pins)
//            FFFCh  none Software Interrupt Vector (SWI opcode)            ;\regardless of
//            FFFEh  1=hi Reset Vector              (/RESET signal and COP) ;/CPU's "I"

            // 0xfff2 - SSPI - unused
            // 0xfff4 - Timer 2 - Trigger $4b.7 Timer2 IRQ
            // 0xfff6 - Timer 1 - unused
            // 0xfff8 - KWI - unused
            // 0xfffa - External interrupt /IRQ1 /IRQ2 - unused
            // 0xfffc - SWI - unused

            subcpu.irq(0xfff4);
//            printf("IRQ 0x%04x\n", 0xfff4);
        }
    }
//    dumpIo(&subcpu);
    return 0;
}
