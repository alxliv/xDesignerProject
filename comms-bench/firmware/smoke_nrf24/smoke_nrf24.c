/*
 * smoke_nrf24.c — Pre-driver SPI smoke test for the nRF24L01+PA+LNA.
 *
 * Goal: prove that the SPI wiring + power are good BEFORE we write a
 * real radio driver. We do four things:
 *
 *   1. Configure SPI0 at 1 MHz (slow, safe).
 *   2. Read CONFIG, RF_SETUP, STATUS at power-on — these have known
 *      datasheet defaults. If we read junk, wiring or power is wrong.
 *   3. Write a distinctive value to CONFIG, read it back. This proves
 *      writes work and SPI is bidirectional (not just MOSI-stuck).
 *   4. Loop forever printing the registers once every 2 s, so a stuck
 *      or intermittent SPI shows up as drift.
 *
 * Output goes over USB-CDC via the SDK's pico_stdio_usb (printf-style).
 * Watch it with PuTTY / minicom / the VS Code Serial Monitor.
 *
 * Wire-up (same on both Picos):
 *   GP16 = MISO     GP17 = CSN     GP18 = SCK     GP19 = MOSI
 *   GP14 = CE       GP15 = IRQ (unused in this smoke test)
 *   3V3, GND  — and 10 µF + 100 nF directly on the module's VCC/GND.
 */
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif
#define LED_PIN PICO_DEFAULT_LED_PIN

#define SPI_PORT     spi0
#define SPI_HZ       (1 * 1000 * 1000)   /* 1 MHz — deliberately slow */

#define PIN_MISO     16
#define PIN_CSN      17
#define PIN_SCK      18
#define PIN_MOSI     19
#define PIN_CE       14
#define PIN_IRQ      15   /* not used here, just reserved */

/* nRF24L01 SPI command opcodes */
#define CMD_R_REGISTER   0x00   /* OR with 5-bit reg address */
#define CMD_W_REGISTER   0x20   /* OR with 5-bit reg address */
#define CMD_NOP          0xFF

/* nRF24L01 register addresses we'll touch */
#define REG_CONFIG       0x00
#define REG_RF_SETUP     0x06
#define REG_STATUS       0x07

/* POR (power-on reset) defaults from the datasheet */
#define POR_CONFIG       0x08   /* EN_CRC = 1, everything else zero        */
#define POR_RF_SETUP     0x0E   /* 2 Mbps + 0 dBm + obsolete bit           */
/* STATUS POR depends on FIFO state; usually 0x0E (RX_P_NO = 111 = empty). */

static inline void csn_low(void)  { gpio_put(PIN_CSN, 0); asm volatile("nop \n nop \n nop"); }
static inline void csn_high(void) { asm volatile("nop \n nop \n nop"); gpio_put(PIN_CSN, 1); }

static uint8_t spi_xfer_byte(uint8_t out) {
    uint8_t in = 0;
    spi_write_read_blocking(SPI_PORT, &out, &in, 1);
    return in;
}

/* Returns register value; ignores STATUS byte returned in the first SPI slot. */
static uint8_t nrf24_read_reg(uint8_t reg) {
    csn_low();
    (void) spi_xfer_byte(CMD_R_REGISTER | (reg & 0x1F));
    uint8_t value = spi_xfer_byte(CMD_NOP);
    csn_high();
    return value;
}

/* Returns the STATUS byte the chip clocks back during the command phase. */
static uint8_t nrf24_write_reg(uint8_t reg, uint8_t value) {
    csn_low();
    uint8_t status = spi_xfer_byte(CMD_W_REGISTER | (reg & 0x1F));
    (void) spi_xfer_byte(value);
    csn_high();
    return status;
}

static void diagnose(uint8_t initial, uint8_t after, uint8_t wrote, const char *what) {
    printf("\n=== %s ===\n", what);
    printf("  wrote     0x%02X\n", wrote);
    printf("  read back 0x%02X\n", after);
    if (after == wrote) {
        printf("  ==> OK: SPI write/read round-trip succeeded.\n");
    } else if (after == initial) {
        printf("  ==> FAIL: write was ignored — chip kept the previous value.\n");
        printf("           Check: CSN wiring (GP17), module power, decoupling.\n");
    } else if (after == 0x00) {
        printf("  ==> FAIL: read returned all-zeros.\n");
        printf("           Check: MISO wiring (GP16) and module 3V3 supply.\n");
    } else if (after == 0xFF) {
        printf("  ==> FAIL: read returned all-ones.\n");
        printf("           Check: MISO floating? Missing pull / wrong pin.\n");
    } else {
        printf("  ==> WARN: unexpected value. SPI may be flaky.\n");
    }
}

int main(void) {
    stdio_init_all();

    /* LED: heartbeat so we can spot a dead board even without a terminal. */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    /* Wait (with timeout) for the USB host to enumerate so the user's
     * terminal can catch the opening banner. */
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }

    printf("\n\n#####################################\n");
    printf("# nRF24L01+PA+LNA SPI smoke test    #\n");
    printf("#####################################\n");
    printf("Pins:  MISO=GP%d  MOSI=GP%d  SCK=GP%d  CSN=GP%d  CE=GP%d\n",
           PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CSN, PIN_CE);
    printf("SPI:   %d Hz on spi0\n", SPI_HZ);

    /* SPI peripheral init */
    spi_init(SPI_PORT, SPI_HZ);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    /* CSN and CE as plain GPIO outputs (we control CSN per transaction). */
    gpio_init(PIN_CSN); gpio_set_dir(PIN_CSN, GPIO_OUT); csn_high();
    gpio_init(PIN_CE);  gpio_set_dir(PIN_CE,  GPIO_OUT); gpio_put(PIN_CE, 0);

    /* The nRF24 needs ~100 ms after Vcc-rise to be ready for SPI. */
    sleep_ms(100);

    /* ---- 1. Read the power-on defaults ---- */
    uint8_t cfg0 = nrf24_read_reg(REG_CONFIG);
    uint8_t rfs0 = nrf24_read_reg(REG_RF_SETUP);
    uint8_t st0  = nrf24_read_reg(REG_STATUS);

    printf("\n--- Initial registers (power-on defaults) ---\n");
    printf("  CONFIG   = 0x%02X   (datasheet POR 0x%02X)\n", cfg0, POR_CONFIG);
    printf("  RF_SETUP = 0x%02X   (datasheet POR 0x%02X)\n", rfs0, POR_RF_SETUP);
    printf("  STATUS   = 0x%02X   (typically 0x0E at POR)\n", st0);

    bool por_looks_right = (cfg0 == POR_CONFIG) && (rfs0 == POR_RF_SETUP);
    if (por_looks_right) {
        printf("  ==> Initial registers match datasheet. Wiring + power look healthy.\n");
    } else {
        printf("  ==> WARN: at least one register is off the datasheet default.\n");
        printf("           Either the chip was previously configured (e.g. by an old\n");
        printf("           firmware) or there's an SPI integrity issue. Continuing...\n");
    }

    /* ---- 2. CONFIG round-trip ---- */
    uint8_t cfg_test = 0x0E;   /* PWR_UP=1, EN_CRC=1, CRCO=1 (16-bit CRC) */
    nrf24_write_reg(REG_CONFIG, cfg_test);
    sleep_ms(5);
    uint8_t cfg1 = nrf24_read_reg(REG_CONFIG);
    diagnose(cfg0, cfg1, cfg_test, "CONFIG round-trip");

    /* ---- 3. RF_SETUP round-trip with a different bit pattern ---- */
    uint8_t rfs_test = 0x06;   /* RF_DR_HIGH=0 (1 Mbps), RF_PWR=11 (0 dBm) */
    nrf24_write_reg(REG_RF_SETUP, rfs_test);
    sleep_ms(5);
    uint8_t rfs1 = nrf24_read_reg(REG_RF_SETUP);
    diagnose(rfs0, rfs1, rfs_test, "RF_SETUP round-trip");

    /* Restore POR-ish state so we don't leave the chip in a weird config
     * for whatever firmware runs next on this Pico. */
    nrf24_write_reg(REG_CONFIG, POR_CONFIG);
    nrf24_write_reg(REG_RF_SETUP, POR_RF_SETUP);

    /* ---- 4. Periodic re-read so the user can watch stability ---- */
    printf("\n--- Periodic re-read (CONFIG should stay at 0x%02X) ---\n", POR_CONFIG);
    printf("If the value flickers or changes spontaneously, SPI is marginal\n");
    printf("(loose wire, weak power) or the module is being reset by spikes.\n\n");

    bool led = true;
    absolute_time_t next_led = make_timeout_time_ms(500);
    absolute_time_t next_print = make_timeout_time_ms(2000);
    int reads_total = 0, reads_ok = 0;

    while (1) {
        if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
            led = !led;
            gpio_put(LED_PIN, led);
            next_led = make_timeout_time_ms(500);
        }
        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {
            uint8_t c = nrf24_read_reg(REG_CONFIG);
            uint8_t r = nrf24_read_reg(REG_RF_SETUP);
            reads_total++;
            if (c == POR_CONFIG && r == POR_RF_SETUP) reads_ok++;
            printf("[%5d] CONFIG=0x%02X  RF_SETUP=0x%02X   stable=%d/%d\n",
                   reads_total, c, r, reads_ok, reads_total);
            next_print = make_timeout_time_ms(2000);
        }
    }
}
