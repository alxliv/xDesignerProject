/*
 * bridge_echo.c — Phase 1 of the comms latency bench.
 *
 * USB-CDC echo. Reads any bytes that arrive over USB and immediately
 * writes them back. No radio yet — that comes in Phase 2.
 *
 * Heartbeat: on-board LED blinks at 1 Hz while running.
 *
 * Build (see firmware/bridge_echo/README hint in repo root):
 *     mkdir build && cd build
 *     cmake -DPICO_BOARD=pico ..        # or -DPICO_BOARD=pico2 for RP2350
 *     cmake --build . -j
 *
 * Flash: hold BOOTSEL, plug in, drag bridge_echo.uf2 onto the mounted
 * RPI-RP2 / RP2350 volume.
 */
#include <stdint.h>

#include "pico/stdlib.h"
#include "tusb.h"

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define LED_PIN PICO_DEFAULT_LED_PIN

int main(void) {
    /* stdio_init_all() brings up tinyusb's CDC interface as a side-effect
     * of enabling stdio_usb. We then talk to it directly through the
     * tud_cdc_* API for binary IO. */
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint8_t buf[64];
    absolute_time_t next_led = make_timeout_time_ms(500);
    bool led_state = false;

    while (true) {
        /* TinyUSB housekeeping — must be called frequently. */
        tud_task();

        if (tud_cdc_connected()) {
            uint32_t avail = tud_cdc_available();
            if (avail > 0) {
                uint32_t want = avail > sizeof(buf) ? sizeof(buf) : avail;
                uint32_t n = tud_cdc_read(buf, want);
                if (n > 0) {
                    /* Echo the exact bytes back. The bench script
                     * validates magic + CRC + payload on the way in. */
                    uint32_t written = 0;
                    while (written < n) {
                        uint32_t w = tud_cdc_write(buf + written, n - written);
                        written += w;
                        if (w == 0) {
                            /* Output buffer full — flush and retry. */
                            tud_cdc_write_flush();
                            tud_task();
                        }
                    }
                    tud_cdc_write_flush();
                }
            }
        }

        /* 1 Hz heartbeat so it's obvious the firmware is alive. */
        if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            next_led = make_timeout_time_ms(500);
        }
    }
}
