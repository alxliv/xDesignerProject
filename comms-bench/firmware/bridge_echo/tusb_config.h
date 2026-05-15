#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// CFG_TUSB_MCU is set automatically by the Pico SDK based on PICO_PLATFORM
// (rp2040 / rp2350-arm-s), so we do not define it here.

#define CFG_TUSB_OS              OPT_OS_PICO

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG           0
#endif

#define CFG_TUD_ENABLED          1
#define CFG_TUD_ENDPOINT0_SIZE   64

// RP2040/RP2350 native USB is full-speed device only. Declaring this
// here lets the no-arg `tusb_init()` resolve the root-hub port mode.
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_CDC              1
#define CFG_TUD_MSC              0
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

// Bench protocol frames are small; 256 B is plenty and fits one bulk
// transfer comfortably.
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256

#ifdef __cplusplus
}
#endif

#endif
