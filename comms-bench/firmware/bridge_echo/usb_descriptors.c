/*
 * USB descriptors for bridge_echo — a single CDC ACM (virtual COM) interface.
 *
 * Lifted from the standard TinyUSB cdc_msc example and trimmed to CDC only.
 * The serial number is generated from the Pico's 64-bit unique flash ID so
 * multiple boards on the same host enumerate distinctly.
 */
#include "tusb.h"
#include "pico/unique_id.h"

// Using TinyUSB's example VID + a custom-range PID. Fine for development;
// pick proper vendor-assigned IDs before any distribution.
#define USB_VID 0xCafe
#define USB_PID 0x4001
#define USB_BCD 0x0200

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // Use IAD for CDC so the host's class driver binds cleanly.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// Endpoint numbers — pick anything non-zero and unique within the device.
#define EPNUM_CDC_NOTIF  0x81
#define EPNUM_CDC_OUT    0x02
#define EPNUM_CDC_IN     0x82

uint8_t const desc_configuration[] = {
    // Config: 1 config, total length, attrib, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CDC: itf number, string idx, EP notif addr+size, EP data addr+size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
// Index 0 is the language ID (English); the rest are UTF-16LE encoded
// at runtime in tud_descriptor_string_cb().
static char const *string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },  // 0: supported language = English (0x0409)
    "Raspberry Pi",                // 1: Manufacturer
    "bridge_echo",                 // 2: Product
    NULL,                          // 3: Serial (filled in at runtime)
    "bridge_echo CDC",             // 4: CDC interface name
};

static uint16_t _desc_str[32 + 1];

// 16-byte hex string of the Pico's unique board id + NUL.
static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static void fill_serial_str(void) {
    if (serial_str[0] != 0) return;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        serial_str[2 * i]     = hex[(id.id[i] >> 4) & 0xF];
        serial_str[2 * i + 1] = hex[id.id[i] & 0xF];
    }
    serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES] = 0;
}

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count = 0;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        if (index == 3) {
            fill_serial_str();
            str = serial_str;
        }
        if (str == NULL) return NULL;

        // Cap at 31 chars to fit the UTF-16 buffer.
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 31) chr_count = 31;

        // Expand ASCII -> UTF-16LE.
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // First word is the string descriptor header: length + type.
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
