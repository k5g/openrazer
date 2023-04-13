/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2015 Terry Cain <terrys-home.co.uk>
 */

#ifndef __HID_RAZER_KRAKENV3_H
#define __HID_RAZER_KRAKENV3_H

// Codename Unknown
#define USB_DEVICE_ID_RAZER_KRAKEN_V3 0x0549

#define USB_INTERFACE_PROTOCOL_NONE 0

union razer_kraken_effect_byte {
    unsigned char value;
    struct razer_kraken_effect_byte_bits {
        unsigned char on_off_static :1;
        unsigned char single_colour_breathing :1;
        unsigned char spectrum_cycling :1;
        unsigned char sync :1;
        unsigned char two_colour_breathing :1;
        unsigned char three_colour_breathing :1;
    } bits;
};

struct razer_kraken_device {
    struct usb_device *usb_dev;
    struct mutex lock;
    unsigned char usb_interface_protocol;
    unsigned short usb_pid;
    unsigned short usb_vid;

    unsigned char brightness;

    char serial[23];
    // 3 Bytes, first byte is whether fw version is collected, 2nd byte is major version, 3rd is minor, should be printed out in hex form as are bcd
    unsigned char firmware_version[3];

    u8 data[33];
};

struct razer_krakenv3_device_info_report {
    unsigned char report_id; /* 0x04 */
    unsigned char destination; /* 0x20 */
    unsigned char length;
    unsigned char addr_h;
    unsigned char addr_l;
    unsigned char arguments[22];
};

struct razer_krakenv3_report {
    unsigned char report_id; /*0x40*/
    unsigned char command;
    unsigned char extra;
    unsigned char arguments[10];
};






#endif
