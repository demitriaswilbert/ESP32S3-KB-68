/*
  Keyboard.h

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once
#include "Print.h"
#include "USBHID.h"
#if CONFIG_TINYUSB_HID_ENABLED

#include "esp_event.h"

extern const uint8_t report_descriptor[];
extern const size_t report_descriptor_size;

ESP_EVENT_DECLARE_BASE(ARDUINO_USB_HID_KEYBOARD_EVENTS);

typedef enum {
    ARDUINO_USB_HID_KEYBOARD_ANY_EVENT = ESP_EVENT_ANY_ID,
    ARDUINO_USB_HID_KEYBOARD_LED_EVENT = 0,
    ARDUINO_USB_HID_KEYBOARD_MAX_EVENT,
} arduino_usb_hid_keyboard_event_t;

typedef union {
    struct {
            uint8_t numlock:1;
            uint8_t capslock:1;
            uint8_t scrolllock:1;
            uint8_t compose:1;
            uint8_t kana:1;
            uint8_t reserved:3;
    };
    uint8_t leds;
} arduino_usb_hid_keyboard_event_data_t;

// Power Control
#define CONSUMER_CONTROL_POWER                             0x0030
#define CONSUMER_CONTROL_RESET                             0x0031
#define CONSUMER_CONTROL_SLEEP                             0x0032

// Screen Brightness
#define CONSUMER_CONTROL_BRIGHTNESS_INCREMENT              0x006F
#define CONSUMER_CONTROL_BRIGHTNESS_DECREMENT              0x0070

// These HID usages operate only on mobile systems (battery powered) and
// require Windows 8 (build 8302 or greater).
#define CONSUMER_CONTROL_WIRELESS_RADIO_CONTROLS           0x000C
#define CONSUMER_CONTROL_WIRELESS_RADIO_BUTTONS            0x00C6
#define CONSUMER_CONTROL_WIRELESS_RADIO_LED                0x00C7
#define CONSUMER_CONTROL_WIRELESS_RADIO_SLIDER_SWITCH      0x00C8

// Media Control
#define CONSUMER_CONTROL_PLAY_PAUSE                        0x00CD
#define CONSUMER_CONTROL_SCAN_NEXT                         0x00B5
#define CONSUMER_CONTROL_SCAN_PREVIOUS                     0x00B6
#define CONSUMER_CONTROL_STOP                              0x00B7
#define CONSUMER_CONTROL_VOLUME                            0x00E0
#define CONSUMER_CONTROL_MUTE                              0x00E2
#define CONSUMER_CONTROL_BASS                              0x00E3
#define CONSUMER_CONTROL_TREBLE                            0x00E4
#define CONSUMER_CONTROL_BASS_BOOST                        0x00E5
#define CONSUMER_CONTROL_VOLUME_INCREMENT                  0x00E9
#define CONSUMER_CONTROL_VOLUME_DECREMENT                  0x00EA
#define CONSUMER_CONTROL_BASS_INCREMENT                    0x0152
#define CONSUMER_CONTROL_BASS_DECREMENT                    0x0153
#define CONSUMER_CONTROL_TREBLE_INCREMENT                  0x0154
#define CONSUMER_CONTROL_TREBLE_DECREMENT                  0x0155

// Application Launcher
#define CONSUMER_CONTROL_CONFIGURATION                     0x0183
#define CONSUMER_CONTROL_EMAIL_READER                      0x018A
#define CONSUMER_CONTROL_CALCULATOR                        0x0192
#define CONSUMER_CONTROL_LOCAL_BROWSER                     0x0194

// Browser/Explorer Specific
#define CONSUMER_CONTROL_SEARCH                            0x0221
#define CONSUMER_CONTROL_HOME                              0x0223
#define CONSUMER_CONTROL_BACK                              0x0224
#define CONSUMER_CONTROL_FORWARD                           0x0225
#define CONSUMER_CONTROL_BR_STOP                           0x0226
#define CONSUMER_CONTROL_REFRESH                           0x0227
#define CONSUMER_CONTROL_BOOKMARKS                         0x022A

// Mouse Horizontal scroll
#define CONSUMER_CONTROL_PAN                               0x0238

#define CC_BIT_0 CONSUMER_CONTROL_SCAN_NEXT
#define CC_BIT_1 CONSUMER_CONTROL_SCAN_PREVIOUS
#define CC_BIT_2 CONSUMER_CONTROL_STOP
#define CC_BIT_3 CONSUMER_CONTROL_PLAY_PAUSE
#define CC_BIT_4 CONSUMER_CONTROL_MUTE
#define CC_BIT_5 CONSUMER_CONTROL_VOLUME_INCREMENT
#define CC_BIT_6 CONSUMER_CONTROL_VOLUME_DECREMENT
#define CC_BIT_7 CONSUMER_CONTROL_HOME
#define CC_BIT_8 CONSUMER_CONTROL_LOCAL_BROWSER
#define CC_BIT_9 CONSUMER_CONTROL_CALCULATOR
#define CC_BIT_A CONSUMER_CONTROL_BRIGHTNESS_DECREMENT
#define CC_BIT_B CONSUMER_CONTROL_BRIGHTNESS_INCREMENT
#define CC_BIT_C CONSUMER_CONTROL_BR_STOP
#define CC_BIT_D CONSUMER_CONTROL_BACK
#define CC_BIT_E CONSUMER_CONTROL_CONFIGURATION
#define CC_BIT_F CONSUMER_CONTROL_EMAIL_READER

#define KEY_TYPE_MASK        ((uint32_t)0xffff0000UL)
#define KEY_TARGET_MASK      ((uint32_t)0xC0000000UL)

#define KEY_TARGET_KB       ((uint32_t)0x40000000UL)
#define KEY_TARGET_CC       ((uint32_t)0x80000000UL)

#define KEY_TYPE_MODIF      (KEY_TARGET_KB | (uint32_t)0x01000000UL)
#define KEY_TYPE_NONPRINT   (KEY_TARGET_KB | (uint32_t)0x02000000UL)
#define KEY_TYPE_KEYPAD     (KEY_TARGET_KB | (uint32_t)0x04000000UL)
#define SHIFT               (KEY_TARGET_KB | (uint32_t)0x08000000UL)

#define KEY_LEFT_CTRL       (KEY_TYPE_MODIF | (uint32_t)0x01UL)
#define KEY_LEFT_SHIFT      (KEY_TYPE_MODIF | (uint32_t)0x02UL)
#define KEY_LEFT_ALT        (KEY_TYPE_MODIF | (uint32_t)0x04UL)
#define KEY_LEFT_GUI        (KEY_TYPE_MODIF | (uint32_t)0x08UL)
#define KEY_RIGHT_CTRL      (KEY_TYPE_MODIF | (uint32_t)0x10UL)
#define KEY_RIGHT_SHIFT     (KEY_TYPE_MODIF | (uint32_t)0x20UL)
#define KEY_RIGHT_ALT       (KEY_TYPE_MODIF | (uint32_t)0x40UL)
#define KEY_RIGHT_GUI       (KEY_TYPE_MODIF | (uint32_t)0x80UL)

#define KEY_UP_ARROW        (KEY_TYPE_NONPRINT | (uint32_t)0x0052UL)
#define KEY_DOWN_ARROW      (KEY_TYPE_NONPRINT | (uint32_t)0x0051UL)
#define KEY_LEFT_ARROW      (KEY_TYPE_NONPRINT | (uint32_t)0x0050UL)
#define KEY_RIGHT_ARROW     (KEY_TYPE_NONPRINT | (uint32_t)0x004fUL)
#define KEY_BACKSPACE       (KEY_TYPE_NONPRINT | (uint32_t)0x002aUL)
#define KEY_TAB             (KEY_TYPE_NONPRINT | (uint32_t)0x002bUL)
#define KEY_RETURN          (KEY_TYPE_NONPRINT | (uint32_t)0x0028UL)
#define KEY_ESC             (KEY_TYPE_NONPRINT | (uint32_t)0x0029UL)
#define KEY_INSERT          (KEY_TYPE_NONPRINT | (uint32_t)0x0049UL)
#define KEY_PRTSC           (KEY_TYPE_NONPRINT | (uint32_t)0x0046UL)
#define KEY_DELETE          (KEY_TYPE_NONPRINT | (uint32_t)0x004cUL)
#define KEY_PAGE_UP         (KEY_TYPE_NONPRINT | (uint32_t)0x004bUL)
#define KEY_PAGE_DOWN       (KEY_TYPE_NONPRINT | (uint32_t)0x004eUL)
#define KEY_HOME            (KEY_TYPE_NONPRINT | (uint32_t)0x004aUL)
#define KEY_END             (KEY_TYPE_NONPRINT | (uint32_t)0x004dUL)
#define KEY_CAPS_LOCK       (KEY_TYPE_NONPRINT | (uint32_t)0x0039UL)
#define KEY_NUM_LOCK        (KEY_TYPE_NONPRINT | (uint32_t)0x0053UL)
#define KEY_SCROLL_LOCK      (KEY_TYPE_NONPRINT | (uint32_t)0x0047UL)

#define KEY_F1              (KEY_TYPE_NONPRINT | (uint32_t)0x003aUL)
#define KEY_F2              (KEY_TYPE_NONPRINT | (uint32_t)0x003bUL)
#define KEY_F3              (KEY_TYPE_NONPRINT | (uint32_t)0x003cUL)
#define KEY_F4              (KEY_TYPE_NONPRINT | (uint32_t)0x003dUL)
#define KEY_F5              (KEY_TYPE_NONPRINT | (uint32_t)0x003eUL)
#define KEY_F6              (KEY_TYPE_NONPRINT | (uint32_t)0x003fUL)
#define KEY_F7              (KEY_TYPE_NONPRINT | (uint32_t)0x0040UL)
#define KEY_F8              (KEY_TYPE_NONPRINT | (uint32_t)0x0041UL)
#define KEY_F9              (KEY_TYPE_NONPRINT | (uint32_t)0x0042UL)
#define KEY_F10             (KEY_TYPE_NONPRINT | (uint32_t)0x0043UL)
#define KEY_F11             (KEY_TYPE_NONPRINT | (uint32_t)0x0044UL)
#define KEY_F12             (KEY_TYPE_NONPRINT | (uint32_t)0x0045UL)
#define KEY_PRTSC           (KEY_TYPE_NONPRINT | (uint32_t)0x0046UL)
#define KEY_F13             (KEY_TYPE_NONPRINT | (uint32_t)0x0068UL)
#define KEY_F14             (KEY_TYPE_NONPRINT | (uint32_t)0x0069UL)
#define KEY_F15             (KEY_TYPE_NONPRINT | (uint32_t)0x006aUL)
#define KEY_F16             (KEY_TYPE_NONPRINT | (uint32_t)0x006bUL)
#define KEY_F17             (KEY_TYPE_NONPRINT | (uint32_t)0x006cUL)
#define KEY_F18             (KEY_TYPE_NONPRINT | (uint32_t)0x006dUL)
#define KEY_F19             (KEY_TYPE_NONPRINT | (uint32_t)0x006eUL)
#define KEY_F20             (KEY_TYPE_NONPRINT | (uint32_t)0x006fUL)
#define KEY_F21             (KEY_TYPE_NONPRINT | (uint32_t)0x0070UL)
#define KEY_F22             (KEY_TYPE_NONPRINT | (uint32_t)0x0071UL)
#define KEY_F23             (KEY_TYPE_NONPRINT | (uint32_t)0x0072UL)
#define KEY_F24             (KEY_TYPE_NONPRINT | (uint32_t)0x0073UL)

#define KEY_NUM_0           (KEY_TYPE_KEYPAD | (uint32_t)0x0062UL)
#define KEY_NUM_1           (KEY_TYPE_KEYPAD | (uint32_t)0x0059UL)
#define KEY_NUM_2           (KEY_TYPE_KEYPAD | (uint32_t)0x005aUL)
#define KEY_NUM_3           (KEY_TYPE_KEYPAD | (uint32_t)0x005bUL)
#define KEY_NUM_4           (KEY_TYPE_KEYPAD | (uint32_t)0x005cUL)
#define KEY_NUM_5           (KEY_TYPE_KEYPAD | (uint32_t)0x005dUL)
#define KEY_NUM_6           (KEY_TYPE_KEYPAD | (uint32_t)0x005eUL)
#define KEY_NUM_7           (KEY_TYPE_KEYPAD | (uint32_t)0x005fUL)
#define KEY_NUM_8           (KEY_TYPE_KEYPAD | (uint32_t)0x0060UL)
#define KEY_NUM_9           (KEY_TYPE_KEYPAD | (uint32_t)0x0061UL)
#define KEY_NUM_SLASH       (KEY_TYPE_KEYPAD | (uint32_t)0x0054UL)
#define KEY_NUM_ASTERISK    (KEY_TYPE_KEYPAD | (uint32_t)0x0055UL)
#define KEY_NUM_MINUS       (KEY_TYPE_KEYPAD | (uint32_t)0x0056UL)
#define KEY_NUM_PLUS        (KEY_TYPE_KEYPAD | (uint32_t)0x0057UL)
#define KEY_NUM_ENTER       (KEY_TYPE_KEYPAD | (uint32_t)0x0058UL)
#define KEY_NUM_PERIOD      (KEY_TYPE_KEYPAD | (uint32_t)0x0063UL)
#define KEY_NUM_EQUAL       (KEY_TYPE_KEYPAD | (uint32_t)0x0067UL)


typedef uint32_t MediaKeyReport;

#define KEY_MEDIA_NEXT              (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x0U))
#define KEY_MEDIA_PREV              (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x1U))
#define KEY_MEDIA_STOP              (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x2U))
#define KEY_MEDIA_PAUSE             (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x3U))
#define KEY_MEDIA_MUTE              (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x4U))
#define KEY_MEDIA_VOLUP             (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x5U))
#define KEY_MEDIA_VOLDN             (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x6U))
#define KEY_MEDIA_WWW_HOME          (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x7U))

#define KEY_FILE_EXPLORER           (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x8U))
#define KEY_MEDIA_CALC              (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0x9U))
#define KEY_MEDIA_BRIGHTNESS_D      (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xAU))
#define KEY_MEDIA_BRIGHTNESS_I      (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xBU))
#define KEY_MEDIA_WWW_STOP          (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xCU))
#define KEY_MEDIA_WWW_BACK          (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xDU))
#define KEY_CONSUMER_CTL            (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xEU))
#define KEY_MEDIA_EMAIL             (KEY_TARGET_CC | (MediaKeyReport) (1UL << 0xFU))

#define STR_LEFT_CTRL "\x80"
#define STR_LEFT_SHIFT "\x81"
#define STR_LEFT_ALT "\x82"
#define STR_LEFT_GUI "\x83"
#define STR_RIGHT_CTRL "\x84"
#define STR_RIGHT_SHIFT "\x85"
#define STR_RIGHT_ALT "\x86"
#define STR_RIGHT_GUI "\x87"
#define STR_UP_ARROW "\x88"
#define STR_DOWN_ARROW "\x89"
#define STR_LEFT_ARROW "\x8a"
#define STR_RIGHT_ARROW "\x8b"
#define STR_BACKSPACE "\x8c"
#define STR_TAB "\x8d"
#define STR_RETURN "\x8e"
#define STR_ESC "\x8f"
#define STR_INSERT "\x90"
#define STR_PRTSC "\x91"
#define STR_DELETE "\x92"
#define STR_PAGE_UP "\x93"
#define STR_PAGE_DOWN "\x94"
#define STR_HOME "\x95"
#define STR_END "\x96"
#define STR_CAPS_LOCK "\x97"
#define STR_NUM_LOCK "\x98"
#define STR_SCROLL_LOCK "\x99"

#define STR_F1 "\x9a"
#define STR_F2 "\x9b" 
#define STR_F3 "\x9c" 
#define STR_F4 "\x9d" 
#define STR_F5 "\x9e" 
#define STR_F6 "\x9f" 
#define STR_F7 "\xa0" 
#define STR_F8 "\xa1" 
#define STR_F9 "\xa2" 
#define STR_F10 "\xa3" 
#define STR_F11 "\xa4" 
#define STR_F12 "\xa5" 
#define STR_F13 "\xa6" 
#define STR_F14 "\xa7" 
#define STR_F15 "\xa8" 
#define STR_F16 "\xa9" 
#define STR_F17 "\xaa" 
#define STR_F18 "\xab" 
#define STR_F19 "\xac" 
#define STR_F20 "\xad" 
#define STR_F21 "\xae" 
#define STR_F22 "\xaf" 
#define STR_F23 "\xb0" 
#define STR_F24 "\xb1" 
#define STR_NUM_0 "\xb2" 
#define STR_NUM_1 "\xb3" 
#define STR_NUM_2 "\xb4" 
#define STR_NUM_3 "\xb5" 
#define STR_NUM_4 "\xb6" 
#define STR_NUM_5 "\xb7" 
#define STR_NUM_6 "\xb8" 
#define STR_NUM_7 "\xb9" 
#define STR_NUM_8 "\xba" 
#define STR_NUM_9 "\xbb" 
#define STR_NUM_SLASH "\xbc" 
#define STR_NUM_ASTERISK "\xbd" 
#define STR_NUM_MINUS "\xbe" 
#define STR_NUM_PLUS "\xbf" 
#define STR_NUM_ENTER "\xc0" 
#define STR_NUM_PERIOD "\xc1" 
#define STR_NUM_EQUAL "\xc2" 
#define STR_MEDIA_NEXT "\xc3" 
#define STR_MEDIA_PREV "\xc4" 
#define STR_MEDIA_STOP "\xc5" 
#define STR_MEDIA_PAUSE "\xc6" 
#define STR_MEDIA_MUTE "\xc7" 
#define STR_MEDIA_VOLUP "\xc8" 
#define STR_MEDIA_VOLDN "\xc9" 
#define STR_MEDIA_WWW_HOME "\xca" 
#define STR_FILE_EXPLORER "\xcb" 
#define STR_MEDIA_CALC "\xcc" 
#define STR_MEDIA_BRIGHTNESS_D "\xcd" 
#define STR_MEDIA_BRIGHTNESS_I "\xce" 
#define STR_MEDIA_WWW_STOP "\xcf" 
#define STR_MEDIA_WWW_BACK "\xd0" 
#define STR_CONSUMER_CTL "\xd1" 
#define STR_MEDIA_EMAIL "\xd2" 

#define LED_NUMLOCK     0x01
#define LED_CAPSLOCK    0x02
#define LED_SCROLLLOCK  0x04
#define LED_COMPOSE     0x08
#define LED_KANA        0x10

//  Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct
{
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t keys[61];
} KeyReport;

class Reports {
protected:
    KeyReport _keyReport;
    uint16_t _cc_report;
public:
    Reports() : _cc_report(0) {
        memset(&_keyReport, 0, sizeof(_keyReport));
    }
    const KeyReport& getKeyReport() const {return _keyReport;}
    const uint16_t& getCCReport() const {return _cc_report;}
    uint32_t pressRaw(uint32_t k);
    uint32_t releaseRaw(uint32_t k);
    uint32_t press(uint8_t data);
    uint32_t release(uint8_t data);
    uint32_t pressMulti(const uint8_t* data);
    uint32_t releaseMulti(const uint8_t* data);
    void releaseAll();
};

class HIDKeyboard: public USBHIDDevice, public Reports
{
private:
    USBHID hid;
    uint8_t _target = 1;

    void (*output_cb)(arduino_usb_hid_keyboard_event_data_t* p);
    void (*esp_now_send_report)(uint8_t report_id, const void *data, size_t len);

public:
    HIDKeyboard(void);
    void begin(void);
    void end(void);
    void sendKeyReport();
    void sendMediaReport();
    void target(uint8_t target);
    uint8_t target();
    void sendReport(uint16_t value);
    void sendReport(KeyReport* keys);

    void register_output_callback(
        void (*_output_cb)(arduino_usb_hid_keyboard_event_data_t* p)) {
        output_cb = _output_cb;
    }
    void unregister_output_callback() { output_cb = NULL; }

    void register_espnow_report(
        void (*espnow_report_fn)(uint8_t report_id,
                                 const void* data,
                                 size_t len)) {
        esp_now_send_report = espnow_report_fn;
    }
    void unregister_espnow_report() {esp_now_send_report = NULL; }

    void onEvent(esp_event_handler_t callback);
    void onEvent(arduino_usb_hid_keyboard_event_t event, esp_event_handler_t callback);

    // internal use
    uint16_t _onGetDescriptor(uint8_t* buffer);
    void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len);
};

#endif
