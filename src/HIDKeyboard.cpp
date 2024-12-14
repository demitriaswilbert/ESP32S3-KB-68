/*
  Keyboard.cpp

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
#include "USBHID.h"
#include "esp32-hal-log.h"

#if CONFIG_TINYUSB_HID_ENABLED

#include "HIDKeyboard.h"

ESP_EVENT_DEFINE_BASE(ARDUINO_USB_HID_KEYBOARD_EVENTS);
esp_err_t arduino_usb_event_post(esp_event_base_t event_base, int32_t event_id, void *event_data, size_t event_data_size, TickType_t ticks_to_wait);
esp_err_t arduino_usb_event_handler_register_with(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler, void *event_handler_arg);

const uint8_t report_descriptor[] = {
    // TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_REPORT_ID_KEYBOARD)),
    HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                    ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_KEYBOARD )                    ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                    ,\
    /* Report ID if any */\
    HID_REPORT_ID(HID_REPORT_ID_KEYBOARD) \
    /* 8 bits Modifier Keys (Shfit, Control, Alt) */ \
    HID_USAGE_PAGE ( HID_USAGE_PAGE_KEYBOARD )                     ,\
      HID_USAGE_MIN    ( 224                                    )  ,\
      HID_USAGE_MAX    ( 231                                    )  ,\
      HID_LOGICAL_MIN  ( 0                                      )  ,\
      HID_LOGICAL_MAX  ( 1                                      )  ,\
      HID_REPORT_COUNT ( 8                                      )  ,\
      HID_REPORT_SIZE  ( 1                                      )  ,\
      HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE )  ,\
      /* 8 bit reserved */ \
      HID_REPORT_COUNT ( 1                                      )  ,\
      HID_REPORT_SIZE  ( 8                                      )  ,\
      HID_INPUT        ( HID_CONSTANT                           )  ,\
    /* Output 5-bit LED Indicator Kana | Compose | ScrollLock | CapsLock | NumLock */ \
    HID_USAGE_PAGE  ( HID_USAGE_PAGE_LED                   )       ,\
      HID_USAGE_MIN    ( 1                                       ) ,\
      HID_USAGE_MAX    ( 5                                       ) ,\
      HID_REPORT_COUNT ( 5                                       ) ,\
      HID_REPORT_SIZE  ( 1                                       ) ,\
      HID_OUTPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ) ,\
      /* led padding */ \
      HID_REPORT_COUNT ( 1                                       ) ,\
      HID_REPORT_SIZE  ( 3                                       ) ,\
      HID_OUTPUT       ( HID_CONSTANT                            ) ,\
    /* 6-byte Keycodes */ \
    HID_USAGE_PAGE ( HID_USAGE_PAGE_KEYBOARD )                     ,\
      HID_USAGE_MIN    ( 0                                   )     ,\
      HID_USAGE_MAX_N  ( 255, 2                              )     ,\
      HID_LOGICAL_MIN  ( 0                                   )     ,\
      HID_LOGICAL_MAX_N( 255, 2                              )     ,\
      HID_REPORT_COUNT ( 61                                  )     ,/* 6 */\
      HID_REPORT_SIZE  ( 8                                   )     ,/* 8 */\
      HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE )     ,\
  HID_COLLECTION_END, \
    HID_USAGE_PAGE(HID_USAGE_PAGE_CONSUMER),         // USAGE_PAGE (Consumer)
    HID_USAGE(HID_USAGE_CONSUMER_CONTROL ),          // USAGE (Consumer Control)
    HID_COLLECTION(HID_COLLECTION_APPLICATION),      // COLLECTION (Application)
    HID_REPORT_ID(HID_REPORT_ID_CONSUMER_CONTROL)    //   REPORT_ID (4)
    HID_LOGICAL_MIN(0x00),                           //   LOGICAL_MINIMUM (0)
    HID_LOGICAL_MAX(0x01),                           //   LOGICAL_MAXIMUM (1)
    HID_REPORT_SIZE(0x01),                           //   REPORT_SIZE (1)
    HID_REPORT_COUNT(0x10),                          //   REPORT_COUNT (16)
    HID_USAGE_N(CC_BIT_0, 2),              //   USAGE (Scan Next Track)     ; bit 0: 1
    HID_USAGE_N(CC_BIT_1, 2),              //   USAGE (Scan Previous Track) ; bit 1: 2
    HID_USAGE_N(CC_BIT_2, 2),              //   USAGE (Stop)                ; bit 2: 4
    HID_USAGE_N(CC_BIT_3, 2),              //   USAGE (Play/Pause)          ; bit 3: 8
    HID_USAGE_N(CC_BIT_4, 2),              //   USAGE (Mute)                ; bit 4: 16
    HID_USAGE_N(CC_BIT_5, 2),              //   USAGE (Volume Increment)    ; bit 5: 32
    HID_USAGE_N(CC_BIT_6, 2),              //   USAGE (Volume Decrement)    ; bit 6: 64
    HID_USAGE_N(CC_BIT_7, 2),        //   Usage (WWW Home)            ; bit 7: 128
    HID_USAGE_N(CC_BIT_8, 2),        //   Usage (My Computer) ; bit 0: 1
    HID_USAGE_N(CC_BIT_9, 2),        //   Usage (Calculator)  ; bit 1: 2
    HID_USAGE_N(CC_BIT_A, 2),        //   Usage (WWW fav)     ; bit 2: 4
    HID_USAGE_N(CC_BIT_B, 2),        //   Usage (WWW search)  ; bit 3: 8
    HID_USAGE_N(CC_BIT_C, 2),        //   Usage (WWW stop)    ; bit 4: 16
    HID_USAGE_N(CC_BIT_D, 2),        //   Usage (WWW back)    ; bit 5: 32
    HID_USAGE_N(CC_BIT_E, 2),        //   Usage (Media sel)   ; bit 6: 64
    HID_USAGE_N(CC_BIT_F, 2),        //   Usage (Mail)        ; bit 7: 128
    HID_INPUT(0x02),           //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    HID_COLLECTION_END            // END_COLLECTION
};

const size_t report_descriptor_size = sizeof(report_descriptor);

const uint32_t _asciimap[] =
{
    0x00U, // NUL
    0x00U, // SOH
    0x00U, // STX
    0x00U, // ETX
    0x00U, // EOT
    0x00U, // ENQ
    0x00U, // ACK
    0x00U, // BEL
    0x2aU, // BS	Backspace
    0x2bU, // TAB	Tab
    0x28U, // LF	Enter
    0x00U, // VT
    0x00U, // FF
    0x00U, // CR
    0x00U, // SO
    0x00U, // SI
    0x00U, // DEL
    0x00U, // DC1
    0x00U, // DC2
    0x00U, // DC3
    0x00U, // DC4
    0x00U, // NAK
    0x00U, // SYN
    0x00U, // ETB
    0x00U, // CAN
    0x00U, // EM
    0x00U, // SUB
    0x00U, // ESC
    0x00U, // FS
    0x00U, // GS
    0x00U, // RS
    0x00U, // US
    0x2cU,         //  ' '
    0x1eU | SHIFT, // !
    0x34U | SHIFT, // "
    0x20U | SHIFT, // #
    0x21U | SHIFT, // $
    0x22U | SHIFT, // %
    0x24U | SHIFT, // &
    0x34U,         // '
    0x26U | SHIFT, // (
    0x27U | SHIFT, // )
    0x25U | SHIFT, // *
    0x2eU | SHIFT, // +
    0x36U,         // ,
    0x2dU,         // -
    0x37U,         // .
    0x38U,         // /
    0x27U,         // 0
    0x1eU,         // 1
    0x1fU,         // 2
    0x20U,         // 3
    0x21U,         // 4
    0x22U,         // 5
    0x23U,         // 6
    0x24U,         // 7
    0x25U,         // 8
    0x26U,         // 9
    0x33U | SHIFT, // :
    0x33U,         // ;
    0x36U | SHIFT, // <
    0x2eU,         // =
    0x37U | SHIFT, // >
    0x38U | SHIFT, // ?
    0x1fU | SHIFT, // @
    0x04U | SHIFT, // A
    0x05U | SHIFT, // B
    0x06U | SHIFT, // C
    0x07U | SHIFT, // D
    0x08U | SHIFT, // E
    0x09U | SHIFT, // F
    0x0aU | SHIFT, // G
    0x0bU | SHIFT, // H
    0x0cU | SHIFT, // I
    0x0dU | SHIFT, // J
    0x0eU | SHIFT, // K
    0x0fU | SHIFT, // L
    0x10U | SHIFT, // M
    0x11U | SHIFT, // N
    0x12U | SHIFT, // O
    0x13U | SHIFT, // P
    0x14U | SHIFT, // Q
    0x15U | SHIFT, // R
    0x16U | SHIFT, // S
    0x17U | SHIFT, // T
    0x18U | SHIFT, // U
    0x19U | SHIFT, // V
    0x1aU | SHIFT, // W
    0x1bU | SHIFT, // X
    0x1cU | SHIFT, // Y
    0x1dU | SHIFT, // Z
    0x2fU,         // [
    0x31U,         // bslash
    0x30U,         // ]
    0x23U | SHIFT, // ^
    0x2dU | SHIFT, // _
    0x35U,         // `
    0x04U,         // a
    0x05U,         // b
    0x06U,         // c
    0x07U,         // d
    0x08U,         // e
    0x09U,         // f
    0x0aU,         // g
    0x0bU,         // h
    0x0cU,         // i
    0x0dU,         // j
    0x0eU,         // k
    0x0fU,         // l
    0x10U,         // m
    0x11U,         // n
    0x12U,         // o
    0x13U,         // p
    0x14U,         // q
    0x15U,         // r
    0x16U,         // s
    0x17U,         // t
    0x18U,         // u
    0x19U,         // v
    0x1aU,         // w
    0x1bU,         // x
    0x1cU,         // y
    0x1dU,         // z
    0x2fU | SHIFT, // {
    0x31U | SHIFT, // |
    0x30U | SHIFT, // }
    0x35U | SHIFT, // ~
    0x00U,         // DEL
    // MODIFIER
    KEY_LEFT_CTRL,
    KEY_LEFT_SHIFT,
    KEY_LEFT_ALT, 
    KEY_LEFT_GUI, 
    KEY_RIGHT_CTRL, 
    KEY_RIGHT_SHIFT, 
    KEY_RIGHT_ALT, 
    KEY_RIGHT_GUI, 
    // NONPRINT
    KEY_UP_ARROW,
    KEY_DOWN_ARROW,
    KEY_LEFT_ARROW,
    KEY_RIGHT_ARROW,
    KEY_BACKSPACE,
    KEY_TAB,
    KEY_RETURN,
    KEY_ESC,
    KEY_INSERT,
    KEY_PRTSC,
    KEY_DELETE,
    KEY_PAGE_UP,
    KEY_PAGE_DOWN,
    KEY_HOME,
    KEY_END,
    KEY_CAPS_LOCK,
    KEY_NUM_LOCK,
    KEY_SCROLL_LOCK,
    
    // F Keys
    KEY_F1, 
    KEY_F2, 
    KEY_F3, 
    KEY_F4, 
    KEY_F5, 
    KEY_F6, 
    KEY_F7, 
    KEY_F8, 
    KEY_F9, 
    KEY_F10, 
    KEY_F11, 
    KEY_F12, 
    KEY_F13, 
    KEY_F14, 
    KEY_F15, 
    KEY_F16, 
    KEY_F17, 
    KEY_F18, 
    KEY_F19, 
    KEY_F20, 
    KEY_F21, 
    KEY_F22, 
    KEY_F23, 
    KEY_F24, 

    // KEYPAD
    KEY_NUM_0,
    KEY_NUM_1,
    KEY_NUM_2,
    KEY_NUM_3,
    KEY_NUM_4,
    KEY_NUM_5,
    KEY_NUM_6,
    KEY_NUM_7,
    KEY_NUM_8,
    KEY_NUM_9,
    KEY_NUM_SLASH,
    KEY_NUM_ASTERISK,
    KEY_NUM_MINUS,
    KEY_NUM_PLUS,
    KEY_NUM_ENTER,
    KEY_NUM_PERIOD,
    KEY_NUM_EQUAL,

    // MEDIA 1
    KEY_MEDIA_NEXT,
    KEY_MEDIA_PREV, 
    KEY_MEDIA_STOP, 
    KEY_MEDIA_PAUSE, 
    KEY_MEDIA_MUTE, 
    KEY_MEDIA_VOLUP, 
    KEY_MEDIA_VOLDN, 
    KEY_MEDIA_WWW_HOME, 
    
    // MEDIA 2
    KEY_FILE_EXPLORER, 
    KEY_MEDIA_CALC, 
    KEY_MEDIA_BRIGHTNESS_D, 
    KEY_MEDIA_BRIGHTNESS_I, 
    KEY_MEDIA_WWW_STOP, 
    KEY_MEDIA_WWW_BACK, 
    KEY_CONSUMER_CTL, 
    KEY_MEDIA_EMAIL, 
};

HIDKeyboard::HIDKeyboard(): hid(), _target(1), esp_now_send_report(NULL), output_cb(NULL) {
    static bool initialized = false;
    if(!initialized){
        initialized = true;
        hid.addDevice(this, sizeof(report_descriptor));
    }
}

uint16_t HIDKeyboard::_onGetDescriptor(uint8_t* dst){
    memcpy(dst, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
}

void HIDKeyboard::begin(){
    hid.begin();
}

void HIDKeyboard::end(){
    hid.end();
}

void HIDKeyboard::target(uint8_t target) {
    if (target == 0) target = 1;
    if (target > 3) target = 3;
    _target = target;
}

uint8_t HIDKeyboard::target() {
    return _target;
}

void HIDKeyboard::onEvent(esp_event_handler_t callback){
    onEvent(ARDUINO_USB_HID_KEYBOARD_ANY_EVENT, callback);
}
void HIDKeyboard::onEvent(arduino_usb_hid_keyboard_event_t event, esp_event_handler_t callback){
    arduino_usb_event_handler_register_with(ARDUINO_USB_HID_KEYBOARD_EVENTS, event, callback, this);
}


void HIDKeyboard::_onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len){
    if(report_id == HID_REPORT_ID_KEYBOARD){
        arduino_usb_hid_keyboard_event_data_t p;
        p.leds = buffer[0];
        if (output_cb != NULL) output_cb(&p);
        arduino_usb_event_post(ARDUINO_USB_HID_KEYBOARD_EVENTS, ARDUINO_USB_HID_KEYBOARD_LED_EVENT, &p, sizeof(arduino_usb_hid_keyboard_event_data_t), portMAX_DELAY);
    }
}

void HIDKeyboard::sendKeyReport() {
    sendReport(&_keyReport);
}

void HIDKeyboard::sendReport(KeyReport* keys)
{
    // hid_keyboard_report_t report;
    // report.reserved = 0;
    // report.modifier = keys->modifiers;
    // if (keys->keys) {
    //     memcpy(report.keycode, keys->keys, sizeof(keys->keys));
    // } else {
    //     memset(report.keycode, 0, sizeof(keys->keys));
    // }
    if (hid.ready() && (_target & 1))
        hid.SendReport(HID_REPORT_ID_KEYBOARD, keys, sizeof(*keys));
    if ((!hid.ready() || (_target & 2)) && esp_now_send_report != NULL) {
        esp_now_send_report(HID_REPORT_ID_KEYBOARD, keys, sizeof(*keys));
    }
}

void HIDKeyboard::sendReport(uint16_t value){
    if (hid.ready() && (_target & 1))
        hid.SendReport(HID_REPORT_ID_CONSUMER_CONTROL, &value, sizeof(value));
    if ((!hid.ready() || (_target & 2)) && esp_now_send_report != NULL) {
        esp_now_send_report(HID_REPORT_ID_CONSUMER_CONTROL, &value, sizeof(value));
    }
}

uint32_t Reports::pressRaw(uint32_t k) {
    uint8_t i; uint32_t retVal = k;
    if ((k & KEY_TYPE_MASK) == KEY_TYPE_MODIF)
    { // it's a modifier key
        _keyReport.modifiers |= (k & 0xff);
        return retVal;
    }
    else if ((k & KEY_TYPE_MASK) == 0x0U)
    { // ascii
        k = _asciimap[k & 0xffU] | KEY_TARGET_KB;
        if ((k & KEY_TYPE_MASK) == SHIFT)
            _keyReport.modifiers |= (KEY_LEFT_SHIFT & 0xff);
    }
    else if ((k & KEY_TYPE_MASK) == KEY_TARGET_CC)
    { // media
        _cc_report |= (k & 0xFFFFU);
        return k;
    }
    else if (!k)
        return 0;

    for (i = 0; i < sizeof(_keyReport.keys); i++)
    {
        if (_keyReport.keys[i] == 0x00)
        {
            _keyReport.keys[i] = (uint8_t)k;
            return k;
        }
    }
    return 0;
}

uint32_t Reports::press(uint8_t data) {
    uint32_t k = (data > 127)? _asciimap[data] : data;
    return pressRaw(k);
}

uint32_t Reports::release(uint8_t data) {
    uint32_t k = (data > 127)? _asciimap[data] : data;
    return releaseRaw(k);
}

uint32_t Reports::pressMulti(const uint8_t* data) {
    if (data == NULL) return 0;
    int i = 0;
    uint32_t retval = 0;
    while (data[i]) {
        retval |= press((const uint8_t)data[i]) & KEY_TARGET_MASK;
        i++;
    }
    return retval;
}

uint32_t Reports::releaseMulti(const uint8_t* data) {
    if (data == NULL) return 0;
    int i = 0;
    uint32_t retval = 0;
    while (data[i]) {
        retval |= release((const uint8_t)data[i]) & KEY_TARGET_MASK;
        i++;
    }
    return retval;
}

uint32_t Reports::releaseRaw(uint32_t k) {
    uint8_t i; uint32_t retVal = k;
    if ((k & KEY_TYPE_MASK) == KEY_TYPE_MODIF)
    { // it's a modifier key
        _keyReport.modifiers &= ~(k & 0xff);
        return retVal;
    }
    else if ((k & KEY_TYPE_MASK) == 0x0U)
    { // ascii
        k = _asciimap[k & 0xffU] | KEY_TARGET_KB;
        if ((k & KEY_TYPE_MASK) == SHIFT)
            _keyReport.modifiers &= ~(KEY_LEFT_SHIFT & 0xff);
    }
    else if ((k & KEY_TYPE_MASK) == KEY_TARGET_CC)
    { // media
        _cc_report &= ~(k & 0xFFFFU);
        return k;
    }
    else if (!k)
        return 0;

    for (i = 0; i < sizeof(_keyReport.keys); i++)
    {
        if (_keyReport.keys[i] == (uint8_t)k)
        {
            _keyReport.keys[i] = 0;
            return k;
        }
    }
    return 0;
}

void Reports::releaseAll(void)
{
    memset(&_keyReport, 0, sizeof(_keyReport));
    _cc_report = 0;
}


void HIDKeyboard::sendMediaReport() {
    return sendReport(_cc_report);
}

#endif /* CONFIG_TINYUSB_HID_ENABLED */
