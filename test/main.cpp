#include <Arduino.h>
#include <WiFi.h>
#include "USB.h"
#include <vector>
#include <deque>
#include "HIDKeyboard.h"
#include "USBCDC.h"
#include <esp_now.h>
#include "esp_private/wifi.h"
#include "esp32-hal-tinyusb.h"
#include <SPIFFS.h>
#include "esp32-hal-log.h"
#include "EXTString.h"
#include "filesystem.h"
extern "C" {
    #include "Enigma.h"
}

// #define PEER_MAC_ADDR "FC:F5:C4:01:00:74"
// #define PEER_MAC_ADDR "24:0A:C4:59:D3:7C"
#define PEER_MAC_ADDR_BT "24:0A:C4:59:D3:7C"
#define PEER_MAC_ADDR "84:F7:03:EA:F8:D0"
// #define PEER_MAC_ADDR "84:F7:03:EA:F1:0A"

#define ERR_CHECK(x) {\
    static char err_buf[0x100U]; esp_err_t errcode = (x);\
    if (errcode != ESP_OK) {\
        esp_err_to_name_r(errcode, err_buf, sizeof(err_buf));\
        log_cdc("[ERROR] line %d : %s", __LINE__, err_buf);\
    }\
}

static inline __attribute__((always_inline))
bool directRead(int pin) {
    if ( pin < 32 )
        return !!(GPIO.in & ((uint32_t)1 << pin));
    else 
        return !!(GPIO.in1.val & ((uint32_t)1 << (pin - 32)));
}


static inline __attribute__((always_inline))
void directWriteLow(int pin) {
    if ( pin < 32 )
        GPIO.out_w1tc = ((uint32_t)1 << pin);
    else if (pin < 54)
        GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
}

static inline __attribute__((always_inline))
void directWriteHigh(int pin) {
    if ( pin < 32 )
        GPIO.out_w1ts = ((uint32_t)1 << pin);
    else if (pin < 54)
        GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
}

static inline __attribute__((always_inline))
void directWrite(int pin, bool state) {
    if (state)
        directWriteHigh(pin);
    else directWriteLow(pin);
}

bool is_espnow_on = false;
bool is_kb_on = false;
static char espnow_buf[0x1000U] = {0};

typedef struct {
    uint8_t* buf;
    size_t size;
} buf_len_t;

static QueueHandle_t esp_now_message_queue = NULL;
static SemaphoreHandle_t esp_now_message_mutex = NULL;

arduino_usb_hid_keyboard_event_data_t p_led_data = {0};
const gpio_num_t capslock_pin = (gpio_num_t)3;
const gpio_num_t numlock_pin = (gpio_num_t)2;

#define log_cdc(format, ...) usb_cdc.printf(format "\r\n", ##__VA_ARGS__)
#define log_esp(format, ...) {\
    static buf_len_t tmp; \
    tmp.size = sprintf(espnow_buf, format "\r\n", ##__VA_ARGS__); \
    tmp.buf = (uint8_t*) malloc(tmp.size); \
    memcpy(tmp.buf, espnow_buf, tmp.size); \
    if (xQueueSend(esp_now_message_queue, &tmp, 100) != pdTRUE) \
        free(tmp.buf); \
} 
#define millisll() (esp_timer_get_time() / 1000ULL)

#define SLEEP_TIMEOUT ((int64_t)300000LL)
static int64_t sleep_timeout = SLEEP_TIMEOUT;

static uint32_t tasks_running = 0UL;
static bool tasks_run = true;

static uint16_t kb_delay = 0U;
static HIDKeyboard* kb_dev = NULL;

static SemaphoreHandle_t kb_target_changed_sem = NULL;
static QueueHandle_t kb_target_change = NULL;

static USBCDC usb_cdc(0);

#define PMK_STRING "Never Gonna Give"
#define LMK_STRING "You Up Never Gon"

static bool tx_lock = false;
static EXTString tx_str = "";

static uint32_t type_pos = 0UL;
static EXTString rx_str = "";

static QueueHandle_t sent_queue = NULL;
static esp_now_peer_info_t* masterInfo;
static esp_now_peer_info_t usb_peer_info;
static esp_now_peer_info_t bt_peer_info;

static Rotor rotor;

/**
 * @brief feed sleep timeout
 * 
 * @param timeout in ms
 */
static inline void feed_timeout(int64_t timeout = SLEEP_TIMEOUT) { 
    sleep_timeout = millisll() + timeout;
}

/**
 * @brief usb cdc event handler
 * 
 * @param event_handler_arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
void cdc_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
}

/**
 * @brief keyboard output led handler
 * 
 * @param led_data 
 */
void kb_led_handler(arduino_usb_hid_keyboard_event_data_t led_data) {
    char buf[1000];
    p_led_data.leds = (led_data.leds & 0x1fU) | (p_led_data.leds & 0xe0U);
    directWrite(numlock_pin, p_led_data.numlock);
    directWrite(capslock_pin, p_led_data.capslock);
}

/**
 * @brief hid keyboard event handler
 * 
 * @param event_handler_arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
void kb_output(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    kb_led_handler(*(arduino_usb_hid_keyboard_event_data_t*) event_data);
}

SemaphoreHandle_t esp_now_send_packet_mtx = NULL;
SemaphoreHandle_t esp_now_switch_mtx = NULL;
/**
 * @brief wrapper function for esp_now_send that 
 * resends the message until the message is successfully sent or timeout is reached
 * 
 * @param peer_addr target peer address
 * @param data data
 * @param len length of data
 * @param timeout timeout
 * @return esp_err_t 
 */
esp_err_t esp_now_send_packet(
    const uint8_t* peer_addr, const uint8_t* data, size_t len, uint32_t timeout) {
    
    if (xSemaphoreTake(esp_now_send_packet_mtx, 0xfffffffful) != pdTRUE)
        return ESP_FAIL;

    int64_t time = millisll();

    esp_err_t err_send_fn;
    do {
        esp_now_send(peer_addr, data, len);
        taskYIELD();
        if (xQueueReceive(sent_queue, &err_send_fn, 5000) != pdTRUE) 
            err_send_fn = ESP_FAIL;
    } while ((err_send_fn != ESP_OK) && (millisll() - time) < timeout);

    GPIO.out ^= (1UL << LED_BUILTIN);

    xSemaphoreGive(esp_now_send_packet_mtx);

    return err_send_fn;
}
/**
 * @brief esp-now sent report callback
 * 
 * @param mac target mac address
 * @param status transmission status
 */
void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    esp_err_t sent_status = (status == ESP_NOW_SEND_SUCCESS)? ESP_OK : ESP_FAIL;

    if (xPortInIsrContext()) {
        xQueueSendFromISR(sent_queue, &sent_status, &pxHigherPriorityTaskWoken);
        if (pxHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    } else xQueueSend(sent_queue, &sent_status, 1000);
}

/**
 * @brief Get a Mac Address array from a string
 *
 * Converts std::string "24:0a:C4:59:D3:7C"
 * to array uint8_t[6] {0x24, 0x0A, 0xC4, 0x59, 0xD3, 0x7C}
 *
 * @param str mac address string
 * @param buf_mac_addr a uint8_t[6] buffer
 * @return true if successful
 * @return false if format incorrect, original mac address remains unchanged
 */
bool getMacAddr(const char* str, uint8_t* buf_mac_addr) {
    uint8_t mac_addr_tmp[6] = {0}, mac_addr_filled = 0;
    for (const char* i = str; *i != '\0' && mac_addr_filled < 12U; i++) {
        const char ch = *i;
        if (ch == ':') continue;
        uint8_t tmpval = 0U;
        if (ch >= '0' && ch <= '9') {
            tmpval = (uint8_t)(ch - '0');
        } else if (ch >= 'a' && ch <= 'f') {
            tmpval = (uint8_t)(ch - 'a' + 10);
        } else if (ch >= 'A' && ch <= 'F') {
            tmpval = (uint8_t)(ch - 'A' + 10);
        }
        tmpval &= 0x0fU;
        tmpval <<= (4U * (!(mac_addr_filled & 1U)));
        mac_addr_tmp[mac_addr_filled / 2U] |= tmpval;
        mac_addr_filled++;
    }
    if (mac_addr_filled != 12U) return false;
    memcpy(buf_mac_addr, mac_addr_tmp, 6U);
    return true;
}

void setMacAddr(const char* addr, esp_now_peer_info_t* peerInfo) {
    uint8_t tmp_addr[6];
    bool success = getMacAddr(addr, tmp_addr);
    if (!success) return;
    if (xSemaphoreTake(esp_now_switch_mtx, 0xfffffffful) == pdTRUE) {
        memcpy(peerInfo->peer_addr, tmp_addr, 6);
        if (!is_espnow_on) {
            xSemaphoreGive(esp_now_switch_mtx);
            return;
        }
        esp_err_t err = esp_now_mod_peer(peerInfo);
        xSemaphoreGive(esp_now_switch_mtx);
    }
    
}

String getMacString(const uint8_t* buf) {
    String tmp;
    const char symbols[] = "0123456789ABCDEF";
    for (int i = 0; i < 6; i++) {
        if (i > 0) tmp += ':';
        tmp += symbols[(buf[i] >> 4U) & 0x0fU];
        tmp += symbols[buf[i] & 0x0fU];
    }
    return tmp;
}

/**
 * @brief send keyboard report via esp_now
 * 
 * @param id report id
 * @param buf report data
 * @param len report length
 */
void ESP_NOW_SendReport(uint8_t id, const void* buf, size_t len) {
    
    if (xSemaphoreTake(esp_now_switch_mtx, 100) != pdTRUE)
        return;
        
    if (!is_espnow_on) {
        xSemaphoreGive(esp_now_switch_mtx);
        return;
    } 

    uint8_t data[0x100U];
    *(uint32_t*)&data[0] = 0x20;
    data[4] = id;
    memcpy(&data[5], buf, len);

    esp_now_send_packet(masterInfo->peer_addr, &data[0], len + 1 + 4, 500);

    xSemaphoreGive(esp_now_switch_mtx);
}


esp_err_t esp_now_send_blocking(
            const uint8_t* peer_addr, 
            const uint8_t* data, 
            size_t len, 
            size_t* transmitted, 
            uint32_t timeout) {

    
    if (xSemaphoreTake(esp_now_switch_mtx, 0xfffffffful) != pdTRUE)
        return ESP_ERR_ESPNOW_NOT_INIT;

    if (!is_espnow_on) {
        xSemaphoreGive(esp_now_switch_mtx);
        return ESP_ERR_ESPNOW_NOT_INIT;
    } 
    
    *transmitted = 0;
    size_t i = 0;
    int64_t resend_timeout = millisll();

    esp_err_t status = ESP_OK;

    for(; i < len;) {

        int txLen = ((len - i) > 242)? 242 : (len - i);

        uint8_t packet[250] = {0};

        *(uint32_t*)&packet[0] = ((uint32_t)((i + txLen) == len)) << 2U;
        *(uint32_t*)&packet[0] |= (1U << (i > 0));
        *(uint32_t*)&packet[4] = (uint32_t)i + 1;

        memcpy(&packet[8], data + i, txLen);

        esp_err_t err = esp_now_send_packet(peer_addr, packet, txLen + 8, 1000UL);
        *transmitted++;

        if (err != ESP_OK) {
            if (millisll() - resend_timeout < timeout) {
                log_cdc(
                    "[ERROR] ESP-NOW Transmission Error, Resending Packet");
                continue;
            } else {
                status = ESP_ERR_TIMEOUT;
                goto esp_now_send_blocking_end;
            }
        }

        i += txLen;
    }
esp_now_send_blocking_end:
    xSemaphoreGive(esp_now_switch_mtx);
    return status;
}


/**
 * @brief function to process input characters
 * 
 * @param c input character
 */
void process_char(char c, bool is_esp_now = false) {

    // backspace received
    static bool enable_backspace_clear = true;
    if (c == '\b' && enable_backspace_clear) {
        rx_str = EXTString();
        log_cdc("cleared");
        if (is_esp_now) log_esp("cleared");
        return;
    }

    // escape received
    if (c == '\033') {

        // set keyboard report delay
        if (strncmp(rx_str.c_str(), "KB target ", 10) == 0 && rx_str.length() > 9 && rx_str.length() < 19) {
            String kb_target_str(rx_str.substring(9).c_str());
            kb_target_str.trim();
            rx_str = EXTString();

            int kb_target = kb_target_str.toInt();
            char ch = kb_target;
            if (kb_target_changed_sem == NULL || kb_target_change == NULL)
                return;
            
            if (xQueueSend(kb_target_change, &ch, 1000) != pdTRUE) return;
            if (xSemaphoreTake(kb_target_changed_sem, 1000) != pdTRUE) return;

            log_cdc("Target: %d", (int) kb_dev->target());
            if (is_esp_now) log_esp("Target: %d", (int) kb_dev->target());
            return;
        }

        if (strncmp(rx_str.c_str(), "KB bkspc", 8) == 0 && rx_str.length() == 8) {
            rx_str = EXTString();

            enable_backspace_clear = !enable_backspace_clear;
            log_cdc((enable_backspace_clear)? "Backspace clears RX buffer" : "Backspace is normal key");
            if (is_esp_now) log_esp((enable_backspace_clear)? "Backspace clears RX buffer" : "Backspace is normal key");
            return;
        }

        if (strncmp(rx_str.c_str(), "KB delay ", 9) == 0 && rx_str.length() > 9 && rx_str.length() < 19) {

            String kb_delay_str(rx_str.substring(9).c_str());
            kb_delay_str.trim();
            rx_str = EXTString();

            kb_delay = kb_delay_str.toInt();
            kb_delay = (kb_delay > 1000)? 1000 : kb_delay;

            log_cdc("Key Delay: %d ms", (int)kb_delay);
            if (is_esp_now) log_esp("Key Delay: %d ms", (int)kb_delay);
            return;
        }

        // set text position
        if (strncmp(rx_str.c_str(), "KB pos ", 7) == 0 && rx_str.length() > 7 && rx_str.length() < 17) {

            String kb_pos_str(rx_str.substring(7).c_str());
            kb_pos_str.trim();

            type_pos = kb_pos_str.toInt();
            type_pos = (type_pos >= tx_str.length())? (tx_str.length() - 1) : type_pos;

            rx_str = EXTString();
            log_cdc("Key Pos: %lu", type_pos);
            if (is_esp_now) log_esp("Key Pos: %lu", type_pos);
            return;
        }

        // set peer mac
        if (strncmp(rx_str.c_str(), "KB getmac", 9) == 0 && rx_str.length() == 9) {
            rx_str = EXTString();
            log_cdc("Peer MAC: %s", getMacString(masterInfo->peer_addr).c_str());
            if (is_esp_now) log_esp("Peer MAC: %s", getMacString(masterInfo->peer_addr).c_str());
            return;
        }
        // set peer mac
        if (strncmp(rx_str.c_str(), "KB setmac ", 10) == 0 && rx_str.length() == 27) {

            String peer_mac(rx_str.substring(10).c_str());
            peer_mac.trim();
            rx_str = EXTString();

            setMacAddr(peer_mac.c_str(), masterInfo);

            log_cdc("Peer MAC: %s", getMacString(masterInfo->peer_addr).c_str());
            if (is_esp_now) log_esp("Peer MAC: %s", getMacString(masterInfo->peer_addr).c_str());
            return;
        }

        // Print this device's MAC Address to usb cdc
        if (strncmp(rx_str.c_str(), "KB mac", 6) == 0 && rx_str.length() == 6) {
            rx_str = EXTString();
            log_cdc("MAC: %s", WiFi.macAddress().c_str());
            if (is_esp_now) log_esp("MAC: %s", WiFi.macAddress().c_str());
            return;
        }

        /**
         * @brief Load data from file to tx_str
         *
         */
        if (strncmp(rx_str.c_str(), "KB load", 7) == 0 &&
            rx_str.length() == 7) {

            rx_str = EXTString();

            // delay for safety
            vTaskDelay(100);

            log_cdc("loading");
            if (is_esp_now) log_esp("loading");

            // try loading string
            EXTString tmp; // temporary string holder
            int load_string_state = loadFileToStr("/file", tmp);

            if (load_string_state > 0) {
                // if string is loaded, move to tx_str
                tx_str.move(tmp);
                log_cdc("Loaded %lu bytes", tx_str.length());
                if (is_esp_now) log_esp("Loaded %lu bytes", tx_str.length());

            } else if (load_string_state == 0) { // if file empty
                log_cdc("File Empty");
                if (is_esp_now) log_esp("File Empty");
            } else { // error
                log_cdc("File Error: %d", load_string_state); 
                if (is_esp_now) log_esp("File Error: %d", load_string_state); 
            }

            return;
        }

        /**
         * @brief save data from tx_str to file
         *
         */
        if (strncmp(rx_str.c_str(), "KB save", 7) == 0 &&
            rx_str.length() == 7) {

            rx_str = EXTString();

            // delay for safety
            vTaskDelay(100);

            log_cdc("saving");
            if (is_esp_now) log_esp("saving");
            
            int saved = saveStrToFile("/file", tx_str);

            if (saved >= 0) {
                log_cdc("Saved %d bytes", saved);
                if (is_esp_now) log_esp("Saved %d bytes", saved);
            } else {
                log_cdc("Error Code: %d", saved);
                if (is_esp_now) log_esp("Error Code: %d", saved);
            }

            return;
        }

        /**
         * @brief save data from tx_str to file
         *
         */
        if (strncmp(rx_str.c_str(), "KB format", 9) == 0 &&
            rx_str.length() == 9) {

            rx_str = EXTString();

            // delay for safety
            vTaskDelay(100);

            log_cdc("Formatting");
            if (is_esp_now) log_esp("Formatting");

            bool success = disk_format();
            log_cdc("Formatting %s", (success)? "Success" : "Failed");
            if (is_esp_now) log_esp("Formatting %s", (success)? "Success" : "Failed");

            return;
        }

        // reshuffle enigma rotor
        if (strncmp(rx_str.c_str(), "KB rotor shuffle", 16) == 0 && rx_str.length() == 16) {

            rx_str = EXTString();

            // new pseudo random seed
            srand(esp_random());

            // reshuffle rotor
            volatile uint32_t cyccnt = ESP.getCycleCount();
            configRotor(&rotor, (const char*)rotor.ttb_out);
            cyccnt = ESP.getCycleCount() - cyccnt;

            // count cpu cycles
            log_cdc("CPU Cycles = %lu", cyccnt);
            if (is_esp_now) log_esp("CPU Cycles = %lu", cyccnt);

            return;
        }

        // run tx_str through enigma rotor
        if (strncmp(rx_str.c_str(), "KB encrypt", 10) == 0 && rx_str.length() == 10) {

            rx_str = EXTString();

            // reset each rotor
            resetrotor(&rotor);
            
            // run through each rotor
            volatile uint32_t cyccnt, cyccnt1;
            
            asm volatile ("rsr %0, ccount": "=a"(cyccnt));
            encryptString((uint8_t*)tx_str.c_str(), tx_str.length(), &rotor);
            asm volatile ("rsr %0, ccount": "=a"(cyccnt1));
            
            log_cdc("CPU Cycles = %lu", cyccnt1 - cyccnt);
            if (is_esp_now) log_esp("CPU Cycles = %lu", cyccnt1 - cyccnt);

            return;
        }

        // change string
        if (!tx_lock && rx_str.length() > 0) {
            tx_lock = true;
            tx_str = rx_str;
            type_pos = 0UL;
            rx_str = EXTString();
            tx_lock = false;
            log_cdc("Received %lu bytes", (uint32_t) tx_str.length());
            if (is_esp_now) log_esp("Received %lu bytes", (uint32_t) tx_str.length());
        }
    } else {
        rx_str += (char)c;
    }
}

QueueHandle_t queue_recv;

/**
 * @brief ESP_NOW packet received callback
 *
 * @param mac sender mac address
 * @param incomingData buffer of received data
 * @param len received data length
 */
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    uint32_t flags;
    uint32_t pos;
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    if (memcmp(mac_addr, masterInfo->peer_addr, sizeof(masterInfo->peer_addr)) != 0) return;
    memcpy(&flags, &data[0], sizeof(flags));
    memcpy(&pos, &data[4], sizeof(pos));

    if ((flags & 0xf0UL) == 0UL) {
        for (int i = 8; i < data_len; i++) {
            if (xPortInIsrContext()) {
                xQueueSendFromISR(queue_recv, &data[i], &pxHigherPriorityTaskWoken);
            } else xQueueSend(queue_recv, &data[i], 0);
        }
        if (flags & 4U) {
            uint8_t esc = '\033';
            if (xPortInIsrContext()) {
                xQueueSendFromISR(queue_recv, &esc, &pxHigherPriorityTaskWoken);
            } else xQueueSend(queue_recv, &esc, 0);
        }
        if (pxHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    } else if (flags == 0x10UL) {
        // led keyboard output
        log_cdc("caps");
        arduino_usb_hid_keyboard_event_data_t led_data;
        led_data.leds = (uint8_t)pos;
        kb_led_handler(led_data);
    }
}

/**
 * @brief keyboard definitions
 * 
 */
const uint8_t row_pins[] = {
    9, 8, 7, 6, 5
};

const uint8_t col_pins[] = {
    17, 16, 21, 18, 34, 33, 36, 35, 38, 37, 40, 12, 13, 11, 10
};

const size_t ROWS = sizeof(row_pins)/sizeof(row_pins[0]);
const size_t COLS = sizeof(col_pins)/sizeof(col_pins[0]);

/**
 * @brief keys[] : default key layout
 * 
 * Key Layout
 *
 * 
 */

const char* keys[ROWS*COLS] = {
    STR_ESC,        "1",      "2",      "3",     "4",      "5",      "6",      "7",      "8",      "9",      "0",      "-",        "=",             STR_BACKSPACE,  "`",
    STR_TAB,        "q",      "w",      "e",     "r",      "t",      "y",      "u",      "i",      "o",      "p",      "[",        "]",             "\\",           STR_DELETE,
    STR_CAPS_LOCK,  "a",      "s",      "d",     "f",      "g",      "h",      "j",      "k",      "l",      ";",      "\'",       "\0",            STR_RETURN,     STR_PAGE_UP,
    STR_LEFT_SHIFT, "z",      "x",      "c",     "v",      "b",      "n",      "m",      ",",      ".",      "/",       "\0",      STR_RIGHT_SHIFT, STR_UP_ARROW,   STR_PAGE_DOWN,
    STR_LEFT_CTRL,  STR_LEFT_GUI, STR_LEFT_ALT, "\0", "\0", " ",     "\0",     "\0",     "\0", STR_RIGHT_ALT, "\0", STR_RIGHT_CTRL, STR_LEFT_ARROW, STR_DOWN_ARROW, STR_RIGHT_ARROW,
};

/**
 * @brief keys_alternate[] : keymap for alternate layout
 * Alternate Key Layout
 * 
 * Key Layout when Fn Key is pressed 
 * (not in autotype mode)
 * 
 */

const char* keys_alternate[ROWS*COLS] = { 
    "\0", STR_F1, STR_F2, STR_F3, STR_F4, STR_F5, STR_F6, STR_F7, STR_F8, STR_F9, STR_F10, STR_F11, STR_F12, STR_MEDIA_STOP, STR_PRTSC, 
    "\0", "\0", "\0", STR_MEDIA_EMAIL, STR_NUM_7, STR_NUM_8, STR_NUM_9, "\0", "\0", "\0", STR_NUM_PERIOD, STR_MEDIA_PREV, STR_MEDIA_NEXT, STR_MEDIA_MUTE, STR_INSERT,
    STR_NUM_LOCK, STR_NUM_PLUS, STR_NUM_MINUS, STR_NUM_SLASH, STR_NUM_4, STR_NUM_5, STR_NUM_6, "\0", "\0", "\0", STR_MEDIA_BRIGHTNESS_D, STR_MEDIA_BRIGHTNESS_I, "\0", STR_MEDIA_PAUSE, STR_HOME,
    "\0", STR_MEDIA_WWW_HOME, STR_CONSUMER_CTL, STR_MEDIA_CALC, STR_NUM_1, STR_NUM_2, STR_NUM_3, STR_NUM_ASTERISK, "\0", "\0", "\0", "\0", "\0", STR_MEDIA_VOLUP, STR_END,
    "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0", STR_MEDIA_VOLDN, "\0",
};
/**
 * @brief main keyboard task
 * 
 * This task iterates through the keyboard matrix
 * by settings the row pins to LOW and then checks
 * if any column pins are high (key not pressed)
 * or low (key pressed). Then presses or releases 
 * the corresponding keys defined in keys array or 
 * keys_aternate array to the corresponding hid report 
 * with the methods defined in the HIDKeyboard 
 * Then, the report is sent to the host
 * 
 * Multiple modes on this keyboard:
 * - normal mode: 
 *       keyboard runs normally
 * 
 * - autotype mode: activate/deactivate by pressing FN key and ',' key
 *       keyboard types a specific text
 * 
 * @param param 
 */
void keyboard_task(void *param) {
    tasks_running++;

    HIDKeyboard* kb = (HIDKeyboard*) param; 
    uint32_t Pressed[ROWS * COLS] = {0};
    int64_t keyTimer[ROWS * COLS] = {0};

    kb_target_changed_sem = xSemaphoreCreateBinary();
    kb_target_change = xQueueCreate(2, sizeof(uint8_t));

    memset(Pressed, 0, sizeof(Pressed));
    for (auto& i : keyTimer) 
        i = millisll();

    const int64_t TIMEOUT_MS = 100LL;
    const uint32_t FN_KEY    = 70;

    uint32_t update = 0;
    uint32_t alternate = 0;
    uint8_t auto_type = 0;
    int64_t autotype_timer = 0;

    const auto clean_out = [Pressed, kb]() {
        memset((void*)Pressed, 0, sizeof(Pressed));
        kb->releaseAll();
        kb->sendKeyReport();
        vTaskDelay(1);
        kb->sendMediaReport();
    };
    
    // initialize pins
    for (int i : row_pins) {
        pinMode(i, OUTPUT);
        directWriteLow(i);
    }
    for (int i : col_pins) {
        pinMode(i, INPUT_PULLUP);
    }

    const char* target_str = "usb";
    if (!directRead(col_pins[1])) {
        masterInfo = &bt_peer_info;
        target_str = "bluetooth";
    }

    directWriteHigh(LED_BUILTIN);

    is_kb_on = true;

    // set row pins to high    
    for (int i : row_pins)
        directWriteHigh(i);    
    
    log_cdc("Target: %s", target_str);
    
    while (tasks_run) {
        bool clean = false;
        uint8_t autotype_flag = 0U;
        uint32_t n_pressed = 0;

        // scan matrix
        for (uint32_t i = 0; i < ROWS; i++) {
            directWriteLow(row_pins[i]);
            delayMicroseconds(1);
            for (int j = 0; j < COLS; j++) {
                uint32_t pos = i * COLS + j;
                uint32_t col_pressed = !directRead(col_pins[j]);
                int64_t thisTick = millisll();

                // set/unset alternate mode
                if (pos == FN_KEY) {
                    alternate = col_pressed;
                    continue;
                }

                if (col_pressed) {
                    feed_timeout();
                    // autotype mode toggle on press (press and release alt key and column 14 of each row simultaneously)
                    // if alternate mode is on and alternate key is defined
                    if (pos == 65 && alternate == 1) {
                        tasks_run = false;
                        goto timeout;
                    }
                    if (pos == 71 || pos == 72 || pos == 74) {
                        autotype_flag += 1;
                        if (autotype_flag == 3U && alternate) {
                            if (auto_type == 0U) auto_type = 2U;
                            else if (auto_type == 3U) auto_type = 1U;
                        }
                    }

                    const uint8_t* key = (const uint8_t*)((alternate) ? 
                                    keys_alternate[pos] : keys[pos]);
                    
                    if (key == NULL) continue;
                    if (key[0] == 0) continue;
                    n_pressed++;

                    // press key
                    // if key of 'pos' is free and not in debounce timeout
                    // if (Pressed[pos] == 0 && TimeOut[pos] == 0) {
                    if (Pressed[pos] == 0 && (thisTick - keyTimer[pos] >= TIMEOUT_MS)) {
                        uint32_t keycode = 0UL;
                        if (auto_type == 3U || auto_type == 0U) {
                            if (auto_type == 0U) {
                                if (keycode = kb->pressMulti(key))
                                    update |= keycode & KEY_TARGET_MASK;
                            }
                            update |= 1U;
                            Pressed[pos] = (uint32_t)key;
                        }
                    }
                } else {
                    // autotype mode toggle on release (press and release alt key and column 14 of each row simultaneously)
                    // release key
                    if (pos == 71 || pos == 72 || pos == 74) {
                        if (autotype_flag == 0U) {
                            if (auto_type == 2U) {
                                clean = true; auto_type = 3U;
                            } else if (auto_type == 1U) {
                                clean = true; type_pos = 0;
                                auto_type = 0U;
                            }
                            p_led_data.reserved = (auto_type > 0);
                        }
                    }
                    if (Pressed[pos] != 0) { // if key[pos] is pressed
                        uint32_t keycode = kb->releaseMulti((const uint8_t*)Pressed[pos]);
                        update |= keycode & KEY_TARGET_MASK;
                        update |= 2U;              // update release flag
                        Pressed[pos] = 0;          // unassign key from pressed array
                        keyTimer[pos] = thisTick;
                    }
                }
            }
            directWriteHigh(row_pins[i]);
        }

        if (auto_type == 3) {    // autotype long press
            if (n_pressed > 0 && (millisll() - autotype_timer >= 500)) {
                update |= 1U;
                autotype_timer = millisll() - 500;
            } else if (n_pressed == 0) {
                autotype_timer = millisll();
                type_pos %= tx_str.length();
            }
        }

        // clean output

        if (clean) 
            clean_out();

        static char target_change_to;
        if (xQueueReceive(kb_target_change, &target_change_to, 0) == pdTRUE) {
            clean_out();
            kb->target(target_change_to);
            xSemaphoreGive(kb_target_changed_sem);
        }

        // send report
        if (update & 3U) {
            if (((update & 1U) == 1U) && auto_type == 3) { 
                // autotype mode
                if (type_pos < tx_str.length()) {
                    uint8_t ch = tx_str[type_pos++];
                    uint32_t type = kb->press(ch);
                    if ((type & KEY_TARGET_CC) != 0UL) {
                        // target is consumer control
                        kb->sendMediaReport();
                        delay(kb_delay);
                        kb->releaseAll();
                        kb->sendMediaReport();
                        delay(kb_delay);
                    }
                    if ((type & KEY_TARGET_KB) != 0UL) {
                        // target is key report
                        kb->sendKeyReport();
                        delay(kb_delay);
                        kb->releaseAll();
                        kb->sendKeyReport();
                        delay(kb_delay);
                    }
                }
            } else if (auto_type == 0) {
                // normal mode
                if (update & KEY_TARGET_CC) {
                    kb->sendMediaReport();
                    delay(kb_delay);
                }
                if (update & KEY_TARGET_KB) {
                    kb->sendKeyReport();
                    delay(kb_delay);
                }
            }
            update = 0;
        } else {
            vTaskDelay(1);
        }
    }

    timeout:
    if (tasks_run) {
        tasks_run = false;
        log_cdc("Keyboard initialization timeout, Going to sleep mode");
    }
    // clean output before exiting task
    clean_out();

    // deinitialize matrix pins
    for (auto& i : row_pins) {
        directWriteLow(i);
        pinMode(i, ANALOG);
    }
    
    for (auto& i : col_pins) {
        directWriteLow(i);
        pinMode(i, ANALOG);
    }

    tasks_running--;
    vTaskDelete(NULL);
}

void esp_now_start() {

    WiFi.mode(WIFI_MODE_STA);
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_S));
    
    while (esp_now_init() != ESP_OK) {
        log_cdc("[ESP_NOW]: Init ERROR");
        log_cdc("[ESP_NOW]: Retrying");
        vTaskDelay(1000);
    }

    ERR_CHECK(esp_now_register_recv_cb(onDataRecv));
    ERR_CHECK(esp_now_register_send_cb(onDataSent));

    ERR_CHECK(esp_now_set_pmk((const uint8_t*)PMK_STRING));

    while (esp_now_add_peer(&usb_peer_info) != ESP_OK) {
        log_cdc("Failed to add peer");
        vTaskDelay(1000);
    }
    while (esp_now_add_peer(&bt_peer_info) != ESP_OK) {
        log_cdc("Failed to add peer");
        vTaskDelay(1000);
    }
}

void esp_now_stop() {
    esp_now_del_peer(usb_peer_info.peer_addr);
    esp_now_del_peer(bt_peer_info.peer_addr);
    
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();

    while (esp_now_deinit() != ESP_OK) 
        vTaskDelay(1);
    
    // WiFi.mode(WIFI_MODE_NULL);
}


/**
 * @brief task for serial input
 * 
 * @param param NULL
 */
void background_task(void* param) {
    tasks_running++;
    memset(espnow_buf, 0, sizeof(espnow_buf));

    log_cdc("Mounting SPIFFS");

    // load file
    int disk_state = disk_init();
    if (disk_state) { // if disk is mounted
        // try loading string
        EXTString tmp; // temporary string holder
        int load_string_state = loadFileToStr("/file", tmp);

        if (load_string_state > 0) {
            // if string is loaded, move to tx_str
            tx_str.move(tmp);
            log_cdc("Loaded %lu bytes", tx_str.length());

        } else if (load_string_state == 0) // if file empty
            log_cdc("File Empty");

        else // error
            log_cdc("File Error: %d", load_string_state); 
        
    } else {
        log_cdc("ERROR Mounting SPIFFS: %d", disk_state);
    }

    // configure enigma rotor
    configRotor(&rotor, "1234567890qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM");

    char c;
    int ch;

    while (tasks_run) {
        while ((ch = usb_cdc.read()) != -1) {
            if (usb_cdc.available() > 0x2000) {
                if (uxTaskPriorityGet(NULL) == 0)
                    vTaskPrioritySet(NULL, 2);
            } else if (uxTaskPriorityGet(NULL) == 2) {
                vTaskPrioritySet(NULL, 0);
            }
            process_char((char)ch);
        }
        while (xQueueReceive(queue_recv, &c, 0) == pdTRUE) {
            process_char(c, true);
        }
        static buf_len_t esp_now_message;
        while (xQueueReceive(esp_now_message_queue, &esp_now_message, 0) == pdTRUE) {
            size_t sent = 0;
            // EXTString& tmp = espnow_dq.front();
            esp_now_send_blocking(
                masterInfo->peer_addr, 
                esp_now_message.buf, 
                esp_now_message.size,
                &sent, 500);
            free(esp_now_message.buf);

            if (sent > 1)
                log_esp("Sent %d times", sent);
            // espnow_dq.pop_front();
        }
        
        vTaskDelay(1);
    }
    tasks_running--;
    vTaskDelete(NULL);
}


/**
 * @brief microcontroller wakeup reasons
 * 
 * @return std::string reason
 */
std::string print_wakeup_reason() {

    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    char reason[200];

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            sprintf(reason, "RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            sprintf(reason, "RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            sprintf(reason, "timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            sprintf(reason, "touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            sprintf(reason, "ULP");
            break;
        default:
            sprintf(reason, "%d", wakeup_reason);
            break;
    }
    return std::string(reason);
}


void setup() {

    // led keyboard on indicator
    pinMode(LED_BUILTIN, OUTPUT);
    const gpio_num_t portable_sleep_pin = (gpio_num_t)1;

    // initialize global variables
    // disable hold functions
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(portable_sleep_pin);
    gpio_hold_dis((gpio_num_t)col_pins[0]);
    for (uint8_t row_pin : row_pins) {
        gpio_hold_dis((gpio_num_t)row_pin);
    }
    
    is_espnow_on = false;
    tasks_running = 0UL;
    tasks_run = true;
    sleep_timeout = SLEEP_TIMEOUT;
    type_pos = 0UL;
    sent_queue = xQueueCreate(10, sizeof(esp_err_t));
    esp_now_send_packet_mtx = xSemaphoreCreateMutex();
    esp_now_switch_mtx = xSemaphoreCreateMutex();
    queue_recv = xQueueCreate(0x8000, sizeof(uint8_t));
    esp_now_message_queue = xQueueCreate(0x1000, sizeof(buf_len_t));
    // sent_status = -1;
    kb_delay = 0U;
    tx_lock = false;

    // instantiate usb
    kb_dev = new HIDKeyboard();

    // initialize usb
    
    USB.manufacturerName("Dewe's Nut");
    USB.productName("Dewe Wireless Keyboard");
    kb_dev->register_espnow_report(ESP_NOW_SendReport);
    kb_dev->begin();
    kb_dev->onEvent(kb_output);
    usb_cdc.setRxBufferSize(0x4000U);
    usb_cdc.begin();
    usb_cdc.onEvent(cdc_event_handler);
    USB.begin();
    
    // log_cdc("wakeup reason: %s", print_wakeup_reason().c_str());
    
    getMacAddr(PEER_MAC_ADDR, usb_peer_info.peer_addr);
    usb_peer_info.channel = 0;
    memcpy(usb_peer_info.lmk, LMK_STRING, 16);
    usb_peer_info.encrypt = true;

    getMacAddr(PEER_MAC_ADDR_BT, bt_peer_info.peer_addr);
    bt_peer_info.channel = 0;
    memcpy(bt_peer_info.lmk, LMK_STRING, 16);
    bt_peer_info.encrypt = true;
    
    masterInfo = &usb_peer_info;

    // start keyboard scan task
    p_led_data = {0};
    pinMode(capslock_pin, OUTPUT);
    pinMode(numlock_pin, OUTPUT);

    rx_str = EXTString();
    tx_str = EXTString();

    xTaskCreate(keyboard_task, "Keyboard", 16384U, kb_dev, 1, NULL);


    // start serial input task
    xTaskCreate(background_task, "background", 8192U, NULL, 0, NULL);

    bool portable_sleep = false;
    pinMode(portable_sleep_pin, INPUT_PULLUP);

    while (sleep_timeout > millisll() && tasks_run) {
        if (!is_espnow_on && is_kb_on && (!tud_ready() || kb_dev->target() > 1)) {
            log_cdc("Turning On Radio");
            if (xSemaphoreTake(esp_now_switch_mtx, 0xfffffffful) == pdTRUE) {
                esp_now_start();
                is_espnow_on = true;
                xSemaphoreGive(esp_now_switch_mtx);
            }
        } else if (is_espnow_on && (tud_ready() && kb_dev->target() == 1)) {
            log_cdc("Turning Off Radio");
            if (xSemaphoreTake(esp_now_switch_mtx, 0xfffffffful) == pdTRUE) {
                esp_now_stop();
                is_espnow_on = false;
                xSemaphoreGive(esp_now_switch_mtx);
            }
        }
        delay:
        vTaskDelay(225);
        if (tud_ready())
            feed_timeout();
        if (!directRead(portable_sleep_pin)) {
            portable_sleep = true;
            break;
        }
        static bool i = 1;
        if (is_kb_on) {
            if (i = !i) {
                directWrite(LED_BUILTIN, !p_led_data.reserved);
                vTaskDelay(50);
                directWrite(LED_BUILTIN, p_led_data.reserved);
            }
        }
        
    }

    // going to sleep mode
    log_d("Going To Sleep");
    tasks_run = false;
    while (tasks_running)
        vTaskDelay(100);

    // deinitialize usb and radio
    usb_cdc.end();
    kb_dev->end();
    delete kb_dev;

    if (is_espnow_on) {
        esp_now_stop();
        is_espnow_on = false;
    }
    WiFi.mode(WIFI_MODE_NULL);

    // deinit led keyboard on indicator

    if (portable_sleep) {
        for (int i = 0; i < 5; i++) {
            directWriteHigh(LED_BUILTIN);
            delay(100);
            directWriteLow(LED_BUILTIN);
            delay(100);
        }
        pinMode(LED_BUILTIN, ANALOG);

        for (auto i : row_pins)
            pinMode(i, ANALOG);

        for (auto i : col_pins)
            pinMode(i, ANALOG);

        esp_sleep_enable_ext0_wakeup(portable_sleep_pin, 0);
        gpio_pad_hold(portable_sleep_pin);
        gpio_deep_sleep_hold_en();
    } else {
        directWriteLow(LED_BUILTIN);
        pinMode(LED_BUILTIN, ANALOG);
        // setup wakeup pins
        pinMode(col_pins[0], INPUT_PULLUP);
        for (uint8_t row_pin : row_pins) {
            pinMode(row_pin, OUTPUT);
            directWriteLow(row_pin);
        }

        // config wakeup pins
        esp_sleep_enable_ext0_wakeup((gpio_num_t)col_pins[0], 0);
        for (uint8_t row_pin : row_pins) {
            gpio_pad_hold((gpio_num_t)row_pin);
        }
        gpio_pad_hold((gpio_num_t)col_pins[0]);
    }
    gpio_deep_sleep_hold_en();

    // https://www.youtube.com/watch?v=bYHB1HcARn8
    esp_deep_sleep_start();
}

void loop() {
    vTaskDelay(1000);
}
