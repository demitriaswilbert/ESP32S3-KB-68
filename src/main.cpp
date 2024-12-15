#include "HIDKeyboard.h"
#include "TFT_eSPI.h"
#include "USB.h"
#include "USBCDC.h"
#include "esp32-hal-log.h"
#include "esp32-hal-tinyusb.h"
#include "esp_private/wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <deque>
#include <esp_now.h>
#include <vector>

extern "C" {
    #include "chudnovsky.h"
}

#define millisll() (esp_timer_get_time() / 1000LL)

#define THIS_MAC_ADDR "84:F7:03:EA:EC:F8"
#define PEER_MAC_ADDR "84:F7:03:EA:F8:D0"
#define PMK_STRING "Never Gonna Give"
#define LMK_STRING "You Up Never Gon"

static USBCDC usb_cdc(0);
static Reports hid_reports;

static QueueHandle_t queue_recv;
static QueueHandle_t sent_queue = NULL;
static SemaphoreHandle_t esp_now_send_packet_mtx;
static esp_now_peer_info_t* target_peer_info;

static uint32_t report_delay = 1000;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite txt_spr(&tft);

#define LED_BUILTIN 16

typedef struct
{
    uint8_t* buf;
    size_t size;
} report_info_t;

#define log_cdc(format, ...) usb_cdc.printf(format "\r\n", ##__VA_ARGS__)

static int64_t tc_last_time = esp_timer_get_time();
static int64_t tc_time_diff = 0;
static int64_t tc_final_delay;

void tc_init() {
    if (esp_timer_get_time() - tc_last_time > 20000) {
        tc_time_diff = 0;
        tc_last_time = esp_timer_get_time();
    }
}

void tc_delay(int64_t us) {
    tc_time_diff += esp_timer_get_time() - tc_last_time;
    tc_final_delay = report_delay;
    if (tc_final_delay > tc_time_diff) {
        tc_final_delay -= tc_time_diff;
        tc_time_diff = 0;
    } else {
        tc_final_delay = 0;
        tc_time_diff -= tc_final_delay;
    }
    vTaskDelay(tc_final_delay / 1000);
    tc_last_time += us;
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
bool getMacAddr(const char* str, uint8_t* buf_mac_addr)
{
    uint8_t mac_addr_tmp[6] = {0}, mac_addr_filled = 0;
    for (const char* i = str; *i != '\0' && mac_addr_filled < 12U; i++)
    {
        const char ch = *i;
        if (ch == ':') continue;
        uint8_t tmpval = 0U;
        if (ch >= '0' && ch <= '9')
        {
            tmpval = (uint8_t)(ch - '0');
        }
        else if (ch >= 'a' && ch <= 'f')
        {
            tmpval = (uint8_t)(ch - 'a' + 10);
        }
        else if (ch >= 'A' && ch <= 'F')
        {
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

/**
 * @brief send a packet, retries up to timeout ms if error
 *
 * @param peer_addr peer address
 * @param data data to send
 * @param len length of data
 * @param timeout (in ms)
 * @return esp_err_t
 */
esp_err_t esp_now_send_packet(const uint8_t* peer_addr, const uint8_t* data,
                              const size_t len, const uint32_t timeout)
{
    const int64_t time = millisll();
    esp_err_t err_send_fn;

    do
    {
        err_send_fn = esp_now_send(peer_addr, data, len);
        if (xQueueReceive(sent_queue, &err_send_fn, timeout) != pdTRUE)
            err_send_fn = ESP_FAIL;

    } while ((err_send_fn != ESP_OK) && (millisll() - time) < timeout);

    GPIO.out ^= (3UL << LED_BUILTIN);
    return err_send_fn;
}

/**
 * @brief esp-now sent report callback
 *
 * @param mac target mac address
 * @param status transmission status
 */
void onDataSent(const uint8_t* mac, esp_now_send_status_t status)
{
    esp_err_t sent_status =
        (status == ESP_NOW_SEND_SUCCESS) ? ESP_OK : ESP_FAIL;
    xQueueSend(sent_queue, &sent_status, 1000);
}

void onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int packet_len)
{
    uint32_t flags = ((uint32_t*)data)[0];
    uint32_t pos = ((uint32_t*)data)[1];

    report_info_t message;

    if (memcmp(mac_addr, target_peer_info->peer_addr,
               sizeof(target_peer_info->peer_addr)) != 0)
        return;

    if ((flags & 0xf0UL) == 0UL)
    {

        // if (flags & 4UL)
        //     ++packet_len;

        const size_t header_size = 8;
        message.size = packet_len - header_size;
        message.buf =
            (uint8_t*)heap_caps_malloc(message.size, MALLOC_CAP_SPIRAM);
        memcpy(message.buf, data + 8, message.size);

        // if (flags & 4UL) message.buf[message.size - 1] = '\033';

        if (xQueueSend(queue_recv, &message, 0) != pdTRUE)
            heap_caps_free(message.buf);
        vPortYieldOtherCore(1);
    }
    else if (flags == 0x10UL)
    {
        // led keyboard output
        arduino_usb_hid_keyboard_event_data_t led_data;
        led_data.leds = (uint8_t)pos;
        log_cdc("caps: %u", (unsigned int)led_data.leds);
        // kb_led_handler(led_data);
    }
}

/**
 * @brief send keyboard report via esp_now
 *
 * @param id report id
 * @param buf report data
 * @param len report length
 */
void ESP_NOW_SendReport(uint8_t id, const void* buf, size_t len)
{

    uint8_t data[0x100U];
    *(uint32_t*)&data[0] = 0x20;
    data[4] = id;
    memcpy(&data[5], buf, len);

    esp_err_t err = ESP_FAIL;
    if (xSemaphoreTake(esp_now_send_packet_mtx, 1000) == pdTRUE)
    {
        err = esp_now_send_packet(target_peer_info->peer_addr, &data[0],
                                  len + 1 + 4, 500);
        xSemaphoreGive(esp_now_send_packet_mtx);
    }
    if (err != ESP_OK) log_e("Report Failed");
}

void esp_now_start()
{

    WiFi.mode(WIFI_MODE_STA);

    uint8_t this_mac_buf[8];
    getMacAddr(THIS_MAC_ADDR, this_mac_buf);
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, this_mac_buf));

    ESP_ERROR_CHECK(
        esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_S));

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
    esp_now_set_pmk((const uint8_t*)PMK_STRING);

    ESP_ERROR_CHECK(esp_now_add_peer(target_peer_info));
}

void esp_now_stop()
{
    esp_now_del_peer(target_peer_info->peer_addr);

    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();

    ESP_ERROR_CHECK(esp_now_deinit());
}

void esp_now_to_cdc_task(void* param)
{
    report_info_t report_info;
    while (true)
    {
        if (xQueueReceive(queue_recv, &report_info, 0xfffffffful) == pdTRUE)
        {
            usb_cdc.write(report_info.buf, report_info.size);
            heap_caps_free(report_info.buf);
        }
    }
    vTaskDelete(NULL);
}

typedef enum
{
    ACT_CMB = 0,
    ACT_SEP,
    ACT_SEP_SLOW,
    ACT_PRS,
    ACT_RLS,
    ACT_RLSALL,
    ACT_DELAY,
    ACT_MOUSE_GOCRAZY,
} action_type_e;

typedef struct
{
    action_type_e type;
    String str;
} kb_action_t;

typedef struct
{
    String name;
    std::vector<kb_action_t> actions;
} kb_command_t;

std::vector<kb_command_t> kb_commands = {
    {"taskmgr",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "taskmgr\n"},
     }},
    {"Ncpa.cpl",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "Ncpa.cpl\n"},
     }},
    {"code",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "code\n"},
     }},
    {"Appwiz.cpl",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "Appwiz.cpl\n"},
     }},
    {"Python Serial",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "python -m serial\n"},
     }},
    {"Rick Roll",
     {
         // rick roll
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "https://youtu.be/dQw4w9WgXcQ\n"},
     }},
    {"Download",
     {
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP,
          "powershell -WindowStyle Hidden -Command \"d:; cd D:\\Testing; git "
          "clone https://github.com/demitriaswilbert/ESP32S2_KB_68.git\"\n"},
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "https://youtu.be/dQw4w9WgXcQ\n"},
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "echo \"\"\n"},
         {ACT_RLSALL, ""},
     }},
    {"BSOD",
     {
         // bsod
         {ACT_SEP_SLOW, STR_LEFT_GUI},
         {ACT_SEP, "powershell"},
         {ACT_DELAY, "100"},
         {ACT_CMB, STR_LEFT_CTRL STR_LEFT_SHIFT "\n"},
         {ACT_DELAY, "500"},
         {ACT_SEP_SLOW, STR_LEFT_ARROW "\n"},
         {ACT_DELAY, "300"},
         {ACT_CMB, STR_LEFT_ALT STR_TAB},
         {ACT_DELAY, "300"},
         {ACT_CMB, STR_LEFT_ALT STR_LEFT_SHIFT STR_TAB},
         {ACT_DELAY, "300"},
         {ACT_SEP, "wininit"},
         {ACT_RLSALL, ""},
     }},
    {"Sign Out PC",
     {
         // shut down
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "logoff\n"},
     }},
    {"Shut Down PC",
     {
         // shut down
         {ACT_CMB, STR_LEFT_GUI "r"},
         {ACT_SEP, "shutdown /s /t 0\n"},
     }},
};

QueueHandle_t mouse_circle_queue = NULL;

hid_mouse_report_t ms_reports[200] = {};

void calculate_circle(hid_mouse_report_t* reports, float radius, size_t samples)
{
    float table[samples][2];
    for (int i = 0; i < samples; i++)
    {
        table[i][0] = -radius * cosf(i * PI / 100);
        table[i][1] = -radius * sinf(i * PI / 100);
    }

    float last_pos[2] = {table[0][0], table[0][1]};

    for (int i = 0; i < samples; i++)
    {
        int dx = table[(i + 1) % samples][0] - last_pos[0];
        int dy = table[(i + 1) % samples][1] - last_pos[1];
        last_pos[0] += dx;
        last_pos[1] += dy;
        reports[i].x = dx;
        reports[i].y = dy;
    }
}

void mouse_circle(int i, int radius = 500.f)
{
    static float r = 0.f;
    if (r != radius)
    {
        r = radius;
        calculate_circle(ms_reports, radius, 200);
    }
    ESP_NOW_SendReport(HID_REPORT_ID_MOUSE, &ms_reports[i % 200],
                        sizeof(hid_mouse_report_t));
}

#define ENCODER_CLK 13
#define ENCODER_DT 12
#define ENCODER_BTN 10

typedef enum
{
    ENC_NONE = 0,
    ENC_LEFT,
    ENC_RIGHT,
    ENC_PRESS,
    ENC_RELEASE,
} encoder_events;

QueueHandle_t encoder_queue;

void encoder_task(void* param)
{
    int64_t clk_tmr = esp_timer_get_time();
    int64_t btn_tmr = esp_timer_get_time();
    while (true)
    {
        int64_t tmr_now = esp_timer_get_time();
        bool clk_state = !!(GPIO.in & (1UL << ENCODER_CLK));
        static bool prev_clk_state = true;
        bool dt_state = !!(GPIO.in & (1UL << ENCODER_DT));

        if (!clk_state && prev_clk_state)
        {
            if (tmr_now - clk_tmr >= 2500)
            {
                prev_clk_state = false;
                encoder_events evt = dt_state ? ENC_RIGHT : ENC_LEFT;
                xQueueSend(encoder_queue, &evt, 0);
            }
            clk_tmr = tmr_now;
        }
        else if (clk_state && !prev_clk_state)
        {
            prev_clk_state = true;
            clk_tmr = tmr_now;
        }

        static bool prev_btnstate = false;
        bool btn_state = !(GPIO.in & (1UL << ENCODER_BTN));

        if (btn_state != prev_btnstate)
        {
            if (tmr_now - btn_tmr > 20000)
            {
                encoder_events evt = btn_state ? ENC_PRESS : ENC_RELEASE;
                xQueueSend(encoder_queue, &evt, 0);
                prev_btnstate = btn_state;
            }
            btn_tmr = tmr_now;
        }
    }
}

void encoder_init()
{
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_BTN, INPUT_PULLUP);

    encoder_queue = xQueueCreate(10, sizeof(encoder_events));
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, 0, NULL,
                            1);
}

encoder_events encoder_read(uint32_t delay_tick)
{
    encoder_events res = ENC_NONE;
    if (xPortInIsrContext())
    {
        xQueueReceiveFromISR(encoder_queue, &res, NULL);
        return res;
    }
    xQueueReceive(encoder_queue, &res, delay_tick);
    return res;
}

void setup()
{
    queue_recv = xQueueCreate(0x1000, sizeof(report_info_t));
    sent_queue = xQueueCreate(0x10, sizeof(esp_err_t));
    esp_now_send_packet_mtx = xSemaphoreCreateMutex();
    mouse_circle_queue = xQueueCreate(1, sizeof(float));

    pinMode(16, OUTPUT);
    pinMode(17, OUTPUT);

    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    pinMode(14, INPUT_PULLUP);

    USB.manufacturerName("Dewe's Nut");
    USB.productName("Dewe Wireless Keyboard");

    usb_cdc.setRxBufferSize(0x4000U);
    usb_cdc.begin();

    target_peer_info =
        (esp_now_peer_info_t*)calloc(1, sizeof(esp_now_peer_info_t));
    getMacAddr(PEER_MAC_ADDR, target_peer_info->peer_addr);
    target_peer_info->channel = 0;
    memcpy(target_peer_info->lmk, LMK_STRING, 16);
    target_peer_info->encrypt = true;
    esp_now_start();
    xTaskCreatePinnedToCore(esp_now_to_cdc_task, "esp_now_to_cdc", 4096, NULL,
                            2, NULL, 1);
    USB.begin();

    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);

    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);
    pinMode(6, OUTPUT);

    digitalWrite(6, LOW);
    digitalWrite(14, LOW);
    digitalWrite(15, HIGH);

    encoder_init();

    pinMode(8, INPUT_PULLUP);
}

void do_command(std::vector<kb_action_t>& vec)
{
    static Reports report;
    int64_t report_time;

    for (int i = 0; i < vec.size(); i++)
    {
        action_type_e combine = vec[i].type;
        const String& str = vec[i].str;

        // multipress combination
        switch (combine)
        {

            int delay;
            case ACT_CMB:
                report.pressMulti((const uint8_t*)str.c_str());
                ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                   &report.getKeyReport(),
                                   sizeof(report.getKeyReport()));
                vTaskDelay(20);
                report.releaseAll();
                ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                   &report.getKeyReport(),
                                   sizeof(report.getKeyReport()));
                vTaskDelay(80);
                break;

            case ACT_SEP:
                tc_init();
                for (size_t j = 0; j < str.length(); j++)
                {
                    report.press(str[j]);
                    ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                       &report.getKeyReport(),
                                       sizeof(report.getKeyReport()));

                    tc_delay(report_delay);

                    report.releaseAll();
                    ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                       &report.getKeyReport(),
                                       sizeof(report.getKeyReport()));

                    tc_delay(report_delay);
                }
                vTaskDelay(100);
                break;

            case ACT_SEP_SLOW:
                for (size_t j = 0; j < str.length(); j++)
                {
                    report.press(str[j]);
                    ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                       &report.getKeyReport(),
                                       sizeof(report.getKeyReport()));
                    vTaskDelay(20);
                    report.releaseAll();
                    ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                       &report.getKeyReport(),
                                       sizeof(report.getKeyReport()));
                    vTaskDelay(30);
                }
                vTaskDelay(100);
                break;

            case ACT_PRS:
                report.pressMulti((const uint8_t*)str.c_str());
                ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                   &report.getKeyReport(),
                                   sizeof(report.getKeyReport()));
                vTaskDelay(20);
                break;

            case ACT_RLS:
                report.releaseMulti((const uint8_t*)str.c_str());
                ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                   &report.getKeyReport(),
                                   sizeof(report.getKeyReport()));
                vTaskDelay(80);
                break;

            case ACT_RLSALL:
                report.releaseAll();
                ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                   &report.getKeyReport(),
                                   sizeof(report.getKeyReport()));
                vTaskDelay(100);
                break;

            case ACT_DELAY:
                delay = str.toInt();
                delay = delay < 100 ? 100 : delay;
                vTaskDelay(delay);
                break;

            case ACT_MOUSE_GOCRAZY:
                mouse_circle(0);
        }
    }
}

static int print_line_to_tft(const String str,
                              int32_t pos, bool selected, bool big)
{
    txt_spr.createSprite(240, big? 30 : 20);
    txt_spr.setFreeFont(big? &FreeMonoBold18pt7b : &FreeMonoBold12pt7b);
    txt_spr.fillSprite(selected ? TFT_WHITE : TFT_BLACK);
    txt_spr.setTextColor(selected ? TFT_BLACK : TFT_WHITE);
    txt_spr.drawString(str, 8, 0);
    txt_spr.pushSprite(0, pos);
    txt_spr.deleteSprite();
    return big? 30 : 20;
}

typedef struct
{
    String label;
    void (*func)(void* param);
} label_function_t;

void enter_command_demo(void* p)
{
    int labels_size = kb_commands.size();

    bool update = true;
    encoder_events evt = ENC_NONE;
    int choice = 0;
    bool run = true;
    bool clear_screen = true;
    int label_start = 0;
    int label_display_len = labels_size > 8? 8 : labels_size;
    int scrollable = labels_size - label_display_len; 

    while (encoder_read(0))
        ;

    while (run)
    {
        if (update)
        {
            if (clear_screen)
            {
                tft.fillScreen(TFT_BLACK);
                clear_screen = false;
            }
            int pos = 0;
            // title
            pos += print_line_to_tft("Commands", pos, false, true);

            // contents
            int i = label_start;
            for (int n = 0; n < label_display_len && i < labels_size; n++) {
                pos += print_line_to_tft(                                  String(i + 1) + ". " + kb_commands[i].name, pos,
                                  (i == choice), false);
                i++;
            }

            // back
            print_line_to_tft("Back", pos, (i == choice), false);
        }
        update = false;
        evt = encoder_read(100);
        switch (evt)
        {
            case ENC_LEFT:
                choice = choice > 0 ? choice - 1 : labels_size;
                break;
            case ENC_RIGHT:
                choice = choice < labels_size ? choice + 1 : 0;
                break;
            case ENC_PRESS:
                if (choice == labels_size)
                    run = false;
                else
                {
                    std::vector<kb_action_t> &actions_vec = kb_commands[choice].actions;
                    do_command(actions_vec);
                }
                break;
            default:
                break;
        }
        int start_idx = label_start;
        int end_idx = label_start + label_display_len - 1;

        if (choice <= start_idx) {
            label_start = choice > 0? choice - 1 : 0;
        } else if (choice >= end_idx) {
            label_start = choice + 1 - label_display_len;
            label_start = label_start < scrollable? label_start : scrollable;
        }
        update |= evt > 0;
    }
    tft.fillScreen(TFT_BLACK);
}

int demo_circle_radius = 500;
void demo_mouse_circle(void* p) 
{ 
    tft.fillScreen(TFT_BLACK);
    int pos = print_line_to_tft("Mouse Circle", 0, false, true);
    bool update = true;
    bool run = true;

    int choice = 0;
    bool pressed = false;
    bool change_radius = false;

    int radius = 500;
    Reports report;

    while (run)
    {
        if (update) {
            update = false;
            print_line_to_tft("Mouse Circle", 30, choice == 0, false);
            print_line_to_tft((change_radius? "    " : "Radius: ") + String(radius), 50, choice == 1, false);
            print_line_to_tft("Back", 70, choice == 2, false);
        }
        if (pressed && choice == 0) {
            tc_init();
            static int i = 0;
            mouse_circle(i, radius);
            i = (i + 1) % 200;
            tc_delay(report_delay);
            usb_cdc.write('a');
        }
        encoder_events evt = encoder_read((pressed && (choice == 0))? 0 : 200);
        update |= (evt > 0);
        switch (evt) {
            case ENC_LEFT: 
                if (pressed) break;
                if (change_radius)
                    radius = radius > 10? radius - 10 : 0;
                else choice = (choice + 2) % 3;

                break;
            case ENC_RIGHT:
                if (pressed) break;
                if (change_radius)
                    radius = radius < 2000? radius + 10 : radius;
                else choice = (choice + 1) % 3;

                break;
            case ENC_PRESS:
                pressed = true;
                if (choice == 1) change_radius = !change_radius;
                else if (choice == 2) run = false;
                break;
            case ENC_RELEASE:
                pressed = false;
                break;
        }
    }
    tft.fillScreen(TFT_BLACK);
}

void demo_print_pi(void* p) 
{
    tft.fillScreen(TFT_BLACK);
    int pos = print_line_to_tft("Print Pi", 0, false, true);

    pos += print_line_to_tft("Computing", pos, false, true);
    pos += print_line_to_tft("Pi", pos, false, true);
    
    // calculate pi
    mpz_t pi;
    mpzinit(pi);
    chudnovsky(pi, 5000);

    // convert to string
    char* str = mpz_get_str(NULL, 10, pi);
    size_t len = strlen(str);

    // free mpz
    mpzclear(pi);

    print_line_to_tft("Pi " + String(len), 0, false, true);

    int choice = 0;
    int pi_index = 0;
    bool type_enable = false;
    Reports report;

    bool update = true;
    bool run = true;

    while (run)
    {
        if (update) {
            update = false;
            print_line_to_tft(String(pi_index) + " / " + String(len), 30, choice == 0, false);
            print_line_to_tft("Reset", 50, choice == 1, false);
            print_line_to_tft("Back", 70, choice == 2, false);
        }
        if (type_enable) {
            if (pi_index == len) 
                pi_index = 0;
            
            tc_init();

            report.press(str[pi_index++]);
            ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                &report.getKeyReport(),
                                sizeof(report.getKeyReport()));

            tc_delay(report_delay);

            report.releaseAll();
            ESP_NOW_SendReport(HID_REPORT_ID_KEYBOARD,
                                &report.getKeyReport(),
                                sizeof(report.getKeyReport()));
            
            tc_delay(report_delay);

            update |= (pi_index % 50 == 49 || pi_index == len);
            type_enable &= pi_index < len;
        }
        encoder_events evt = encoder_read(type_enable? 0 : 200);
        update |= (evt > 0);
        switch (evt) {
            case ENC_LEFT: 
                if (type_enable) break;
                choice = (choice + 2) % 3;
                break;
            case ENC_RIGHT:
                if (type_enable) break;
                choice = (choice + 1) % 3;
                break;
            case ENC_PRESS:
                if (choice == 0) type_enable = true;
                else if (choice == 1) pi_index = 0;
                else run = false;
                break;
            case ENC_RELEASE:
                if (choice == 0) type_enable = false;
                break;
        }
    }
    free(str);
    tft.fillScreen(TFT_BLACK);
}

void demo_hid_delay(void* p) 
{ 
    tft.fillScreen(TFT_BLACK);
    int pos = print_line_to_tft("HID Delay", 0, false, true);
    bool update = true;
    bool run = true;

    bool choice = false;
    bool change_delay = false;

    while (run)
    {
        if (update) {
            update = false;
            print_line_to_tft((change_delay? "    " : "Delay: ") + String(report_delay), 30, choice == 0, false);
            print_line_to_tft("Back", 50, choice == 1, false);
        }
        encoder_events evt = encoder_read(200);
        update |= (evt > 0);
        switch (evt) {
            case ENC_LEFT: 
                if (change_delay) {
                    uint32_t tmp_delay = report_delay;
                    if (tmp_delay <= 5000)
                        tmp_delay = 1000;
                    if (tmp_delay > 5000)
                        tmp_delay -= 5000;
                    report_delay = tmp_delay;
                }
                else choice = !choice;

                break;
            case ENC_RIGHT:
                if (change_delay) {
                    uint32_t tmp_delay = report_delay;
                    if (tmp_delay == 1000)
                        tmp_delay += 4000;
                    if (tmp_delay < 500000)
                        tmp_delay += 5000;
                    report_delay = tmp_delay;
                }
                else choice = !choice;

                break;
            case ENC_PRESS:
                if (choice) run = false;
                else change_delay = !change_delay;
                break;
            case ENC_RELEASE:
                break;
        }
    }
    tft.fillScreen(TFT_BLACK);
}

void enter_option_demo(String& str)
{
    static std::vector<label_function_t> labels = {
        {"Commands", enter_command_demo},
        {"Pi", demo_print_pi},
        {"Mouse", demo_mouse_circle},
        {"HID delay", demo_hid_delay},
    };
    int labels_size = labels.size();

    bool update = true;
    encoder_events evt = ENC_NONE;
    int choice = 0;
    bool run = true;
    bool clear_screen = true;
    int label_start = 0;
    int label_display_len = labels_size > 8? 8 : labels_size;
    int scrollable = labels_size - label_display_len; 
    while (encoder_read(0))
        ;

    while (run)
    {
        if (update)
        {
            update = false;
            if (clear_screen)
            {
                tft.fillScreen(TFT_BLACK);
                clear_screen = false;
            }
            int pos = 0;
            // title
            pos += print_line_to_tft("Main Menu", pos, false, true);

            // contents
            int i = label_start;
            for (int n = 0; n < label_display_len && i < labels_size; n++) {
                pos += print_line_to_tft(String(i + 1) + ". " + labels[i].label, pos,
                                  (i == choice), false);
                i++;
            }

            // back
            print_line_to_tft("Back", pos, (i == choice), false);
        }
        evt = encoder_read(100);
        update |= evt > 0;
        switch (evt)
        {
            case ENC_LEFT:
                choice = choice > 0 ? choice - 1 : labels_size;
                break;
            case ENC_RIGHT:
                choice = choice < labels_size ? choice + 1 : 0;
                break;
            case ENC_PRESS:
                if (choice == labels_size)
                    run = false;
                else
                {
                    if (labels[choice].func != NULL)
                        labels[choice].func(NULL);
                }
                break;
            default:
                break;
        }
        int start_idx = label_start;
        int end_idx = label_start + label_display_len - 1;

        if (choice <= start_idx) {
            label_start = choice > 0? choice - 1 : 0;
        } else if (choice >= end_idx) {
            label_start = choice + 1 - label_display_len;
            label_start = label_start < scrollable? label_start : scrollable;
        }
    }
    tft.fillScreen(TFT_BLACK);
}

void loop()
{
    String str = "Main menu";
    enter_option_demo(str);
}