// clang-format off
#include "main.hpp"
// clang-format on
#include <Arduino.h>
#include <EthernetUdp.h>
#include <M5GFX.h>
#include <M5Module_LAN.h>
#include <M5Unified.h>
#include <SPI.h>
#include <Wire.h>

#include "control_manager.hpp"
#include "st_control_state.hpp"
#include "system_manager.hpp"

// LAN module
M5Module_LAN lan_;
EthernetUDP udp_;
// M5GFX instance
M5GFX display;
M5Canvas canvas(&display);

// Board type
enum class BoardType : int
{
    M5STACK_BASIC = 0,
    M5STACK_CORES3 = 1,
    M5STACK_ATOMS3 = 2
};

BoardType board_type = BoardType::M5STACK_CORES3;

// thread priority
#define MAIN_SYSTEM_TASK_PRIORITY 3
#define UDP_RECV_TASK_PRIORITY    1
#define CTRL_TASK_PRIORITY        2

#define CTRL_TASK_TIME_INTERVAL     5
#define MAIN_TASK_TIME_INTERVAL     50
#define UDP_RECV_TASK_TIME_INTERVAL 50

// M5Stack PIN CoreS3
// LAN Module RST    0
// PA_SCL            1  [NOTE]
// デフォルトだとI2CとLANのCSが被るんでCSを13に変える
// PA_SDA            2
// Display CS        3
// TF CARD CS        4
// >> GPIO           5
// >> GPIO           6
// >> GPIO           7
#define EMS_INPUT 8  // PB_IN
// PB_OUT            9
// ADC              10
// INTERNAL   SCL   11
// INTERNAL   SDA   12
// LAN Module CS    13
// LAN Module INT   14 [NOTE] デフォルトだと10でADC使いたいかもしれんから変える
// Display RST      15
#define TX_TWAI_NUM 17  // CAN TX pin
#define RX_TWAI_NUM 18  // CAN RX pin
// LAN Module MISO  35
// LAN Module SCK   36
// LAN Module MOSI  37
// CAM_HREF         38
// >> TXD0          43
// >> RXD0          44
// CAM_PCLK         45
// CAM_VSYNC        46

// I2C
#define UNIT_SCALES_DEFAULT_ADDR  0x26
#define UNIT_ENCODER_DEFAULT_ADDR 0x40

// UDP and LAN
#define UDP_PORT_RECV 50001
#define UDP_PORT_SEND 50002
IPAddress destination_ip(192, 168, 8, 116);
IPAddress ip(192, 168, 8, 217);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x89};

SystemManager sys_manager;
ControlManager ctrl_manager;

bool exist_weight_scale = false;

/*
 *
 * TASK FUNCTIONS
 *
 */

static void udp_receive_task(void *arg) {
    int packet_size = 0;
    uint32_t packet_buffer_size = sys_manager.get_udp_recv_packet_size();
    uint8_t packetBuffer[packet_buffer_size];

    while (sys_manager.check_init_lan()) {
        packet_size = udp_.parsePacket();
        if (packet_size > 0) {
            udp_.read(packetBuffer, packet_size);
            sys_manager.set_udp_recv_packet(packetBuffer);
        }
        vTaskDelay(pdMS_TO_TICKS(UDP_RECV_TASK_TIME_INTERVAL));
    }
    vTaskDelete(NULL);
}

static void main_task(void *arg) {
    /*
     * 1. Initialize
     */
    sys_manager.set_state_machine_initializing();
    // >> 1.1. Initialize the task thread
    uint32_t display_time_interval = 250;          // ms
    uint32_t udp_heart_beat_time_interval = 1000;  // ms
    uint32_t reset_time_interval = 10000;          // ms
    // >> 1.2. Caluculation time Data
    // Timer
    unsigned long start, end;
    unsigned long start_debug, end_debug;
    start = millis();              // 計測開始時間
    end = millis();                // 計測終了時間
    double elapsed = end - start;  // 処理に要した時間をミリ秒に変換
    uint32_t max_calc_time_of_main_task = 0;
    uint32_t ave_calc_time_of_main_task = 0;
    uint32_t max_calc_time_of_send_task = 0;
    uint32_t ave_calc_time_of_send_task = 0;
    uint32_t time_cnt = 0;
    // >> 1.3. Heart Beat
    uint32_t heart_beat_interval =
        (uint32_t)((float)display_time_interval /
                   (float)MAIN_TASK_TIME_INTERVAL);  // ms
    if (heart_beat_interval == 0) {
        heart_beat_interval = 1;
    }
    // >> 1.4. Reset Time
    uint32_t reset_time_cnt = (uint32_t)((float)reset_time_interval /
                                         (float)MAIN_TASK_TIME_INTERVAL);  // ms
    // >> 1.5. UDP Send Packet
    uint8_t send_packet_buffer[sys_manager.get_udp_send_packet_size()];
    // >> 1.6. EMS Button
    uint8_t ems_input = 0;

    // >> 1.7. Initialize I2C
    Wire.begin(M5DEV.Ex_I2C.getSDA(), M5DEV.Ex_I2C.getSCL(), 400000UL);
    M5DEV_LOGI("SDA: %d, SCL: %d", M5DEV.Ex_I2C.getSDA(),
               M5DEV.Ex_I2C.getSCL());

    // >> 1.8. Control Manager
    sys_manager.set_control_cmd(ctrl_manager.get_cmd_ptr());

    sys_manager.set_initialized_main_task();
    // << END Initialize
    sys_manager.set_state_machine_ready();

    /*
     * 2. MAIN LOOP
     */
    while (1) {
        start = millis();  // 計測開始時間
        /* ------------------------- */
        // 1. Get State
        // >> 1.1. Update M5Stack status
        M5_UPDATE();
        // M5DEV_LOGI("M5Stack status updated");
        ems_input = digitalRead(EMS_INPUT);
        // M5_LOGI("EMS Input: %d", ems_input);
        if (ems_input == 0) {
            sys_manager.set_emergency_stop_for_control_task(false);
            ctrl_manager.set_emergency_stop(false);
        } else {
            sys_manager.set_emergency_stop_for_control_task(true);
            ctrl_manager.set_emergency_stop(true);
        }
        sys_manager.update_encoder_button(sys_manager.check_force_stop());

        sys_manager.update_manual_operating();

        sys_manager.cmd_executor();

        // 2. Display
        if (time_cnt % heart_beat_interval == 0) {
            sys_manager.updateDisplay();
            time_cnt = 0;
            // M5DEV_LOGI("Display updated");
        }

        // 3. UDP Send
        start_debug = millis();
        if (sys_manager.check_connected_udp()) {
            auto send_packet_size =
                sys_manager.get_udp_send_packet(send_packet_buffer);
            if (send_packet_size > 0) {
                udp_.beginPacket(destination_ip, UDP_PORT_SEND);
                udp_.write(send_packet_buffer, send_packet_size);
                udp_.endPacket();
            }
        } else {
            // TODO: Heart Beat for UDP
        }
        end_debug = millis();

        /* ------------------------- */
        end = millis();  // 計測終了時間

        // >> 処理時間の更新
        elapsed = end - start;  // 処理に要した時間をミリ秒に変換
        if (elapsed > max_calc_time_of_main_task) {
            max_calc_time_of_main_task = (uint32_t)elapsed;
        }
        ave_calc_time_of_main_task =
            ave_calc_time_of_main_task * 0.9 + (uint32_t)elapsed * 0.1;
        sys_manager.set_calc_time_of_main_task(ave_calc_time_of_main_task,
                                               max_calc_time_of_main_task);
        time_cnt++;
        if (time_cnt >= reset_time_cnt) {
            time_cnt = 0;
            max_calc_time_of_main_task = (uint32_t)elapsed;
        }
        elapsed = end_debug - start_debug;
        if (elapsed > max_calc_time_of_send_task) {
            max_calc_time_of_send_task = (uint32_t)elapsed;
        }
        ave_calc_time_of_send_task =
            ave_calc_time_of_send_task * 0.9 + (uint32_t)elapsed * 0.1;
        sys_manager.set_calc_time_of_udp_send_task(ave_calc_time_of_send_task,
                                                   max_calc_time_of_send_task);

        // >> END 処理時間の更新
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_TIME_INTERVAL));
    }
    vTaskDelete(NULL);
}

static void ctrl_task(void *arg) {
    /*
     * 1. Initialize
     */
    // >> 1.1. Initialize weight sensor module
    float weight = 0.0f;
    int32_t raw_adc = 0;
    M5_LOGW(" exist_weight_scale: %d", exist_weight_scale);

    // >> 1.2. Caluculation time Data
    uint32_t max_calc_time_of_ctrl_task = 0;
    uint32_t ave_calc_time_of_ctrl_task = 0;
    unsigned long start, end;      // 型は auto で可
    start = millis();              // 計測開始時間
    end = millis();                // 計測終了時間
    double elapsed = end - start;  // 処理に要した時間をミリ秒に変換

    ControlState state;

    ctrl_manager.init_servo_dummy();

    sys_manager.set_initialized_ctrl_task();

    while (1) {
        start = millis();  // 計測開始時間
        /* ------------------------- */

        // 1. Get State
        // >> 1.1. Get Weight and Raw ADC
        if (exist_weight_scale) {
            weight = ctrl_manager.get_weight();
            raw_adc = ctrl_manager.get_weightRawADC();
        }
        ctrl_manager.set_state_machine(sys_manager.check_state_machine());

        ctrl_manager.cmd_executor();

        ctrl_manager.update();

        // M5DEV_LOGI("CTRL TASK LOOP");
        //  6. Set Control State
        //  control state: CtrlManager -> SystemManager
        ctrl_manager.get_control_state(state);
        sys_manager.set_control_state(state);
        /* ------------------------- */
        end = millis();  // 計測開始時間

        // >> 処理時間の更新
        elapsed = end - start;  // 処理に要した時間をミリ秒に変換
        if (elapsed > max_calc_time_of_ctrl_task) {
            max_calc_time_of_ctrl_task = (uint32_t)elapsed;
        }
        ave_calc_time_of_ctrl_task =
            ave_calc_time_of_ctrl_task * 0.9 + (uint32_t)elapsed * 0.1;
        sys_manager.set_calc_time_of_ctrl_task(ave_calc_time_of_ctrl_task,
                                               max_calc_time_of_ctrl_task);
        // >> END 処理時間の更新

        vTaskDelay(pdMS_TO_TICKS(CTRL_TASK_TIME_INTERVAL));
    }
    vTaskDelete(NULL);
}

/*
 *
 * SETUP & LOOP
 *
 */

void setup(void) {
    /*
     * Initialize M5Stack
     * 1. IO
     * 2. Serial
     * 3. Canvas
     * 4. I2C
     * 5. LAN(UDP)
     * 6. CAN
     * 7. THREAD
     */
    // set config
    M5_BEGIN();
    M5_LOGI("M5Stack initialized");

    // 1. Initialize the IO
    pinMode(EMS_INPUT, INPUT_PULLUP);
    // <--END 1. Initialize the IO

    // 3. Initialize the canvas
    display.begin();

    canvas.setPsram(false);
    canvas.setColorDepth(1);
    canvas.createSprite((int32_t)M5DEV.Lcd.width(),
                        (int32_t)M5DEV.Lcd.height());
    canvas.setPaletteColor(1, GREEN);
    canvas.setTextScroll(false);
    canvas.setTextSize(2);
    sys_manager.set_canvas(&canvas);
    //
    M5DEV_LOGI("Canvas initialized");
    // <--END 3. Initialize the canvas

    // 4. Initialize I2C
    Wire.begin(M5DEV.Ex_I2C.getSDA(), M5DEV.Ex_I2C.getSCL(), 400000UL);
    M5DEV_LOGI("SDA: %d, SCL: %d", M5DEV.Ex_I2C.getSDA(),
               M5DEV.Ex_I2C.getSCL());

    exist_weight_scale =
        ctrl_manager.begin_scale(Wire, UNIT_SCALES_DEFAULT_ADDR);
    sys_manager.begin_encoder_button(&Wire, UNIT_ENCODER_DEFAULT_ADDR);
    M5_LOGI("IO initialized");
    // <--END 4. Initialize I2C

    // 5. Initialize LAN(UDP)
    // UDP Data
    uint8_t watchdog = 0;
    uint8_t cs_pin;
    uint8_t rst_pin;
    uint8_t int_pin;
    switch (board_type) {
        case BoardType::M5STACK_BASIC: {
            cs_pin = 5;    //  5;
            rst_pin = 0;   //  0;
            int_pin = 35;  // 35;
        } break;
        case BoardType::M5STACK_CORES3: {
            cs_pin = 13;
            rst_pin = 0;
            int_pin = 14;
        } break;
        case BoardType::M5STACK_ATOMS3: {
            // TODO: Check the pin number
            cs_pin = 5;
            rst_pin = 0;
            int_pin = 35;
        } break;
        default:
            break;
    }
    SPI.begin(SCK, MISO, MOSI, -1);
    M5_LOGI("SPI initialized");
    lan_.setResetPin(rst_pin);
    lan_.reset();
    M5_LOGI("LAN reset");
    lan_.init(cs_pin);
    M5_LOGI("LAN initialized");
    while (lan_.begin(mac, 1000, 1000) != 1) {
        M5DEV_LOGE("Error getting IP address via DHCP, trying again...");
        delay(1000);
        watchdog++;
        if (watchdog > 5) {
            M5DEV_LOGE("Ethernet shield was not found. Break");
            break;
        }
    }
    if (watchdog <= 5) {
        watchdog = 0;
        while (lan_.hardwareStatus() == EthernetNoHardware) {
            M5DEV_LOGE(
                "Ethernet shield was not found.  Sorry, can't run without "
                "hardware. :(");
            delay(500);
            watchdog++;
            if (watchdog > 5) {
                M5DEV_LOGE("Ethernet shield was not found. Break");
                break;
            }
        }
    }
    if (watchdog <= 5) {
        watchdog = 0;
        while (lan_.linkStatus() == LinkOFF) {
            M5DEV_LOGE("Ethernet cable is not connected.");
            delay(500);
            watchdog++;
            if (watchdog > 5) {
                M5DEV_LOGE("Ethernet cable is not connected. Break");
                break;
            }
        }
    }
    if (watchdog <= 5) {
        M5DEV_LOGI("Ethernet cable is connected.");
        sys_manager.set_initialized_lan();
        sys_manager.set_lan_info(lan_.localIP(), destination_ip, UDP_PORT_RECV,
                                 UDP_PORT_SEND);
        udp_.begin(UDP_PORT_RECV);
        M5DEV_LOGI("UDP initialized");
    }
    // <--END 5. Initialize LAN(UDP)

    // 6. Initialize the CAN
    ctrl_manager.init_twai(TX_TWAI_NUM, RX_TWAI_NUM);
    ctrl_manager.init_motor_driver();
    //    <--END Initialize the CAN

    // 7. Initialize THREAD
    // Make thread for receiving UDP packet
    xTaskCreate(udp_receive_task, "UDP_RECV_TASK", 4096, NULL,
                UDP_RECV_TASK_PRIORITY, NULL);
    // Make thread for main task
    xTaskCreate(main_task, "MAIN_TASK", 4096, NULL, MAIN_SYSTEM_TASK_PRIORITY,
                NULL);
    // Make thread for control task
    xTaskCreate(ctrl_task, "CTRL_TASK", 4096, NULL, CTRL_TASK_PRIORITY, NULL);
    //
    // M5DEV_LOGI("Thread initialized");
    // <--END 7. Initialize THREAD
}

void loop(void) {
    // M5_UPDATE();
}
