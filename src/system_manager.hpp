#pragma once

#include <M5GFX.h>

#include <cstdint>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
//
#include "st_control_state.hpp"
#include "st_system_state.hpp"
//
#include "Common/st_udp_frame.hpp"
//
#include "Sensor/unit_encoder.h"

#define TITLE_FONT_SIZE      2
#define TEXT_FONT_SIZE       1.5
#define TEXT_FONT_SIZE_SMALL 1.3

#define MAX_NODE_CMD_DATA_SIZE  200
#define MAX_NODE_CMD_STACK_SIZE 10

class SystemManager {
private:
    M5Canvas* canvas;

    // Flag
    // >> System
    bool is_init_canvas = false;
    bool is_init_main_task = false;
    bool is_init_ctrl_task = false;
    // >> LAN and UDP
    bool is_init_lan = false;
    bool is_connected_udp = false;
    bool is_requested_state_at_once = false;
    bool is_streaming_state = false;
    // >> EMS Button
    bool prev_emergency_stop_switch_for_control_task = false;

    // Data
    node_state system_state_data;
    node_state control_state_data;

    SystemState* system_state_;
    ControlState* control_state_;
    // SystemState system_state_;
    // ControlState control_state_;
    // node_state node_state_;
    //  >> Stack
    std::shared_ptr<node_cmd> node_cmd_;
    // << Stack
    udp_frame udp_send_packet_;        // TODO node class への移植
    int recent_stack_marker_size = 0;  // TODO node class への移植
    uint8_t unsent_data[MAX_STACK_SIZE_AT_ONCE];  // TODO node class への移植
    bool is_unsent_data = false;  // TODO node class への移植
    int unsent_stack_marker = 0;  // TODO node class への移植

    bool display_heart_beat = false;
    // LAN
    IPAddress local_ip;
    uint32_t recv_port;
    IPAddress destination_ip;
    uint32_t send_port;
    // Encoder Button
    Unit_Encoder encoder_button;

    // Function
    // >> UDP
    void set_udp_send_state(st_node_state state);  // TODO node class への移植
    void set_udp_send_cmd(st_node_cmd cmd);  // TODO node class への移植
    void reset_udp_send_packet(
        bool discard_unsent_data);  // TODO node class への移植

public:
    SystemManager();
    ~SystemManager();
    // Thread
    void set_initialized_main_task();
    void set_initialized_ctrl_task();
    void set_calc_time_of_main_task(uint32_t ave_calc_time,
                                    uint32_t max_calc_time);
    void set_calc_time_of_ctrl_task(uint32_t ave_calc_time,
                                    uint32_t max_calc_time);
    void set_calc_time_of_udp_send_task(uint32_t ave_calc_time,
                                        uint32_t max_calc_time);
    // System
    void set_state_machine_initializing();
    void set_state_machine_ready();
    void set_state_machine_stable();
    void set_state_machine_force_stop();
    void cmd_executor();
    bool check_force_stop();
    // EMS Button
    void set_emergency_stop_for_control_task(bool em_stop);
    // Encoder Button
    void begin_encoder_button(TwoWire* wire = &Wire,
                              uint8_t addr = ENCODER_ADDR);
    void update_encoder_button();
    void update_encoder_button(bool force_stop_status);
    // Display
    void set_canvas(M5Canvas* canvas_);
    void updateDisplay();
    // LAN
    void set_initialized_lan();
    bool check_init_lan();
    void set_lan_info(IPAddress local_ip_, IPAddress destination_ip_,
                      uint32_t recv_port_, uint32_t send_port_);
    uint32_t get_udp_recv_packet_size();
    uint32_t get_udp_send_packet_size();
    bool check_connected_udp();
    void set_udp_recv_packet(uint8_t* packetBuffer);
    int get_udp_send_packet(uint8_t* packetBuffer);
    // Control Unit
    void set_control_state(ControlState& state);
};