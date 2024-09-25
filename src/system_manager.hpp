#pragma once

/*
 *
 * 機能説明
 * - 内部状態を表示する機能
 *   - VALUE
 *     - bool is_init_canvas
 *   - FUNC
 *     - void set_canvas(M5Canvas* canvas_);
       - void updateDisplay();
 *
 *
 *  TODO: 説明を追加する
 *
 */

#include <M5GFX.h>

#include <cstdint>
#include <memory>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
//
#include "st_control_state.hpp"
#include "st_system_state.hpp"
//
#include "Common/st_udp_frame.hpp"
//
#include "Sensor/unit_encoder.h"
//
#include "DataStruct/st_manual_operating.hpp"
#include "module_display.hpp"
#include "module_manual_operation.hpp"
#include "module_system_cmd.hpp"

#define MAX_NODE_CMD_DATA_SIZE  200
#define MAX_NODE_CMD_STACK_SIZE 10

class SystemManager {
private:
    ModuleDisplay display_;
    ModuleManualOperation manual_operation_;
    ModuleSystemCmd system_cmd_;

    // Flag
    // >> System
    bool is_init_main_task = false;
    // >> LAN and UDP
    bool is_requested_state_at_once = false;
    bool is_streaming_state = false;
    bool is_streaming_state_for_logging = false;
    // >> EMS Button
    bool prev_emergency_stop_switch_for_control_task = false;

    // Data
    std::shared_ptr<node_state> system_state_data;
    std::shared_ptr<node_state> control_state_data;

    SystemState* system_state_;
    ControlState* control_state_;
    //  >> Stack
    std::shared_ptr<node_cmd> node_cmd_;
    std::shared_ptr<node_cmd> ctrl_cmd_;
    // << Stack

    // for udp
    udp_frame udp_send_packet_;  // TODO node class への移植
    // int recent_stack_marker_size = 0;  // TODO node class への移植
    uint8_t unsent_data[MAX_STACK_SIZE_AT_ONCE];  // TODO node class への移植
    bool is_unsent_data = false;  // TODO node class への移植
    int unsent_stack_marker = 0;  // TODO node class への移植

    // Encoder Button
    Unit_Encoder encoder_button;

    // for Manual Operating
    manual_operating_state manual_operating_state_;

    // << END Data

    // Function
    // >> UDP
    void set_udp_send_state(
        std::shared_ptr<st_node_state> state);  // TODO node class への移植
    void set_udp_send_cmd(st_node_cmd cmd);     // TODO node class への移植
    void reset_udp_send_packet(
        bool discard_unsent_data);  // TODO node class への移植
    // >> B Node
    void set_cmd_start_logging();
    void set_cmd_stop_logging();
    void set_res_cmd_start_logging();
    void set_res_cmd_stop_logging();

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
    // >> State Machine
    void set_state_machine_initializing();
    void set_state_machine_ready();
    void set_state_machine_stable();
    void set_state_machine_force_stop();
    // >> Command
    void cmd_executor();
    bool check_force_stop();
    node_state_machine check_state_machine();

    // >> Manual Operating Command
    void set_cmd_connect_can(connected_can_switch connected_can_switch_);
    void set_cmd_change_state_machine(node_state_machine state_machine);
    void set_cmd_change_servo_id(uint8_t servo_id);
    void set_cmd_change_servo_power(uint8_t servo_id, bool is_power_on);
    void set_cmd_change_servo_ctrl_mode(uint8_t servo_id,
                                        basic_servo_ctrl_cmd_list ctrl_mode);
    void set_cmd_position_control(uint8_t servo_id, double target_position);
    void set_cmd_velocity_control(uint8_t servo_id, double velocity);
    void set_cmd_torque_control(uint8_t servo_id, double torque);

    // >> System Operating Mode
    void update_manual_operating();

    // EMS Button
    void set_emergency_stop_for_control_task(bool em_stop);
    // Encoder Button
    void begin_encoder_button(TwoWire* wire = &Wire,
                              uint8_t addr = ENCODER_ADDR);
    void update_encoder_button();
    void update_encoder_button(bool force_stop_status);
    bool check_encoder_button_flag();
    // Display
    void set_canvas(M5Canvas* canvas_);
    void update_display();
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
    void set_control_cmd(std::shared_ptr<node_cmd> cmd);
};