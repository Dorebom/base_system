#pragma once

#include <string>

#include "Common/node_cmd.hpp"

struct SystemState
{
    uint32_t ave_calc_time_of_main_task;
    uint32_t max_calc_time_of_main_task;
    uint32_t ave_calc_time_of_ctrl_task;
    uint32_t max_calc_time_of_ctrl_task;
    uint32_t ave_calc_time_of_udp_send_task;
    uint32_t max_calc_time_of_udp_send_task;
    bool emergency_stop_switch_for_control_task;
    bool encoder_button_flag;
    bool is_connected_udp;
    bool is_occured_error_;
    bool is_occured_warning_;
    bool is_logging;
    bool is_init_ctrl_task;
    bool is_init_lan;
    bool is_init_scale;
    bool dummmy[7];
    signed short int encoder_button_value;
    unsigned short int udp_recv_num;
    unsigned short int udp_send_num;
    signed short int dummy2[1];
    basic_m5stack_cmd_list act_cmd_type;
    float sensor_weight;
    int32_t sensor_weight_raw_adc;
    int32_t dummy3;

    SystemState() {
        init();
    }

    void init() {
        is_connected_udp = false;
        is_logging = false;
        is_occured_error_ = false;
        is_occured_warning_ = false;
        ave_calc_time_of_main_task = 0;
        max_calc_time_of_main_task = 0;
        ave_calc_time_of_ctrl_task = 0;
        max_calc_time_of_ctrl_task = 0;
        ave_calc_time_of_udp_send_task = 0;
        max_calc_time_of_udp_send_task = 0;
        emergency_stop_switch_for_control_task = false;
        encoder_button_flag = false;
        encoder_button_value = 0;
        udp_recv_num = 0;
        udp_send_num = 0;
        act_cmd_type = basic_m5stack_cmd_list::NONE;

        is_init_scale = false;
        sensor_weight = 0.0;
        sensor_weight_raw_adc = 0;

        dummy3 = 0xFFFF;
    }
};
