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
    bool is_occured_error_;
    bool is_occured_warning_;
    bool dummy[4];
    signed short int encoder_button_value;
    unsigned short int udp_recv_num;
    unsigned short int udp_send_num;
    signed short int dummy2[1];
    basic_m5stack_cmd_list act_cmd_type;
    unsigned int dummy3;

    SystemState() {
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
        dummy3 = 255;
    }
};
