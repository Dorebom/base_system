#pragma once

#include <cstdint>

#include "Common/node_cmd.hpp"

struct ControlState
{
    bool is_init_scale;
    bool is_power_on;
    bool is_init_joint_pos;
    bool dummy[5];

    int servo_id;
    basic_servo_ctrl_cmd_list ctrl_mode;
    double act_joint_position;
    double act_joint_velocity;
    double act_joint_torque;

    double cmd_joint_position;
    double cmd_joint_velocity;
    double cmd_joint_torque;

    // servo info
    

    float sensor_weight;
    int32_t sensor_weight_raw_adc;

    ControlState() {
        // >> Servo
        servo_id = 0;  // 0: not used, 1: servo1, 2: servo2
        is_init_joint_pos = false;
        is_power_on = false;
        ctrl_mode = basic_servo_ctrl_cmd_list::STAY;

        // >> weight scale
        is_init_scale = false;
        sensor_weight = 0.0f;
        sensor_weight_raw_adc = 0.0;

        act_joint_position = 0.0;
        act_joint_velocity = 0.0;
        act_joint_torque = 0.0;

        cmd_joint_position = 0.0;
        cmd_joint_velocity = 0.0;
        cmd_joint_torque = 0.0;
    }

    void deepcopy(const ControlState& state) {
        // >> Servo
        servo_id = state.servo_id;
        is_init_joint_pos = state.is_init_joint_pos;
        is_power_on = state.is_power_on;
        ctrl_mode = state.ctrl_mode;

        // >> weight scale
        is_init_scale = state.is_init_scale;
        sensor_weight = state.sensor_weight;
        sensor_weight_raw_adc = state.sensor_weight_raw_adc;

        act_joint_position = state.act_joint_position;
        act_joint_velocity = state.act_joint_velocity;
        act_joint_torque = state.act_joint_torque;

        cmd_joint_position = state.cmd_joint_position;
        cmd_joint_velocity = state.cmd_joint_velocity;
        cmd_joint_torque = state.cmd_joint_torque;
    }
};