#pragma once

#include <cstdint>

#include "Common/node_cmd.hpp"

struct ControlState
{
    bool is_init_scale;
    bool is_power_on;
    bool act_can_connection_status;
    bool is_init_joint_pos;
    unsigned long timestamp;

    int servo_id;
    basic_servo_ctrl_cmd_list ctrl_mode;
    double act_joint_position;
    double act_joint_velocity;
    double act_joint_torque;

    double cmd_joint_position;
    double cmd_joint_velocity;
    double cmd_joint_torque;

    double act_joint_position_0;
    double act_joint_velocity_0;
    double act_joint_torque_0;

    // for control
    double sum_error_vel;
    double sum_error_torque;

    double prev_joint_position;

    // servo info

    float sensor_weight;
    int32_t sensor_weight_raw_adc;

    int waiting_servo_id;
    int dummy2;

    ControlState() {
        init();
    }

    void init() {
        // >> Servo
        act_can_connection_status = false;
        servo_id = 0;  // 0: not used, 1: servo1, 2: servo2
        is_init_joint_pos = false;
        is_power_on = false;
        ctrl_mode = basic_servo_ctrl_cmd_list::STAY;
        timestamp = 0;

        // >> weight scale
        is_init_scale = false;
        sensor_weight = 0.0f;
        sensor_weight_raw_adc = 0.0;

        act_joint_position = 0.0;
        prev_joint_position = 0.0;
        act_joint_velocity = 0.0;
        act_joint_torque = 0.0;

        cmd_joint_position = 0.0;
        cmd_joint_velocity = 0.0;
        cmd_joint_torque = 0.0;

        act_joint_position_0 = 0.0;
        act_joint_velocity_0 = 0.0;
        act_joint_torque_0 = 0.0;

        sum_error_vel = 0.0;

        dummy2 = 0x2525;
    }

    void deepcopy(const ControlState& state) {
        // >> Servo
        act_can_connection_status = state.act_can_connection_status;
        servo_id = state.servo_id;
        is_init_joint_pos = state.is_init_joint_pos;
        is_power_on = state.is_power_on;
        ctrl_mode = state.ctrl_mode;
        timestamp = state.timestamp;

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

        act_joint_position_0 = state.act_joint_position_0;
        act_joint_velocity_0 = state.act_joint_velocity_0;
        act_joint_torque_0 = state.act_joint_torque_0;

        sum_error_vel = state.sum_error_vel;
    }
};