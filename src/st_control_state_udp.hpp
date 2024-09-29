#pragma once

#include "st_control_state.hpp"

struct ControlStateUdp
{
    uint8_t servo_id;
    uint8_t ctrl_mode;
    bool is_power_on;
    bool dummy[1];

    unsigned long timestamp;

    float act_joint_position;
    float act_joint_velocity;
    float act_joint_torque;

    float cmd_joint_position;
    float cmd_joint_velocity;
    float cmd_joint_torque;

    ControlStateUdp() {
        init();
    }

    void init() {
        servo_id = 0;
        ctrl_mode = 0;
        is_power_on = false;

        timestamp = 0;

        act_joint_position = 0.0;
        act_joint_velocity = 0.0;
        act_joint_torque = 0.0;
        cmd_joint_position = 0.0;
        cmd_joint_velocity = 0.0;
        cmd_joint_torque = 0.0;
    }

    void compressed_copy(ControlState& state) {
        servo_id = state.servo_id;
        ctrl_mode = static_cast<uint8_t>(state.ctrl_mode);
        is_power_on = state.is_power_on;
        timestamp = state.timestamp;
        act_joint_position = state.act_joint_position;
        act_joint_velocity = state.act_joint_velocity;
        act_joint_torque = state.act_joint_torque;
        cmd_joint_position = state.cmd_joint_position;
        cmd_joint_velocity = state.cmd_joint_velocity;
        cmd_joint_torque = state.cmd_joint_torque;
    }
};