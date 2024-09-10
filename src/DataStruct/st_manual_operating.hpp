#pragma once

enum class manual_operating_mode : int
{
    NONE = 0,
    CHANGE_SM,
    CHANGE_SERVO_ID,
    CHANGE_SERVO_POWER,
    CHANGE_SERVO_CONTROL_MODE,
    CMD_SERVO_CONTROL,
    CONNECT_CAN
};

enum class manual_operating_phase : int
{
    VALUE_CHANGE = 0,
    MODE_CHANGE = 1
};

enum class connected_can_switch : int
{
    DISCONNECT = 0,
    CONNECT = 1
};

struct manual_operating_state
{
    /* data */
    manual_operating_mode mode;
    manual_operating_phase act_phase;
    bool act_encoder_button_flag_pressed;
    bool prev_encoder_button_flag_pressed;
    bool encoder_button_flag_pressed_just_before;
    unsigned long first_pressed_time;
    bool act_encoder_button_flag_double_pressed;
    int act_encoder_value;
    int encoder_offest;

    manual_operating_state(/* args */) {
        mode = manual_operating_mode::NONE;
        act_phase = manual_operating_phase::VALUE_CHANGE;
        act_encoder_button_flag_pressed = false;
        prev_encoder_button_flag_pressed = false;
        encoder_button_flag_pressed_just_before = false;
        first_pressed_time = 0;
        act_encoder_button_flag_double_pressed = false;
        encoder_offest = 0;
    }
};
