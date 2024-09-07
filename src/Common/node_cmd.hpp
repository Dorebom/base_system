#pragma once

#include "node_state_machine.hpp"
#include "stacker.hpp"

#define MAX_NODE_CMD_DATA_SIZE 200

enum class basic_m5stack_cmd_list : int
{
    /* data */
    NONE,
    CHANGE_SM_STABLE,
    CHANGE_SM_READY,
    CHANGE_SM_FORCE_STOP,
    RELEASE_FORCE_STOP,

    REQUEST_STATE,
    START_STREAM_STATE,
    STOP_STREAM_STATE,
    SET_NODE_ID,
    SET_CONFIG,

    RESET_ERROR,
    RESET_ALERT,
    // 以下、サーボ制御コマンド
    CHANGE_CONTROLLED_SRV_ID,
    CHANGE_SRV_POWER,  // ひとつだけサーボON/OFF
    CHANGE_SRV_CTRLMODE,

    SERVO_POSITION_CONTROL,
    SERVO_VELOCITY_CONTROL,
    SERVO_TORQUE_CONTROL,

    PERIOD_CMD  // この行は削除しないこと
};

struct common_cmd_code  // 32 bytes
{
    /* data */
    int source;
    int destination;
    int priority;
    int cmd_id;
    basic_m5stack_cmd_list cmd_type;
    int data_size;
    bool is_sys_cmd;
    bool is_used_msgpack;
    uint16_t dummy1;
    uint32_t dummy2;

    common_cmd_code(/* args */) {
        source = 0;
        destination = 0;
        priority = 0;
        cmd_id = 0;
        cmd_type = basic_m5stack_cmd_list::NONE;
        data_size = 0;
        is_sys_cmd = false;
        is_used_msgpack = false;
        dummy1 = 0;
        dummy2 = 0;
    }
};

struct st_node_cmd
{
    /* data */
    common_cmd_code cmd_code;
    std::uint8_t data[MAX_NODE_CMD_DATA_SIZE];
    st_node_cmd(/* args */) {
        cmd_code = common_cmd_code();
    }
};

class node_cmd {
private:
    /* data */
public:
    circular_stacker<st_node_cmd> cmd_stack_;
    node_cmd(size_t stack_size) : cmd_stack_(stack_size) {
    }
    ~node_cmd() {
    }
};

struct cmd_change_controlled_servo_id
{
    /* data */
    uint8_t servo_id;

    cmd_change_controlled_servo_id(/* args */) {
        servo_id = 0;
    }
};

struct cmd_change_servo_power
{
    /* data */
    uint8_t servo_id;
    bool is_on;

    cmd_change_servo_power(/* args */) {
        servo_id = 0;
        is_on = false;
    }
};

enum class basic_servo_ctrl_cmd_list : int
{
    STAY,
    POSITION,
    VELOCITY,
    TORQUE
};

struct cmd_change_servo_ctrl_mode
{
    /* data */
    uint8_t servo_id;
    basic_servo_ctrl_cmd_list ctrl_mode;

    cmd_change_servo_ctrl_mode(/* args */) {
        servo_id = 0;
        ctrl_mode = basic_servo_ctrl_cmd_list::STAY;
    }
};

struct cmd_position_control
{
    /* data */
    uint8_t servo_id;
    float target_position;

    cmd_position_control(/* args */) {
        servo_id = 0;
        target_position = 0.0;
    }
};

struct cmd_velocity_control
{
    /* data */
    uint8_t servo_id;
    float velocity;

    cmd_velocity_control(/* args */) {
        servo_id = 0;
        velocity = 0.0;
    }
};

struct cmd_torque_control
{
    /* data */
    uint8_t servo_id;
    float torque;

    cmd_torque_control(/* args */) {
        servo_id = 0;
        torque = 0.0;
    }
};