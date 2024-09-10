#pragma once

#include <cstdint>
#include <map>
#include <memory>

#include "Common/node_cmd.hpp"
#include "Sensor/unit_scales.hpp"
//
#include "Device/circular_buffer.hpp"
#include "Device/xiaomi_cybergear_driver.hpp"
#include "driver/twai.h"
//
#include "st_control_state.hpp"

#define MAX_CTRL_CMD_DATA_SIZE  200
#define MAX_CTRL_CMD_STACK_SIZE 10

struct control_status
{
    /* data */
    bool is_init_scale;

    control_status() {
        is_init_scale = false;
    }
};

class ControlManager {
private:
    // Data
    control_status status_;
    ControlState state_;
    std::shared_ptr<node_cmd> ctrl_cmd_;
    node_state_machine state_machine_;
    bool is_init_all = false;

    // >> CAN
    twai_message_t receive_can_packet;
    twai_message_t send_can_packet;
    CircularBuffer<twai_message_t, 10> receive_can_packet_buffer;
    CircularBuffer<twai_message_t, 10> send_can_packet_buffer;
    uint8_t rx_can_cnt;
    bool is_connecting_servo = false;
    bool is_waiting_rx_can = false;
    XiaomiCyberGearDriver cybergear_driver;
    std::map<uint8_t, XiaomiCyberGearStatus> motor_status;
    void init_motor_status() {
        motor_status.clear();
        motor_status[0x10] = XiaomiCyberGearStatus();
        motor_status[0x10].can_id = 0x10;
    }
    void get_motor_id(uint8_t can_id);
    bool recv_can();
    void stop_motor(uint8_t can_id);
    void stay_motor(uint8_t can_id);
    void position_control(uint8_t can_id, double target_position);
    void velocity_control(uint8_t can_id, double target_velocity);
    void torque_control(uint8_t can_id, double target_torque);
    // << END CAN

    // Sensor class
    UNIT_SCALES scale;

    // FUNCTIONS
    // >> Servo Config
    void set_controlled_servo_id(uint8_t servo_id);
    void change_controlled_servo_id(uint8_t servo_id);
    void set_servo_power(uint8_t servo_id, bool is_power_on);
    void set_servo_ctrl_mode(uint8_t servo_id,
                             basic_servo_ctrl_cmd_list ctrl_mode);
    void set_servo_position(uint8_t servo_id, double target_position);
    void set_servo_velocity(uint8_t servo_id, double target_velocity);
    void set_servo_torque(uint8_t servo_id, double target_torque);

public:
    ControlManager();
    ~ControlManager();
    // FUNCTIONS
    // >> System
    std::shared_ptr<node_cmd> get_cmd_ptr();
    void update();
    void cmd_executor();
    void set_emergency_stop(bool em_stop);
    // << END System

    // >> OBSERVER
    void get_control_state(ControlState& state);
    void set_state_machine(node_state_machine state_machine);
    bool check_init_all();
    // << END OBSERVER
    void init_servo_dummy();

    // >> SENSOR >> weight scale
    bool begin_scale(TwoWire& wire, uint8_t addr);
    void reset_scale();
    float get_weight();
    int32_t get_weightRawADC();
    // << END SENSOR

    // >> SENSOR >> CAN
    void init_twai(uint8_t tx_num, uint8_t rx_num);
    void send_can_packet_task(const twai_message_t& packet);
    bool recv_can_packet_task(twai_message_t& packet);
    void set_send_packet(const twai_message_t& packet);
    void init_motor_driver() {
        cybergear_driver.set_master_can_id(0x00);
        init_motor_status();
    }
    // << END SENSOR
};