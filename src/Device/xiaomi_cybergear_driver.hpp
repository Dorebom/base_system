#pragma once

#include <cstdint>

#include "driver/twai.h"
#include "logger.hpp"
#include "xiaomi_cybergear_defs.hpp"

// Longduino/cores/arduino/GD32VF103_Firmware_Library/GD32VF103_standard_peripheral/Include/gd32vf103_can.h
/* frame type */
#define CAN_FT_DATA   ((uint32_t)0x00000000U) /*!< data frame */
#define CAN_FT_REMOTE ((uint32_t)0x00000002U) /*!< remote frame */

/* CAN frame format */
#define CAN_FF_STANDARD ((uint32_t)0x00000000U) /*!< standard frame */
#define CAN_FF_EXTENDED ((uint32_t)0x00000004U) /*!< extended frame */

struct XiaomiCyberGearStatus
{
    uint32_t timestamp;

    float pos_ref;
    float vel_ref;
    float cur_ref;

    float mech_pos;
    float mech_vel;
    float vbus;
    float iqf;

    float pos_kp;
    float vel_kp;
    float vel_ki;
    float cur_kp;
    float cur_ki;
    float cur_filter_gain;

    float limit_vel;
    float limit_torque;
    float limit_cur;

    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_effort;
    uint16_t raw_temperature;

    float act_position;
    float act_velocity;
    float act_effort;
    float act_temperature;

    int16_t rotation;
    uint8_t ctrl_mode;

    uint16_t can_id;
    uint64_t serial_id;

    XiaomiCyberGearStatus() {
        timestamp = 0;

        pos_ref = 0.0f;
        vel_ref = 0.0f;
        cur_ref = 0.0f;

        mech_pos = 0.0f;
        mech_vel = 0.0f;
        vbus = 0.0f;
        iqf = 0.0f;

        pos_kp = 0.0f;
        vel_kp = 0.0f;
        vel_ki = 0.0f;
        cur_kp = 0.0f;
        cur_ki = 0.0f;
        cur_filter_gain = 0.0f;

        limit_vel = 0.0f;
        limit_torque = 0.0f;
        limit_cur = 0.0f;

        raw_position = 0;
        raw_velocity = 0;
        raw_effort = 0;
        raw_temperature = 0;

        act_position = 0.0f;
        act_velocity = 0.0f;
        act_effort = 0.0f;
        act_temperature = 0.0f;

        rotation = 0;
        ctrl_mode = 0;

        can_id = 0;
        serial_id = 0;
    }
};

class XiaomiCyberGearDriver {
private:
    uint8_t master_can_id;

    void generate_twai_message(twai_message_t &msg, uint8_t can_id,
                               uint8_t cmd_id, uint16_t option,
                               uint8_t data_length, uint8_t *data) {
        uint32_t id = cmd_id << 24 | option << 8 | can_id;

        // msg.extd = id;
        msg.flags = TWAI_MSG_FLAG_EXTD;
        msg.extd = 1;
        msg.identifier = id;
        msg.data_length_code = data_length;
        for (int i = 0; i < data_length; i++) {
            msg.data[i] = data[i];
        }
    }

    void generate_float_twai_message(twai_message_t &msg, uint8_t can_id,
                                     uint16_t addr, float cmd_value,
                                     float min_value, float max_value) {
        uint8_t data[8] = {0x00};
        data[0] = addr & 0x00FF;
        data[1] = addr >> 8;

        float value = (max_value < cmd_value) ? max_value : cmd_value;
        value = (min_value > value) ? min_value : value;
        memcpy(&data[4], &value, sizeof(float));
        generate_twai_message(msg, can_id, CMD_RAM_WRITE, master_can_id, 8,
                              data);
    }

    float translate_uint2float(uint16_t u_value, float min_value,
                               float max_value) {
        uint16_t type_max = 0xFFFF;
        float span = max_value - min_value;
        return min_value + span * (float)u_value / type_max;
    }

    uint16_t translate_float2uint(float f_value, float min_value,
                                  float max_value, int bits) {
        if (bits > 16)
            bits = 16;
        float span = max_value - min_value;
        if (f_value < min_value)
            f_value = min_value;
        else if (f_value > max_value)
            f_value = max_value;
        return (uint16_t)((f_value - min_value) * ((float)((1 << bits) - 1)) /
                          span);
    }

    /*
     * Read RAM Value
     */
    void read_ram_cmd(uint8_t can_id, uint16_t addr, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        memcpy(&data[0], &addr, sizeof(uint16_t));
        generate_twai_message(msg, can_id, CMD_RAM_READ, master_can_id, 8,
                              data);
    }

    void set_control_mode(uint8_t can_id, uint8_t mode, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        data[0] = ADDR_RUN_MODE & 0x00FF;
        data[1] = ADDR_RUN_MODE >> 8;
        data[4] = mode;
        generate_twai_message(msg, can_id, CMD_RAM_WRITE, master_can_id, 8,
                              data);
    }

public:
    XiaomiCyberGearDriver() : master_can_id(0x00) {
    }
    virtual ~XiaomiCyberGearDriver() {
    }

    // Initialize
    void set_master_can_id(uint8_t master_can_id_) {
        master_can_id = master_can_id_;
    }

    /*
     * Cybergear Motor Driver API
     *
     * 0: Get Motor ID
     * 1: Motion Control (The control API will be explained later.)
     * 2: Response Motion State
     * 3: Enable Motor
     * 4: Stop Motor
     * 6: Set Mechanical Position to Zero
     * 7: Change CAN ID
     *21: Get Motor Fail
     */
    // 0. Get Motor ID
    void get_motor_id(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_GET_ID, master_can_id, 8, data);
    }
    // 3. Motion Control
    void enable_motor(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_ENABLE, master_can_id, 8, data);
    }
    // 4. Stop Motor
    void stop_motor(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_STOP, master_can_id, 8, data);
    }
    // 6. Set Mechanical Position to Zero
    void set_mechanical_position_to_zero(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        data[0] = 0x01;
        generate_twai_message(msg, can_id, CMD_SET_MECH_POSITION_TO_ZERO,
                              master_can_id, 8, data);
    }
    // 7. Change CAN ID
    void change_can_id(uint8_t can_id, uint8_t new_can_id,
                       twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        uint16_t option = new_can_id << 8 | master_can_id;
        generate_twai_message(msg, can_id, CMD_CHANGE_CAN_ID, option, 8, data);
    }
    // 21. Get Motor Fail (Not implemented yet)
    void get_motor_fail(uint8_t can_id, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        generate_twai_message(msg, can_id, CMD_GET_MOTOR_FAIL, master_can_id, 8,
                              data);
    }

    /*
     * Control API
     *
     * 0: MODE_MOTION
     *    - enable_motor
     *    - motion_control
     * 1: MODE_POSITION
     *    - set_control_mode
     *    - enable_motor
     *    - set_limit_spd
     *    - set_loc_ref
     * 2: MODE_SPEED
     *    - set_control_mode
     *    - enable_motor
     *    - set_limit_cur
     *    - set_spd_ref
     * 3: MODE_CURRENT
     *    - set_control_mode
     *    - enable_motor
     *    - set_iq_ref
     */
    void set_position_mode(uint8_t can_id, twai_message_t &msg) {
        set_control_mode(can_id, MODE_POSITION, msg);
    }
    void set_velocity_mode(uint8_t can_id, twai_message_t &msg) {
        set_control_mode(can_id, MODE_SPEED, msg);
    }
    void set_current_mode(uint8_t can_id, twai_message_t &msg) {
        set_control_mode(can_id, MODE_CURRENT, msg);
    }
    void set_motion_mode(uint8_t can_id, twai_message_t &msg) {
        set_control_mode(can_id, MODE_MOTION, msg);
    }
    // 0. Mode Motion
    // After the motor is powered on, it is in the operation control mode by
    // default;
    void motion_control(uint8_t can_id, float position, float velocity,
                        float torque, float kp, float kd, twai_message_t &msg) {
        uint8_t data[8] = {0x00};
        uint16_t position_ =
            translate_float2uint(position, POS_MIN, POS_MAX, 16);
        uint16_t velocity_ =
            translate_float2uint(velocity, VEL_MIN, VEL_MAX, 16);
        uint16_t torque_ = translate_float2uint(torque, T_MIN, T_MAX, 16);
        uint16_t kp_ = translate_float2uint(kp, MP_KP_MIN, MP_KP_MAX, 16);
        uint16_t kd_ = translate_float2uint(kd, MV_KD_MIN, MV_KD_MAX, 16);

        data[0] = position_ >> 8;
        data[1] = position_ & 0x00FF;
        data[2] = velocity_ >> 8;
        data[3] = velocity_ & 0x00FF;
        data[4] = kp_ >> 8;
        data[5] = kp_ & 0x00FF;
        data[6] = kd_ >> 8;
        data[7] = kd_ & 0x00FF;
        generate_twai_message(msg, can_id, CMD_MOTION_CTRL, torque_, 8, data);
    }

    // set reference
    void set_position_ref(uint8_t can_id, float pos_ref, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_POSITION_REF, pos_ref,
                                    POS_MIN, POS_MAX);
    }
    void set_spd_ref(uint8_t can_id, float spd_ref, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_SPEED_REF, spd_ref,
                                    SPD_REF_MIN, SPD_REF_MAX);
    }
    void set_iq_ref(uint8_t can_id, float iq_ref, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_IQ_REF, iq_ref,
                                    IQ_REF_MIN, IQ_REF_MAX);
    }

    // set limit
    void set_limit_spd(uint8_t can_id, float limit_spd, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_LIMIT_SPEED, limit_spd,
                                    LIMIT_SPD_MIN, LIMIT_SPD_MAX);
    }
    void set_limit_torque(uint8_t can_id, float limit_torque,
                          twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_LIMIT_TORQUE,
                                    limit_torque, LIMIT_TORQUE_MIN,
                                    LIMIT_TORQUE_MAX);
    }
    void set_limit_cur(uint8_t can_id, float limit_cur, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_LIMIT_CURRENT, limit_cur,
                                    IQ_MIN, IQ_MAX);
    }

    // set gain
    void set_pos_kp(uint8_t can_id, float pos_kp, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_POSITION_KP, pos_kp,
                                    POS_KP_MIN, POS_KP_MAX);
    }
    void set_vel_kp(uint8_t can_id, float vel_kp, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_SPEED_KP, vel_kp,
                                    SPD_KP_MIN, SPD_KP_MAX);
    }
    void set_vel_ki(uint8_t can_id, float vel_ki, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_CURRENT_KP, vel_ki,
                                    SPD_KI_MIN, SPD_KI_MAX);
    }
    void set_cur_kp(uint8_t can_id, float cur_kp, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_CURRENT_KP, cur_kp,
                                    CUR_KP_MIN, CUR_KP_MAX);
    }
    void set_cur_ki(uint8_t can_id, float cur_ki, twai_message_t &msg) {
        generate_float_twai_message(msg, can_id, ADDR_CURRENT_KI, cur_ki,
                                    CUR_KI_MIN, CUR_KI_MAX);
    }
    // set filter
    void set_velocity_filter_gain(uint8_t can_id, float velocity_filter_gain,
                                  twai_message_t &msg) {
        // Not implemented yet
        // generate_float_twai_message(msg, can_id, ADDR_SPEED_FILTER_GAIN,
        //                            velocity_filter_gain,
        //                            SPEED_FILTER_GAIN_MIN,
        //                            SPEED_FILTER_GAIN_MAX);
    }
    void set_current_filter_gain(uint8_t can_id, float current_filter_gain,
                                 twai_message_t &msg) {
        generate_float_twai_message(
            msg, can_id, ADDR_CURRENT_FILTER_GAIN, current_filter_gain,
            CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX);
    }

    // set rotation
    void set_rotation(uint8_t can_id, float rotation, twai_message_t &msg) {
        // Not implemented yet
        // generate_float_twai_message(msg, can_id, ADDR_ROTATION, rotation,
        //                            ROTATION_MIN, ROTATION_MAX);
    }

    // get value
    void get_mech_position(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_MECH_POS, msg);
    }
    void get_mech_velocity(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_MECH_VEL, msg);
    }
    void get_vbus(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_VBUS, msg);
    }
    void get_iqf(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_IQF, msg);
    }
    void get_rotation(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_ROTATION, msg);
    }
    //
    void get_ctrl_mode(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_RUN_MODE, msg);
    }
    void get_pos_ref(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_POSITION_REF, msg);
    }
    void get_vel_ref(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_SPEED_REF, msg);
    }
    void get_iq_ref(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_IQ_REF, msg);
    }
    void get_limit_velocity(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_LIMIT_SPEED, msg);
    }
    void get_limit_torque(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_LIMIT_TORQUE, msg);
    }
    void get_limit_current(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_LIMIT_CURRENT, msg);
    }
    void get_pos_kp(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_POSITION_KP, msg);
    }
    void get_vel_kp(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_SPEED_KP, msg);
    }
    void get_vel_ki(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_SPEED_KI, msg);
    }
    void get_cur_kp(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_CURRENT_KP, msg);
    }
    void get_cur_ki(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_CURRENT_KI, msg);
    }
    void get_cur_filter_gain(uint8_t can_id, twai_message_t &msg) {
        read_ram_cmd(can_id, ADDR_CURRENT_FILTER_GAIN, msg);
    }

    bool check_motor_id(twai_message_t &msg, uint8_t &can_id) {
        uint8_t recieved_master_can_id = msg.identifier & 0x000000FF;
        if (recieved_master_can_id == master_can_id) {
            can_id = (msg.identifier & 0x0000FF00) >> 8;
            return true;
        }
        return false;
    }

    bool check_serial_response(twai_message_t &msg, uint8_t &can_id,
                               uint64_t &serial_id) {
        uint8_t recieved_serial_response_cmd = msg.identifier & 0x000000FF;
        if (recieved_serial_response_cmd == 0xFE) {
            can_id = (msg.identifier & 0x0000FF00) >> 8;
            serial_id =
                (uint64_t)msg.data[7] << 56 | (uint64_t)msg.data[6] << 48 |
                (uint64_t)msg.data[5] << 40 | (uint64_t)msg.data[4] << 32 |
                (uint64_t)msg.data[3] << 24 | (uint64_t)msg.data[2] << 16 |
                (uint64_t)msg.data[1] << 8 | (uint64_t)msg.data[0];
            return true;
        }
        return false;
    }

    bool update_motor_status(twai_message_t &msg,
                             XiaomiCyberGearStatus &status) {
        uint8_t packet_type = (msg.identifier & 0x3F000000) >> 24;
        if (packet_type == CMD_RESPONSE_MOTION_STATE) {
            M5_LOGI("CMD_RESPONSE_MOTION_STATE");
            update_ctrl_state(msg, status);
        } else if (packet_type == CMD_RAM_READ) {
            M5_LOGI("CMD_RAM_READ: 0X%02X", status.can_id);
            update_ram_data(msg, status);
        } else if (packet_type == CMD_GET_MOTOR_FAIL) {
            // not implemented yet
        } else {
            return false;
        }
        return true;
    }

    bool update_ctrl_state(twai_message_t &msg,
                           XiaomiCyberGearStatus &motor_status) {
        motor_status.raw_position = msg.data[1] | msg.data[0] << 8;
        motor_status.raw_velocity = msg.data[3] | msg.data[2] << 8;
        motor_status.raw_effort = msg.data[5] | msg.data[4] << 8;
        motor_status.raw_temperature = msg.data[7] | msg.data[6] << 8;

        motor_status.act_position =
            translate_uint2float(motor_status.raw_position, POS_MIN, POS_MAX);
        motor_status.act_velocity = translate_uint2float(  // 0.1 deg/s
            motor_status.raw_velocity, VEL_MIN, VEL_MAX);
        motor_status.act_effort =
            translate_uint2float(motor_status.raw_effort, T_MIN, T_MAX);
        motor_status.act_temperature =
            (float)motor_status.raw_temperature * 0.1;
        motor_status.timestamp = micros();

        return true;
    }

    bool update_ram_data(twai_message_t &msg, XiaomiCyberGearStatus &status) {
        uint16_t addr_ = (msg.data[1] << 8) | msg.data[0];

        uint8_t uint8_data_;  // for control mode
        memcpy(&uint8_data_, &msg.data[4], sizeof(uint8_t));
        int16_t int16_data_;  // for rotation
        memcpy(&int16_data_, &msg.data[4], sizeof(int16_t));
        float float_data_;  // otherwise
        memcpy(&float_data_, &msg.data[4], sizeof(float));

        switch (addr_) {
            case ADDR_RUN_MODE:
                status.ctrl_mode = uint8_data_;
                break;
            case ADDR_ROTATION:
                status.rotation = int16_data_;
                break;
            case ADDR_MECH_POS:
                status.mech_pos = float_data_;
                break;
            case ADDR_MECH_VEL:
                status.mech_vel = float_data_;
                break;
            case ADDR_VBUS:
                status.vbus = float_data_;
                break;
            case ADDR_IQF:
                status.iqf = float_data_;
                break;
            case ADDR_POSITION_REF:
                status.pos_ref = float_data_;
                break;
            case ADDR_SPEED_REF:
                status.vel_ref = float_data_;
                break;
            case ADDR_IQ_REF:
                status.cur_ref = float_data_;
                break;
            case ADDR_POSITION_KP:
                status.pos_kp = float_data_;
                break;
            case ADDR_SPEED_KP:
                status.vel_kp = float_data_;
                break;
            case ADDR_SPEED_KI:
                status.vel_ki = float_data_;
                break;
            case ADDR_CURRENT_KP:
                status.cur_kp = float_data_;
                break;
            case ADDR_CURRENT_KI:
                status.cur_ki = float_data_;
                break;
            case ADDR_CURRENT_FILTER_GAIN:
                status.cur_filter_gain = float_data_;
                break;
            case ADDR_LIMIT_SPEED:
                status.limit_vel = float_data_;
                break;
            case ADDR_LIMIT_TORQUE:
                status.limit_torque = float_data_;
                break;
            case ADDR_LIMIT_CURRENT:
                status.limit_cur = float_data_;
                break;
            default:
                return false;
                break;
        }
        status.timestamp = micros();

        return true;
    }
};