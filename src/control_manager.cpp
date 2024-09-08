#include "control_manager.hpp"

#include "logger.hpp"

void ControlManager::update_cybergear_status(twai_message_t& rx_msg) {
    uint8_t can_id = rx_msg.identifier;
    if (motor_status.find(can_id) == motor_status.end()) {
        motor_status[can_id] = XiaomiCyberGearStatus();
        motor_status[can_id].can_id = can_id;
    }

    XiaomiCyberGearStatus& status = motor_status[can_id];
    cybergear_driver.update_ram_data(rx_msg, status);
    /*
    M5_LOGI("ControlManager::update_cybergear_status: can_id: %d", can_id);
    M5_LOGI("ControlManager::update_cybergear_status: ctrl_mode: %d",
            status.ctrl_mode);
    M5_LOGI("ControlManager::update_cybergear_status: rotation: %d",
            status.rotation);
    M5_LOGI("ControlManager::update_cybergear_status: mech_pos: %f",
            status.mech_pos);
    M5_LOGI("ControlManager::update_cybergear_status: mech_vel: %f",
            status.mech_vel);
    M5_LOGI("ControlManager::update_cybergear_status: vbus: %f", status.vbus);
    M5_LOGI("ControlManager::update_cybergear_status: iqf: %f", status.iqf);
    M5_LOGI("ControlManager::update_cybergear_status: pos_ref: %f",
            status.pos_ref);
    M5_LOGI("ControlManager::update_cybergear_status: vel_ref: %f",
            status.vel_ref);
    M5_LOGI("ControlManager::update_cybergear_status: cur_ref: %f",
            status.cur_ref);
    M5_LOGI("ControlManager::update_cybergear_status: pos_kp: %f",
            status.pos_kp);
    M5_LOGI("ControlManager::update_cybergear_status: vel_kp: %f",
            status.vel_kp);
    M5_LOGI("ControlManager::update_cybergear_status: vel_ki: %f",
            status.vel_ki);
    M5_LOGI("ControlManager::update_cybergear_status: cur_kp: %f",
            status.cur_kp);
    M5_LOGI("ControlManager::update_cybergear_status: cur_ki: %f",
            status.cur_ki);
    M5_LOGI("ControlManager::update_cybergear_status: cur_filter_gain: %f",
            status.cur_filter_gain);
            */
}

void ControlManager::get_motor_id(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.get_motor_id(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::recv_can() {
    while (is_waiting_rx_can) {
        twai_message_t rx_msg;
        if (recv_can_packet_task(rx_msg)) {
            is_waiting_rx_can = false;
        }
        // recv_can_packet_task(rx_msg);
        // uint8_t recieved_master_can_id = rx_msg.identifier & 0x000000FF;
        // M5_LOGI("ControlManager::update: recieved_master_can_id: %d",
        //        recieved_master_can_id);
        uint8_t can_id = (rx_msg.identifier & 0x0000FF00) >> 8;
        M5_LOGI("ControlManager::update: can_id: %d", can_id);
        if ((can_id) == 0x10) {
            update_cybergear_status(rx_msg);
            is_waiting_rx_can = false;
            is_connecting_servo = true;
        }
    }
}

ControlManager::ControlManager() {
    ctrl_cmd_ = std::make_shared<node_cmd>(MAX_CTRL_CMD_STACK_SIZE);
}

ControlManager::~ControlManager() {
}

std::shared_ptr<node_cmd> ControlManager::get_cmd_ptr() {
    return ctrl_cmd_;
}

void ControlManager::update() {
    if (state_machine_ != node_state_machine::STABLE) {
        state_.is_power_on = false;
    }

    if (state_machine_ == node_state_machine::READY && !is_connecting_servo) {
        get_motor_id(0x10);
        is_waiting_rx_can = true;
    }

    if (state_.is_power_on) {
        switch (state_.ctrl_mode) {
            case basic_servo_ctrl_cmd_list::STAY:
                break;
            case basic_servo_ctrl_cmd_list::POSITION:
                state_.act_joint_position += 0.05 * (state_.cmd_joint_position -
                                                     state_.act_joint_position);
                break;
            case basic_servo_ctrl_cmd_list::VELOCITY:
                state_.act_joint_velocity += 0.05 * (state_.cmd_joint_velocity -
                                                     state_.act_joint_velocity);
                break;
            case basic_servo_ctrl_cmd_list::TORQUE:
                state_.act_joint_torque +=
                    0.05 * (state_.cmd_joint_torque - state_.act_joint_torque);
                break;
            default:
                break;
        }
    } else {
        state_.cmd_joint_velocity = 0.0;
        state_.cmd_joint_torque = 0.0;
        state_.cmd_joint_position = state_.act_joint_position;
        state_.act_joint_velocity +=
            0.05 * (state_.cmd_joint_velocity - state_.act_joint_velocity);
        state_.act_joint_torque +=
            0.05 * (state_.cmd_joint_torque - state_.act_joint_torque);
    }

    recv_can();
}

void ControlManager::cmd_executor() {
    st_node_cmd cmd;
    cmd_change_controlled_servo_id* temp_data_1;
    cmd_change_servo_power* temp_data_2;
    cmd_change_servo_ctrl_mode* temp_data_3;
    cmd_position_control* temp_data_4;
    cmd_velocity_control* temp_data_5;
    cmd_torque_control* temp_data_6;

    if (ctrl_cmd_->cmd_stack_.size() != 0) {
        cmd = ctrl_cmd_->cmd_stack_.pop();

        M5_LOGI("ControlManager::cmd_executor: cmd_type: %d",
                cmd.cmd_code.cmd_type);

        switch (cmd.cmd_code.cmd_type) {
            case basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID:
                temp_data_1 = (cmd_change_controlled_servo_id*)cmd.data;
                set_controlled_servo_id(temp_data_1->servo_id);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_POWER:
                temp_data_2 = (cmd_change_servo_power*)cmd.data;
                set_servo_power(temp_data_2->servo_id, temp_data_2->is_on);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_CTRLMODE:
                temp_data_3 = (cmd_change_servo_ctrl_mode*)cmd.data;
                set_servo_ctrl_mode(temp_data_3->servo_id,
                                    temp_data_3->ctrl_mode);
                break;
            case basic_m5stack_cmd_list::SERVO_POSITION_CONTROL:
                temp_data_4 = (cmd_position_control*)cmd.data;
                set_servo_position(temp_data_4->servo_id,
                                   temp_data_4->target_position);
                break;
            case basic_m5stack_cmd_list::SERVO_VELOCITY_CONTROL:
                temp_data_5 = (cmd_velocity_control*)cmd.data;
                set_servo_velocity(temp_data_5->servo_id,
                                   temp_data_5->velocity);
                break;
            case basic_m5stack_cmd_list::SERVO_TORQUE_CONTROL:
                temp_data_6 = (cmd_torque_control*)cmd.data;
                set_servo_torque(temp_data_6->servo_id, temp_data_6->torque);
                break;
            case basic_m5stack_cmd_list::CHANGE_SM_FORCE_STOP:
                break;
            default:
                break;
        }

    } else {
        return;
    }
}

void ControlManager::init_servo_dummy() {
    state_.is_init_joint_pos = true;
    state_.act_joint_position = 0.0;
    state_.act_joint_velocity = 0.0;
    state_.act_joint_torque = 0.0;
}

void ControlManager::get_control_state(ControlState& state) {
    state = state_;
}

void ControlManager::set_state_machine(node_state_machine state_machine) {
    state_machine_ = state_machine;
}

bool ControlManager::check_init_all() {
    return is_init_all;
}

/*
 * WEIGHT SCALE
 */
bool ControlManager::begin_scale(TwoWire& wire, uint8_t addr) {
    uint8_t watchdog = 0;
    while (!scale.begin(&wire, addr)) {
        M5DEV_LOGE("Scale not found");
        status_.is_init_scale = false;
        delay(200);
        watchdog++;
        if (watchdog > 5) {
            M5DEV_LOGE("Scale not found. Break");
            return false;
        }
    }
    M5DEV_LOGI("Scale found");
    status_.is_init_scale = true;

    scale.setLEDColor(0x001000);
    // scale.setLPFilter(50);
    // scale.setAvgFilter(10);
    scale.setEmaFilter(50);
    scale.setGapValue(200.0f);
    scale.setOffset();
    // scale.setGapValue(0.0f);
    return true;
}
void ControlManager::reset_scale() {
    scale.setOffset();
    // scale.setGapValue(0.0f);
}
float ControlManager::get_weight() {
    if (!status_.is_init_scale) {
        M5_LOGW("Scale not found");
        return -1.0f;
    }
    if (scale.getBtnStatus() == 1) {
        scale.setLEDColor(0x001000);
    } else {
        scale.setLEDColor(0x000010);
    }
    M5DEV_LOGI("Scale Gap Value: %f", scale.getGapValue());
    float weight = scale.getWeight();
    state_.sensor_weight = weight;
    return weight;
}
int32_t ControlManager::get_weightRawADC() {
    if (!status_.is_init_scale) {
        return -1.0;
    }
    int32_t raw_adc = scale.getRawADC();
    state_.sensor_weight_raw_adc = raw_adc;
    return raw_adc;
}

void ControlManager::set_emergency_stop(bool em_stop) {
    if (em_stop) {
        state_.is_power_on = false;
    }
}

/*
 * CAN
 */
void ControlManager::init_twai(uint8_t tx_num, uint8_t rx_num) {
    gpio_num_t TX_GPIO_NUM = gpio_num_t(tx_num);
    gpio_num_t RX_GPIO_NUM = gpio_num_t(rx_num);

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}
void ControlManager::send_can_packet_task(const twai_message_t& packet) {
    twai_transmit(&packet, portMAX_DELAY);
    set_send_packet(packet);
}
bool ControlManager::recv_can_packet_task(twai_message_t& packet) {
    auto ret = twai_receive(&packet, portMAX_DELAY);
    if (ret != ESP_OK) {
        M5_LOGE(
            "ControlManager::recv_can_packet_task: Failed to receive CAN "
            "packet");
        return false;
    }
    update_cybergear_status(packet);
    rx_can_cnt++;
    set_receive_packet(packet);
    return true;
}
void ControlManager::set_receive_packet(const twai_message_t& packet) {
    receive_can_packet = packet;
    receive_can_packet_buffer.push(packet);
}
void ControlManager::set_send_packet(const twai_message_t& packet) {
    send_can_packet = packet;
    send_can_packet_buffer.push(packet);
}

void ControlManager::set_controlled_servo_id(uint8_t servo_id) {
    state_.servo_id = servo_id;
}
void ControlManager::set_servo_power(uint8_t servo_id, bool is_power_on) {
    if (is_power_on && state_.is_init_joint_pos) {
        state_.cmd_joint_position = state_.act_joint_position;
        state_.cmd_joint_velocity = 0.0;
        state_.cmd_joint_torque = 0.0;  // TODO: Implement

        state_.is_power_on = true;
    } else {
        state_.is_power_on = false;
    }
}
void ControlManager::set_servo_ctrl_mode(uint8_t servo_id,
                                         basic_servo_ctrl_cmd_list ctrl_mode) {
    state_.ctrl_mode = ctrl_mode;
}
void ControlManager::set_servo_velocity(uint8_t servo_id,
                                        double target_velocity) {
    state_.cmd_joint_velocity = target_velocity;
}
void ControlManager::set_servo_torque(uint8_t servo_id, double target_torque) {
    state_.cmd_joint_torque = target_torque;
}
void ControlManager::set_servo_position(uint8_t servo_id,
                                        double target_position) {
    state_.cmd_joint_position = target_position;
}
// << END WEIGHT SCALE
