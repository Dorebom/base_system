#include "control_manager.hpp"

#include "logger.hpp"

ControlManager::ControlManager() {
    ctrl_cmd_ = std::make_shared<node_cmd>(MAX_CTRL_CMD_STACK_SIZE);

    trq_lpf_.set_param_lpf(1000.0, 100.0, 0.7);
    vel_lpf_.set_param_lpf(1000.0, 100.0, 0.7);
}

ControlManager::~ControlManager() {
}

void ControlManager::update() {
    if (state_machine_ != node_state_machine::STABLE) {
        state_.is_power_on = false;
    }

    // CAN(Cybergear) Connected
    if (state_.act_can_connection_status) {
        if (state_.is_power_on) {
            // M5_LOGI("Ctrl Mode: %d", state_.ctrl_mode);
            switch (state_.ctrl_mode) {
                case basic_servo_ctrl_cmd_list::STAY:
                    stay_motor(state_.servo_id);
                    break;
                case basic_servo_ctrl_cmd_list::POSITION:
                    position_control(state_.servo_id,
                                     state_.cmd_joint_position);
                    break;
                case basic_servo_ctrl_cmd_list::VELOCITY:
                    velocity_control(state_.servo_id,
                                     state_.cmd_joint_velocity);
                    break;
                case basic_servo_ctrl_cmd_list::TORQUE:
                    torque_control(state_.servo_id, state_.cmd_joint_torque);
                    break;
                default:
                    break;
            }
        } else {
            stop_motor(state_.servo_id);
        }
    } else {  // CAN(Cybergear) is not Connected
        if (!state_.is_init_joint_pos) {
            init_servo_dummy();
        }

        if (state_.is_power_on) {
            switch (state_.ctrl_mode) {
                case basic_servo_ctrl_cmd_list::STAY:
                    break;
                case basic_servo_ctrl_cmd_list::POSITION:
                    state_.act_joint_position +=
                        0.05 *
                        (state_.cmd_joint_position - state_.act_joint_position);
                    break;
                case basic_servo_ctrl_cmd_list::VELOCITY:
                    state_.act_joint_velocity +=
                        0.05 *
                        (state_.cmd_joint_velocity - state_.act_joint_velocity);
                    break;
                case basic_servo_ctrl_cmd_list::TORQUE:
                    state_.act_joint_torque += 0.05 * (state_.cmd_joint_torque -
                                                       state_.act_joint_torque);
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
    }
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

        // M5_LOGI("ControlManager::cmd_executor: cmd_type: %d",
        //         cmd.cmd_code.cmd_type);

        switch (cmd.cmd_code.cmd_type) {
            case basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID:
                temp_data_1 = (cmd_change_controlled_servo_id*)cmd.data;
                change_controlled_servo_id(temp_data_1->servo_id);
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
            // case basic_m5stack_cmd_list::CONNECT_CAN:
            //     if (!state_.act_can_connection_status) {
            //         state_.act_can_connection_status =
            //         check_connecting_servo();
            //     }
            //     break;
            case basic_m5stack_cmd_list::DISCONNECT_CAN:
                state_.act_can_connection_status = false;
                break;
            case basic_m5stack_cmd_list::CHANGE_SM_FORCE_STOP:
                break;
            default:
                break;
        }
    }
}

std::shared_ptr<node_cmd> ControlManager::get_cmd_ptr() {
    return ctrl_cmd_;
}

bool ControlManager::check_init_all() {
    return is_init_all;
}

void ControlManager::set_emergency_stop(bool em_stop) {
    if (em_stop) {
        state_.is_power_on = false;
    }
}

void ControlManager::get_control_state(ControlState& state) {
    state = state_;
}

void ControlManager::set_state_machine(node_state_machine state_machine) {
    state_machine_ = state_machine;
}

// ちゃんとつながってデータも合ってたらtrueを返す
bool ControlManager::recv_can() {
    twai_message_t rx_msg;
    bool is_recv_can = recv_can_packet_task(rx_msg);

    uint8_t can_id;
    XiaomiCyberGearStatus* motor_status_;
    uint8_t msg_cmd;

    if (is_recv_can) {
        // 受信データの解析
        can_id = (rx_msg.identifier & 0x0000FF00) >> 8;
        motor_status_ = &motor_status[can_id];
        msg_cmd = (rx_msg.identifier & 0xFF000000) >> 24;

        // ここで、mapに登録がない場合は、登録する
        if (motor_status.find(can_id) == motor_status.end()) {
            motor_status[can_id] = XiaomiCyberGearStatus();
            motor_status[can_id].can_id = can_id;
        }

        //  　ここで、受信データの詳細解析を行う
        switch (msg_cmd) {
            case CMD_GET_ID:
                if (!motor_status_->is_connected) {
                    if (((rx_msg.identifier & 0x000000FF) == 0xFE) &&
                        can_id == state_.waiting_servo_id) {
                        // M5_LOGI(
                        //     "ControlManager::check_connecting_servo:
                        //     Connected "
                        //     "(%d)",
                        //     can_id);
                        motor_status_->is_connected = true;
                        motor_status_->serial_id =
                            (uint64_t)rx_msg.data[7] << 56 |
                            (uint64_t)rx_msg.data[6] << 48 |
                            (uint64_t)rx_msg.data[5] << 40 |
                            (uint64_t)rx_msg.data[4] << 32 |
                            (uint64_t)rx_msg.data[3] << 24 |
                            (uint64_t)rx_msg.data[2] << 16 |
                            (uint64_t)rx_msg.data[1] << 8 |
                            (uint64_t)rx_msg.data[0];
                        return true;
                    } else {
                        // M5_LOGI(
                        //     "ControlManager::check_connecting_servo: Not "
                        //     "Connected "
                        //     "(%d)",
                        //     can_id);
                        motor_status_->is_connected = false;
                        return false;
                    }
                }
                // M5_LOGI(
                //     "ControlManager::check_connecting_servo: Already
                //     Connected "
                //     "(%d)",
                //     can_id);
                break;
            case CMD_RESPONSE_MOTION_STATE:
                // RAW
                motor_status_->raw_position = rx_msg.data[1] | rx_msg.data[0]
                                                                   << 8;
                motor_status_->raw_velocity = rx_msg.data[3] | rx_msg.data[2]
                                                                   << 8;
                motor_status_->raw_effort = rx_msg.data[5] | rx_msg.data[4]
                                                                 << 8;
                motor_status_->raw_temperature = rx_msg.data[7] | rx_msg.data[6]
                                                                      << 8;
                // RAW -> ACT
                motor_status_->act_position =
                    cybergear_driver.translate_uint2float(
                        motor_status_->raw_position, POS_MIN, POS_MAX);
                motor_status_->act_velocity =
                    cybergear_driver.translate_uint2float(
                        motor_status_->raw_velocity, VEL_MIN, VEL_MAX);
                motor_status_->act_effort =
                    cybergear_driver.translate_uint2float(
                        motor_status_->raw_effort, T_MIN, T_MAX);
                motor_status_->act_temperature =
                    (float)motor_status_->raw_temperature * 0.1;
                // TIMESTAMP
                motor_status_->timestamp = micros();
                //
                motor_status_->master_id = (rx_msg.identifier & 0x000000FF);

                state_.timestamp = motor_status_->timestamp;
                state_.prev_joint_position = state_.act_joint_position;
                state_.act_joint_position = motor_status_->act_position;
                state_.act_joint_velocity =
                    vel_lpf_.update(motor_status_->act_velocity);
                state_.act_joint_torque =
                    trq_lpf_.update(motor_status_->act_effort);

                if (!state_.is_init_joint_pos) {
                    state_.is_init_joint_pos = true;
                }
                return true;
                break;
            case CMD_RAM_READ:
                /* code */
                break;
            case CMD_GET_MOTOR_FAIL:
                /* code */
                break;
            default:
                break;
        }
    } else {
        M5_LOGW(
            "ControlManager::check_connecting_servo: Failed to receive CAN "
            "packet");
        return false;
    }
    return false;
}

void ControlManager::stop_motor(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.stop_motor(can_id, msg);
    send_can_packet_task(msg);
    recv_can();
    state_.is_power_on = false;
}

void ControlManager::stay_motor(uint8_t can_id) {
    twai_message_t msg;
    // cybergear_driver.set_position_mode(can_id, msg);
    // send_can_packet_task(msg);
    // recv_can();

    cybergear_driver.set_position_ref(can_id, state_.act_joint_position_0, msg);
    send_can_packet_task(msg);
    recv_can();
}

void ControlManager::position_control(uint8_t can_id, double target_position) {
    // SAFETY
    if (target_position < MIN_THRESHOLD_JOINT_POSITION) {
        target_position = MIN_THRESHOLD_JOINT_POSITION;
    } else if (target_position > MAX_THRESHOLD_JOINT_POSITION) {
        target_position = MAX_THRESHOLD_JOINT_POSITION;
    }
    // << SAFETY

    twai_message_t msg;
    cybergear_driver.set_position_ref(can_id, target_position, msg);
    send_can_packet_task(msg);
    recv_can();
}

void ControlManager::velocity_control(uint8_t can_id, double target_velocity) {
    // SAFETY
    if (target_velocity < MIN_THRESHOLD_JOINT_VELOCITY) {
        target_velocity = MIN_THRESHOLD_JOINT_VELOCITY;
    } else if (target_velocity > MAX_THRESHOLD_JOINT_VELOCITY) {
        target_velocity = MAX_THRESHOLD_JOINT_VELOCITY;
    }

    if (state_.act_joint_position < MIN_THRESHOLD_JOINT_POSITION &&
        target_velocity < 0.0) {
        target_velocity =
            (-1.0) *
            (state_.act_joint_position - MIN_THRESHOLD_JOINT_POSITION) * 20.0;
    } else if (state_.act_joint_position > MAX_THRESHOLD_JOINT_POSITION &&
               target_velocity > 0.0) {
        target_velocity =
            (-1.0) *
            (state_.act_joint_position - MAX_THRESHOLD_JOINT_POSITION) * 20.0;
    }
    // << SAFETY

    twai_message_t msg;
    cybergear_driver.set_spd_ref(can_id, target_velocity, msg);
    send_can_packet_task(msg);
    recv_can();
}

void ControlManager::torque_control(uint8_t can_id, double target_torque) {
    float cmd_value = 0.0;
    // SAFETY
    // >> 位置・速度超過の確認
    bool is_over_position = false;
    bool is_over_velocity = false;

    double gain_pos_p = 10.0;
    double gain_vel_p = 2.0;
    double gain_vel_i = 0.021;

    double err_trq = 0.0;
    double diff_pos = 0.0;

    double v_cmd = 0.0;

    if (state_.act_joint_position < MIN_THRESHOLD_JOINT_POSITION) {
        is_over_position = true;
    } else if (state_.act_joint_position > MAX_THRESHOLD_JOINT_POSITION) {
        is_over_position = true;
    }
    if (state_.act_joint_velocity < MIN_THRESHOLD_JOINT_VELOCITY) {
        is_over_velocity = true;
    } else if (state_.act_joint_velocity > MAX_THRESHOLD_JOINT_VELOCITY) {
        is_over_velocity = true;
    }
    //
    if (is_over_position) {
        if (state_.act_joint_position < MIN_THRESHOLD_JOINT_POSITION) {
            v_cmd = gain_pos_p *
                    (MIN_THRESHOLD_JOINT_POSITION - state_.act_joint_position);
        } else if (state_.act_joint_position > MAX_THRESHOLD_JOINT_POSITION) {
            v_cmd = gain_pos_p *
                    (MAX_THRESHOLD_JOINT_POSITION - state_.act_joint_position);
        }

        if (v_cmd < MIN_THRESHOLD_JOINT_VELOCITY) {
            v_cmd = MIN_THRESHOLD_JOINT_VELOCITY;
        } else if (v_cmd > MAX_THRESHOLD_JOINT_VELOCITY) {
            v_cmd = MAX_THRESHOLD_JOINT_VELOCITY;
        }
        state_.sum_error_vel += v_cmd - state_.act_joint_velocity;

        target_torque = gain_vel_p * (v_cmd - state_.act_joint_velocity) +
                        gain_vel_i * state_.sum_error_vel;
    } else if (is_over_velocity) {
        if (state_.act_joint_velocity < MIN_THRESHOLD_JOINT_VELOCITY) {
            state_.sum_error_vel +=
                MIN_THRESHOLD_JOINT_VELOCITY - state_.act_joint_velocity;
            target_torque = gain_vel_p * (MIN_THRESHOLD_JOINT_VELOCITY -
                                          state_.act_joint_velocity) +
                            gain_vel_i * state_.sum_error_vel;
        } else if (state_.act_joint_velocity > MAX_THRESHOLD_JOINT_VELOCITY) {
            state_.sum_error_vel +=
                MAX_THRESHOLD_JOINT_VELOCITY - state_.act_joint_velocity;
            target_torque = gain_vel_p * (MAX_THRESHOLD_JOINT_VELOCITY -
                                          state_.act_joint_velocity) +
                            gain_vel_i * state_.sum_error_vel;
        }
    } else {
        v_cmd = 0.0;
        state_.sum_error_vel = 0.0;
        state_.sum_error_torque = 0.0;
    }

    if (target_torque < MIN_THRESHOLD_JOINT_TORQUE) {
        target_torque = MIN_THRESHOLD_JOINT_TORQUE;
    } else if (target_torque > MAX_THRESHOLD_JOINT_TORQUE) {
        target_torque = MAX_THRESHOLD_JOINT_TORQUE;
    }

    /*
    if (state_.act_joint_position < MIN_THRESHOLD_JOINT_POSITION &&
        target_torque < 0.0) {
        target_torque =
            (-1.0) *
            (state_.act_joint_position - MIN_THRESHOLD_JOINT_POSITION) * 5.0;
    } else if (state_.act_joint_position > MAX_THRESHOLD_JOINT_POSITION &&
               target_torque > 0.0) {
        target_torque =
            (-1.0) *
            (state_.act_joint_position - MAX_THRESHOLD_JOINT_POSITION) * 5.0;
    }
    */
    // << SAFETY
    err_trq = target_torque - state_.act_joint_torque;
    state_.sum_error_torque += err_trq;

    cmd_value = 2.0 * err_trq + 0.01 * state_.sum_error_torque;

    if (cmd_value < MIN_THRESHOLD_JOINT_CURRENT) {
        cmd_value = MIN_THRESHOLD_JOINT_CURRENT;
    } else if (cmd_value > MAX_THRESHOLD_JOINT_CURRENT) {
        cmd_value = MAX_THRESHOLD_JOINT_CURRENT;
    }

    twai_message_t msg;
    cybergear_driver.set_iq_ref(can_id, cmd_value, msg);
    send_can_packet_task(msg);
    recv_can();
}

void ControlManager::init_servo_dummy() {
    state_.is_init_joint_pos = true;
    state_.act_joint_position = 0.0;
    state_.act_joint_velocity = 0.0;
    state_.act_joint_torque = 0.0;
}

void ControlManager::get_motor_id(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.get_motor_id(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::set_controlled_servo_id(uint8_t servo_id) {
    // send
    get_motor_id(servo_id);
    // recv
    state_.act_can_connection_status = recv_can();
    if (state_.act_can_connection_status) {
        state_.cmd_joint_position = state_.act_joint_position;
        state_.cmd_joint_velocity = 0.0;
        state_.cmd_joint_torque = 0.0;

        state_.act_joint_position_0 = state_.act_joint_position;
        state_.act_joint_velocity_0 = 0.0f;
        state_.act_joint_torque_0 = 0.0f;

        set_servo_ctrl_mode(servo_id, basic_servo_ctrl_cmd_list::STAY);

        // M5_LOGI("ControlManager::set_controlled_servo_id: Connected (%d)",
        //         servo_id);
    } else {
        // M5_LOGI("ControlManager::set_controlled_servo_id: Not Connected
        // (%d)",
        //         servo_id);
    }
    // SET
    state_.servo_id = servo_id;
    // RESET
    // ---
}

// [NOTE]単軸モードで使う想定の関数
void ControlManager::change_controlled_servo_id(uint8_t servo_id) {
    // 前処理
    // >> 現在のサーボに停止処理を送る
    if (state_.act_can_connection_status) {
        stop_motor(state_.servo_id);
        motor_status[state_.servo_id].is_connected = false;
        // M5_LOGI("ControlManager::change_controlled_servo_id: Disconnected
        // (%d)",
        //         state_.servo_id);
    }
    // SET
    state_.waiting_servo_id = servo_id;
    // RESET
    state_.is_init_joint_pos = false;
    state_.act_can_connection_status = false;
    state_.ctrl_mode = basic_servo_ctrl_cmd_list::STAY;
    // 処理
    // >> 新しいサーボに接続処理を送る
    set_controlled_servo_id(servo_id);
}

void ControlManager::set_servo_power(uint8_t servo_id, bool is_power_on) {
    if (is_power_on && state_.is_init_joint_pos) {
        state_.cmd_joint_position = state_.act_joint_position;
        state_.cmd_joint_velocity = 0.0;
        state_.cmd_joint_torque = 0.0;  // TODO: Implement

        state_.act_joint_position_0 = state_.act_joint_position;
        state_.act_joint_velocity_0 = 0.0;
        state_.act_joint_torque_0 = 0.0;

        if (state_.act_can_connection_status) {  // CAN Connected
            twai_message_t msg;

            cybergear_driver.set_position_ref(servo_id,
                                              state_.act_joint_position, msg);
            send_can_packet_task(msg);
            recv_can();

            cybergear_driver.set_spd_ref(servo_id, 0.0, msg);
            send_can_packet_task(msg);
            recv_can();

            cybergear_driver.set_iq_ref(servo_id, 0.0, msg);
            send_can_packet_task(msg);
            recv_can();

            cybergear_driver.enable_motor(servo_id, msg);
            send_can_packet_task(msg);
            recv_can();
            state_.is_power_on = true;
        } else {  // CAN Disconnected
            state_.is_power_on = true;
        }
    } else {
        if (state_.act_can_connection_status) {  // CAN Connected
            twai_message_t msg;
            cybergear_driver.stop_motor(servo_id, msg);
            send_can_packet_task(msg);
            recv_can();
            state_.is_power_on = false;
        } else {  // CAN Disconnected
            state_.is_power_on = false;
        }
    }
}
void ControlManager::set_servo_ctrl_mode(uint8_t servo_id,
                                         basic_servo_ctrl_cmd_list ctrl_mode) {
    twai_message_t msg;
    if (state_.act_can_connection_status &&
        ctrl_mode != state_.ctrl_mode)  // CAN Connected
    {
        switch (ctrl_mode) {
            case basic_servo_ctrl_cmd_list::STAY:
                // 位置指令のリセット
                state_.act_joint_position_0 = state_.act_joint_position;
                state_.cmd_joint_position = state_.act_joint_position;
                position_control(servo_id, state_.act_joint_position);
                // 速度制限の設定
                cybergear_driver.set_limit_spd(servo_id, 10.0, msg);
                send_can_packet_task(msg);
                recv_can();
                // 位置制御モードへ変更
                cybergear_driver.set_position_mode(servo_id, msg);
                motor_status[servo_id].ctrl_mode = MODE_POSITION;
                break;
            case basic_servo_ctrl_cmd_list::POSITION:
                // 位置指令のリセット
                state_.cmd_joint_position = state_.act_joint_position;
                position_control(servo_id, state_.act_joint_position);
                // 速度制限の設定
                cybergear_driver.set_limit_spd(servo_id, 10.0, msg);
                send_can_packet_task(msg);
                recv_can();
                // 位置制御モードへ変更
                cybergear_driver.set_position_mode(servo_id, msg);
                motor_status[servo_id].ctrl_mode = MODE_POSITION;
                break;
            case basic_servo_ctrl_cmd_list::VELOCITY:
                state_.cmd_joint_velocity = 0.0;
                velocity_control(servo_id, 0.0f);
                cybergear_driver.set_velocity_mode(servo_id, msg);
                motor_status[servo_id].ctrl_mode = MODE_SPEED;
                break;
            case basic_servo_ctrl_cmd_list::TORQUE:
                state_.cmd_joint_torque = 0.0;
                torque_control(servo_id, 0.0f);
                cybergear_driver.set_current_mode(servo_id, msg);
                motor_status[servo_id].ctrl_mode = MODE_CURRENT;
                break;
            case basic_servo_ctrl_cmd_list::MOTION:
                cybergear_driver.set_motion_mode(servo_id, msg);
                motor_status[servo_id].ctrl_mode = MODE_MOTION;
                break;
            default:
                break;
        }
        send_can_packet_task(msg);
        recv_can();
        state_.ctrl_mode = ctrl_mode;
    } else {  // CAN Disconnected
        state_.ctrl_mode = ctrl_mode;
    }
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

/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////

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
    auto ret = twai_receive(&packet, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        M5_LOGE(
            "ControlManager::recv_can_packet_task: Failed to receive CAN "
            "packet");
        return false;
    }
    rx_can_cnt++;
    return true;
}
void ControlManager::set_send_packet(const twai_message_t& packet) {
    send_can_packet = packet;
    send_can_packet_buffer.push(packet);
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

    scale.setLEDColor(0x100000);
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
    // M5DEV_LOGI("Scale weight Value: %f", scale.getWeight());
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
// << END WEIGHT SCALE
