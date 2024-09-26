#include "system_manager.hpp"

#include <memory>

#include "logger.hpp"

SystemManager::SystemManager() {
    system_state_data = std::make_shared<node_state>();
    control_state_data = std::make_shared<node_state>();
    node_cmd_ = std::make_shared<node_cmd>(MAX_NODE_CMD_STACK_SIZE);

    udp_send_packet_.fixed_state_header_data_size = sizeof(common_state_code);
    udp_send_packet_.fixed_cmd_header_data_size = sizeof(common_cmd_code);

    system_state_data->state_code.node_id = 1;
    control_state_data->state_code.node_id = 2;

    system_state_data->state_code.data_size = sizeof(SystemState);
    control_state_data->state_code.data_size = sizeof(ControlState);

    system_state_ = (SystemState*)system_state_data->data;
    control_state_ = (ControlState*)control_state_data->data;

    system_state_->init();
    control_state_->init();

    // system_cmd_reg_.setup(node_cmd_, system_state_data, control_state_data,
    //                       control_state_);
}

SystemManager::~SystemManager() {
}

void SystemManager::set_udp_send_state(std::shared_ptr<st_node_state> state) {
    uint8_t stack_marker_size;
    int stack_data_size =
        state->state_code.data_size + sizeof(common_state_code);

    if (stack_data_size > udp_send_packet_.max_stack_size_at_once) {
        M5_LOGE("(STATE)Stack Data Size Over %d", stack_data_size);
        return;
    }

    int stack_data_surplus = stack_data_size % udp_send_packet_.one_stack_size;
    if (stack_data_surplus == 0) {
        stack_marker_size = stack_data_size / udp_send_packet_.one_stack_size;
    } else {
        stack_marker_size = (stack_data_size - stack_data_surplus) /
                                udp_send_packet_.one_stack_size +
                            1;
    }

    if (stack_marker_size >
        udp_send_packet_.max_stack_marker_size - udp_send_packet_.stack_num) {
        M5_LOGW("(STATE)Stack Marker Size Over %d", stack_marker_size);
        is_unsent_data = true;
        memcpy(unsent_data, &state, stack_data_size);
        unsent_stack_marker = stack_marker_size;
        return;
    } else {
        memcpy(&udp_send_packet_.data[udp_send_packet_.stack_num *
                                      udp_send_packet_.one_stack_size],
               &state, stack_data_size);
        udp_send_packet_.stack_marker[udp_send_packet_.stack_marker_num] =
            (int8_t)stack_marker_size;
        // stateの場合は、プラスをつける
        udp_send_packet_.stack_num += stack_marker_size;
        udp_send_packet_.stack_marker_num++;
    }
}

void SystemManager::set_udp_send_cmd(st_node_cmd cmd) {
    int stack_marker_size;
    int stack_data_size = cmd.cmd_code.data_size + sizeof(common_cmd_code);

    if (stack_data_size > udp_send_packet_.max_stack_size_at_once) {
        M5_LOGE("(CMD)Stack Data Size Over %d", stack_data_size);
        return;
    }

    int stack_data_surplus = stack_data_size % udp_send_packet_.one_stack_size;
    if (stack_data_surplus == 0) {
        stack_marker_size = stack_data_size / udp_send_packet_.one_stack_size;
    } else {
        stack_marker_size = (stack_data_size - stack_data_surplus) /
                                udp_send_packet_.one_stack_size +
                            1;
    }

    if (stack_marker_size >
        udp_send_packet_.max_stack_marker_size - udp_send_packet_.stack_num) {
        M5_LOGW("(CMD)Stack Marker Size Over (marker size)%d, (recent_sum)%d",
                stack_marker_size, udp_send_packet_.stack_num);
        is_unsent_data = true;
        memcpy(unsent_data, &cmd, stack_data_size);
        unsent_stack_marker = -stack_marker_size;
        return;
    } else {
        memcpy(&udp_send_packet_.data[udp_send_packet_.stack_num *
                                      udp_send_packet_.one_stack_size],
               &cmd, stack_data_size);
        udp_send_packet_.stack_marker[udp_send_packet_.stack_marker_num] =
            -stack_marker_size;
        // commandの場合は、マイナスをつける
        udp_send_packet_.stack_num += stack_marker_size;
        udp_send_packet_.stack_marker_num++;
    }
}

void SystemManager::reset_udp_send_packet(bool discard_unsent_data) {
    udp_send_packet_.stack_num = 0;
    //
    udp_send_packet_.stack_marker_num = 0;
    memset(udp_send_packet_.stack_marker, 0, MAX_UDP_STACK_MARKER_NUM);
    // memset(udp_send_packet_.data, 0, MAX_UDP_SEND_STATE_DATA_SIZE);
    if (is_unsent_data && !discard_unsent_data) {
        memcpy(&udp_send_packet_.data[0], unsent_data,
               std::abs(unsent_stack_marker) * udp_send_packet_.one_stack_size);
        udp_send_packet_.stack_marker[0] = unsent_stack_marker;
        udp_send_packet_.stack_num += std::abs(unsent_stack_marker);
        udp_send_packet_.stack_marker_num++;
        is_unsent_data = false;
    }
}

void SystemManager::set_initialized_main_task() {
    is_init_main_task = true;
}

void SystemManager::set_initialized_ctrl_task() {
    system_state_->is_init_ctrl_task = true;
}

void SystemManager::set_calc_time_of_main_task(uint32_t ave_calc_time,
                                               uint32_t max_calc_time) {
    system_state_->ave_calc_time_of_main_task = ave_calc_time;
    system_state_->max_calc_time_of_main_task = max_calc_time;
}

void SystemManager::set_calc_time_of_ctrl_task(uint32_t ave_calc_time,
                                               uint32_t max_calc_time) {
    system_state_->ave_calc_time_of_ctrl_task = ave_calc_time;
    system_state_->max_calc_time_of_ctrl_task = max_calc_time;
}

void SystemManager::set_calc_time_of_udp_send_task(uint32_t ave_calc_time,
                                                   uint32_t max_calc_time) {
    system_state_->ave_calc_time_of_udp_send_task = ave_calc_time;
    system_state_->max_calc_time_of_udp_send_task = max_calc_time;
}

void SystemManager::set_state_machine_initializing() {
    system_state_data->state_code.state_machine =
        node_state_machine::INITIALIZING;
}

void SystemManager::set_state_machine_ready() {
    system_state_data->state_code.state_machine = node_state_machine::READY;
}

void SystemManager::set_state_machine_stable() {
    system_state_data->state_code.state_machine = node_state_machine::STABLE;
}

void SystemManager::set_state_machine_force_stop() {
    system_state_data->state_code.state_machine =
        node_state_machine::FORCE_STOP;
}

void SystemManager::cmd_executor() {
    st_node_cmd cmd;

    if (node_cmd_->cmd_stack_.size() != 0) {
        // Execute the command
        cmd = node_cmd_->cmd_stack_.pop();

        system_state_->act_cmd_type = cmd.cmd_code.cmd_type;

        switch (cmd.cmd_code.cmd_type) {
            case basic_m5stack_cmd_list::CHANGE_SM_READY:
                if (system_state_->emergency_stop_switch_for_control_task ||
                    system_state_data->state_code.state_machine ==
                        node_state_machine::FORCE_STOP) {
                    break;
                }
                set_state_machine_ready();
                break;
            case basic_m5stack_cmd_list::CHANGE_SM_STABLE:
                if (system_state_->emergency_stop_switch_for_control_task ||
                    system_state_data->state_code.state_machine ==
                        node_state_machine::FORCE_STOP) {
                    break;
                }
                if (system_state_->is_occured_error_ ||
                    system_state_->is_occured_warning_) {
                    break;
                }
                set_state_machine_stable();
                break;
            case basic_m5stack_cmd_list::CHANGE_SM_FORCE_STOP:
                set_state_machine_force_stop();
                break;
            case basic_m5stack_cmd_list::RELEASE_FORCE_STOP:
                if (system_state_->emergency_stop_switch_for_control_task ||
                    system_state_data->state_code.state_machine !=
                        node_state_machine::FORCE_STOP) {
                    break;
                }
                set_state_machine_ready();
                break;
            case basic_m5stack_cmd_list::REQUEST_STATE:
                is_requested_state_at_once = true;
                break;
            case basic_m5stack_cmd_list::START_STREAM_STATE:
                is_streaming_state = true;
                break;
            case basic_m5stack_cmd_list::STOP_STREAM_STATE:
                is_streaming_state = false;
                break;
            case basic_m5stack_cmd_list::SET_NODE_ID:
                // TODO: Implement
                break;
            case basic_m5stack_cmd_list::SET_CONFIG:
                // TODO: Implement
                break;
            case basic_m5stack_cmd_list::RESET_ERROR:
                system_state_->is_occured_error_ = false;
                break;
            case basic_m5stack_cmd_list::RESET_ALERT:
                system_state_->is_occured_warning_ = false;
                break;
            case basic_m5stack_cmd_list::CONNECT_CAN:
                if (system_state_data->state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::DISCONNECT_CAN:
                if (system_state_data->state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID:
                if (system_state_data->state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_POWER:
                if (system_state_data->state_code.state_machine !=
                    node_state_machine::STABLE) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_CTRLMODE:
                if (system_state_data->state_code.state_machine ==
                    node_state_machine::FORCE_STOP) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::SERVO_POSITION_CONTROL:
                if (!control_state_->is_power_on) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::SERVO_VELOCITY_CONTROL:
                if (!control_state_->is_power_on) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::SERVO_TORQUE_CONTROL:
                if (!control_state_->is_power_on) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::START_LOGGING:
                if (system_state_->is_logging ||
                    !system_state_->is_connected_udp) {
                    break;
                }
                set_res_cmd_start_logging();
                system_state_->is_logging = true;
                break;
            case basic_m5stack_cmd_list::STOP_LOGGING:
                if (!system_state_->is_logging ||
                    !system_state_->is_connected_udp) {
                    break;
                }
                set_res_cmd_stop_logging();
                system_state_->is_logging = false;
                break;
            default:
                system_state_->is_connected_udp = false;
                break;
        }
    }
}

bool SystemManager::check_force_stop() {
    if (system_state_data->state_code.state_machine !=
        node_state_machine::FORCE_STOP) {
        return false;
    }
    return true;
}

node_state_machine SystemManager::check_state_machine() {
    return system_state_data->state_code.state_machine;
}

void SystemManager::set_cmd_change_state_machine(
    node_state_machine state_machine) {
    st_node_cmd cmd;
    cmd.default_init();

    // すでに同じ状態になっている場合は、何もしない
    if (state_machine == system_state_data->state_code.state_machine) {
        return;
    }
    //
    switch (state_machine) {
        case node_state_machine::READY:
            cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SM_READY;
            break;
        case node_state_machine::STABLE:
            cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SM_STABLE;
            break;
        case node_state_machine::FORCE_STOP:
            cmd.cmd_code.cmd_type =
                basic_m5stack_cmd_list::CHANGE_SM_FORCE_STOP;
            break;
        default:
            break;
    }

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_change_servo_id(uint8_t servo_id) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_data->state_code.state_machine !=
        node_state_machine::READY) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID;
    cmd_change_controlled_servo_id* cmd_data =
        (cmd_change_controlled_servo_id*)cmd.data;
    cmd_data->servo_id = servo_id;

    cmd.cmd_code.data_size = sizeof(cmd_change_controlled_servo_id);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_change_servo_power(uint8_t servo_id,
                                               bool is_power_on) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_data->state_code.state_machine !=
        node_state_machine::STABLE) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SRV_POWER;
    cmd_change_servo_power* cmd_data = (cmd_change_servo_power*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->is_on = is_power_on;

    cmd.cmd_code.data_size = sizeof(cmd_change_servo_power);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_change_servo_ctrl_mode(
    uint8_t servo_id, basic_servo_ctrl_cmd_list ctrl_mode) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_data->state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SRV_CTRLMODE;
    cmd_change_servo_ctrl_mode* cmd_data =
        (cmd_change_servo_ctrl_mode*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->ctrl_mode = ctrl_mode;

    cmd.cmd_code.data_size = sizeof(cmd_change_servo_ctrl_mode);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_velocity_control(uint8_t servo_id,
                                             double velocity) {
    st_node_cmd cmd;
    cmd.default_init();

    if (!control_state_->is_power_on) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::SERVO_VELOCITY_CONTROL;
    cmd_velocity_control* cmd_data = (cmd_velocity_control*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->velocity = velocity;

    cmd.cmd_code.data_size = sizeof(cmd_velocity_control);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_torque_control(uint8_t servo_id, double torque) {
    st_node_cmd cmd;
    cmd.default_init();

    if (!control_state_->is_power_on) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::SERVO_TORQUE_CONTROL;
    cmd_torque_control* cmd_data = (cmd_torque_control*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->torque = torque;

    cmd.cmd_code.data_size = sizeof(cmd_torque_control);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_position_control(uint8_t servo_id,
                                             double target_position) {
    st_node_cmd cmd;
    cmd.default_init();

    if (!control_state_->is_power_on) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::SERVO_POSITION_CONTROL;
    cmd_position_control* cmd_data = (cmd_position_control*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->target_position = target_position;

    cmd.cmd_code.data_size = sizeof(cmd_position_control);

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_start_logging() {
    st_node_cmd cmd;
    cmd.default_init();

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::START_LOGGING;

    node_cmd_->cmd_stack_.push(cmd);
}

void SystemManager::set_cmd_stop_logging() {
    st_node_cmd cmd;
    cmd.default_init();

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::STOP_LOGGING;

    node_cmd_->cmd_stack_.push(cmd);
}

// RES CMD
void SystemManager::set_res_cmd_start_logging() {
    if (is_streaming_state) {
        is_streaming_state_for_logging = false;
    } else {
        is_streaming_state_for_logging = true;
        is_streaming_state = true;
    }

    st_node_cmd res_cmd;
    res_cmd.default_init();
    res_cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::START_LOGGING;
    set_udp_send_cmd(res_cmd);
}

void SystemManager::set_res_cmd_stop_logging() {
    if (is_streaming_state_for_logging) {
        is_streaming_state_for_logging = false;
        is_streaming_state = false;
    }
    st_node_cmd res_cmd;
    res_cmd.default_init();
    res_cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::STOP_LOGGING;
    set_udp_send_cmd(res_cmd);
}

void SystemManager::update_manual_operating() {
    int diff = 0;
    // >> ダブルクリックされたら、フェーズを変更
    if (manual_operating_state_.act_encoder_button_flag_double_pressed) {
        switch (manual_operating_state_.act_phase) {
            case manual_operating_phase::VALUE_CHANGE:
                manual_operating_state_.act_phase =
                    manual_operating_phase::MODE_CHANGE;
                break;
            case manual_operating_phase::MODE_CHANGE:
                manual_operating_state_.act_phase =
                    manual_operating_phase::VALUE_CHANGE;
                break;
            default:
                break;
        }
        manual_operating_state_.encoder_offest =
            manual_operating_state_.act_encoder_value;
    }

    if (manual_operating_state_.act_phase ==
        manual_operating_phase::MODE_CHANGE) {
        if (manual_operating_state_.act_encoder_value >
            manual_operating_state_.encoder_offest + 1) {
            // command stackに命令を追加
            switch (manual_operating_state_.mode) {
                case manual_operating_mode::NONE:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SM;
                    break;
                // case manual_operating_mode::CONNECT_CAN:
                //     manual_operating_state_.mode =
                //         manual_operating_mode::CHANGE_SM;
                //     break;
                case manual_operating_mode::CHANGE_SM:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_ID;
                    break;
                case manual_operating_mode::CHANGE_SERVO_ID:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_POWER;
                    break;
                case manual_operating_mode::CHANGE_SERVO_POWER:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_CONTROL_MODE;
                    break;
                case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                    manual_operating_state_.mode =
                        manual_operating_mode::CMD_SERVO_CONTROL;
                    break;
                case manual_operating_mode::CMD_SERVO_CONTROL:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_LOGGING_MODE;
                    break;
                // 循環させない
                // case manual_operating_mode::CMD_SERVO_CONTROL:
                //    manual_operating_state_.mode =
                //    manual_operating_mode::NONE; break;
                default:
                    break;
            }
            manual_operating_state_.encoder_offest =
                manual_operating_state_.act_encoder_value;
        } else if (manual_operating_state_.act_encoder_value <
                   manual_operating_state_.encoder_offest - 1) {
            // command stackに命令を追加
            switch (manual_operating_state_.mode) {
                case manual_operating_mode::CHANGE_LOGGING_MODE:
                    manual_operating_state_.mode =
                        manual_operating_mode::CMD_SERVO_CONTROL;
                    break;
                case manual_operating_mode::CMD_SERVO_CONTROL:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_CONTROL_MODE;
                    break;
                case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_POWER;
                    break;
                case manual_operating_mode::CHANGE_SERVO_POWER:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SERVO_ID;
                    break;
                case manual_operating_mode::CHANGE_SERVO_ID:
                    manual_operating_state_.mode =
                        manual_operating_mode::CHANGE_SM;
                    break;
                case manual_operating_mode::CHANGE_SM:
                    manual_operating_state_.mode = manual_operating_mode::NONE;
                    break;
                // case manual_operating_mode::CONNECT_CAN:
                //     manual_operating_state_.mode =
                //     manual_operating_mode::NONE; break;
                default:
                    break;
            }
            manual_operating_state_.encoder_offest =
                manual_operating_state_.act_encoder_value;
        }
    }

    if (manual_operating_state_.act_phase ==
        manual_operating_phase::VALUE_CHANGE) {
        switch (manual_operating_state_.mode) {
            case manual_operating_mode::CHANGE_LOGGING_MODE:
                if (manual_operating_state_.act_encoder_value >
                    manual_operating_state_.encoder_offest + 1) {
                    // command stackに命令を追加
                    if (!system_state_->is_logging) {
                        set_cmd_start_logging();
                    }
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                } else if (manual_operating_state_.act_encoder_value <
                           manual_operating_state_.encoder_offest - 1) {
                    // command stackに命令を追加
                    if (system_state_->is_logging) {
                        set_cmd_stop_logging();
                    }
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SM:
                if (manual_operating_state_.act_encoder_value >
                    manual_operating_state_.encoder_offest + 1) {
                    // command stackに命令を追加
                    set_cmd_change_state_machine(node_state_machine::STABLE);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                } else if (manual_operating_state_.act_encoder_value <
                           manual_operating_state_.encoder_offest - 1) {
                    // command stackに命令を追加
                    set_cmd_change_state_machine(node_state_machine::READY);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_ID:
                if (manual_operating_state_.act_encoder_value >
                    manual_operating_state_.encoder_offest + 1) {
                    if (control_state_->servo_id < 100)
                        set_cmd_change_servo_id(control_state_->servo_id + 1);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                } else if (manual_operating_state_.act_encoder_value <
                           manual_operating_state_.encoder_offest - 1) {
                    if (control_state_->servo_id > 0)
                        set_cmd_change_servo_id(control_state_->servo_id - 1);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_POWER:
                if (manual_operating_state_.act_encoder_value >
                    manual_operating_state_.encoder_offest + 1) {
                    set_cmd_change_servo_power(control_state_->servo_id, true);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                } else if (manual_operating_state_.act_encoder_value <
                           manual_operating_state_.encoder_offest - 1) {
                    set_cmd_change_servo_power(control_state_->servo_id, false);
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                if (manual_operating_state_.act_encoder_value >
                    manual_operating_state_.encoder_offest + 1) {
                    switch (control_state_->ctrl_mode) {
                        case basic_servo_ctrl_cmd_list::TORQUE:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::VELOCITY);
                            break;
                        case basic_servo_ctrl_cmd_list::VELOCITY:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::POSITION);
                            break;
                        case basic_servo_ctrl_cmd_list::POSITION:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::STAY);
                            break;
                        case basic_servo_ctrl_cmd_list::STAY:
                            break;
                        default:
                            break;
                    }
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                } else if (manual_operating_state_.act_encoder_value <
                           manual_operating_state_.encoder_offest - 1) {
                    switch (control_state_->ctrl_mode) {
                        case basic_servo_ctrl_cmd_list::STAY:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::POSITION);
                            break;
                        case basic_servo_ctrl_cmd_list::POSITION:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::VELOCITY);
                            break;
                        case basic_servo_ctrl_cmd_list::VELOCITY:
                            set_cmd_change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::TORQUE);
                            break;
                        case basic_servo_ctrl_cmd_list::TORQUE:
                            break;
                        default:
                            break;
                    }
                    manual_operating_state_.encoder_offest =
                        manual_operating_state_.act_encoder_value;
                }
                break;
            case manual_operating_mode::CMD_SERVO_CONTROL:
                if (manual_operating_state_.act_encoder_button_flag_pressed) {
                    // ctrl_levelを変更
                    if (manual_operating_state_.act_encoder_value >
                        manual_operating_state_.encoder_offest + 1) {
                        if (manual_operating_state_.ctrl_level < 10) {
                            manual_operating_state_.ctrl_level++;
                        }
                        manual_operating_state_.encoder_offest =
                            manual_operating_state_.act_encoder_value;
                    } else if (manual_operating_state_.act_encoder_value <
                               manual_operating_state_.encoder_offest - 1) {
                        if (manual_operating_state_.ctrl_level > 1) {
                            manual_operating_state_.ctrl_level--;
                        }
                        manual_operating_state_.encoder_offest =
                            manual_operating_state_.act_encoder_value;
                    }
                } else {
                    diff = manual_operating_state_.act_encoder_value -
                           manual_operating_state_.encoder_offest;
                    diff *= manual_operating_state_.ctrl_level;

                    switch (control_state_->ctrl_mode) {
                        case basic_servo_ctrl_cmd_list::STAY:
                            break;
                        case basic_servo_ctrl_cmd_list::POSITION:
                            // 偏差
                            if (std::abs(diff) > 1) {
                                if (std::abs(diff) > 50) {
                                    if (diff > 0) {
                                        diff = 50;
                                    } else {
                                        diff = -50;
                                    }
                                }
                                if (control_state_->is_power_on) {
                                    set_cmd_position_control(
                                        control_state_->servo_id,
                                        control_state_->act_joint_position +
                                            0.005 * diff);
                                }
                                manual_operating_state_.encoder_offest =
                                    manual_operating_state_.act_encoder_value;
                            }
                            break;
                        case basic_servo_ctrl_cmd_list::VELOCITY:
                            // 偏差
                            if (std::abs(diff) > 1) {
                                if (std::abs(diff) > 50) {
                                    if (diff > 0) {
                                        diff = 50;
                                    } else {
                                        diff = -50;
                                    }
                                }
                                if (control_state_->is_power_on) {
                                    set_cmd_velocity_control(
                                        control_state_->servo_id,
                                        control_state_->cmd_joint_velocity +
                                            0.02 * diff);
                                }
                                manual_operating_state_.encoder_offest =
                                    manual_operating_state_.act_encoder_value;
                            }
                            break;
                        case basic_servo_ctrl_cmd_list::TORQUE:
                            // 偏差
                            if (std::abs(diff) > 1) {
                                if (std::abs(diff) > 50) {
                                    if (diff > 0) {
                                        diff = 50;
                                    } else {
                                        diff = -50;
                                    }
                                }
                                if (control_state_->is_power_on) {
                                    set_cmd_torque_control(
                                        control_state_->servo_id,
                                        control_state_->cmd_joint_torque +
                                            0.001 * diff);
                                }
                                manual_operating_state_.encoder_offest =
                                    manual_operating_state_.act_encoder_value;
                            }
                            break;
                        default:
                            break;
                    }
                }
            default:
                break;
        }
    }
}

void SystemManager::set_canvas(M5Canvas* canvas_) {
    display_.setup(canvas_);
}

void SystemManager::update_display() {
    display_.update(&manual_operating_state_, control_state_, system_state_,
                    &system_state_data->state_code);
}

void SystemManager::set_control_state(ControlState& state) {
    // deep copy
    control_state_->deepcopy(state);
    // Note:
    // ディスプレイ表示やUDP通信への影響は、状態マシンをReady状態にするかどうかで防ぐ。
}

void SystemManager::set_control_cmd(std::shared_ptr<node_cmd> cmd) {
    ctrl_cmd_ = cmd;
}

void SystemManager::set_initialized_lan() {
    system_state_->is_init_lan = true;
}

bool SystemManager::check_init_lan() {
    return system_state_->is_init_lan;
}

void SystemManager::set_lan_info(IPAddress local_ip_, IPAddress destination_ip_,
                                 uint32_t recv_port_, uint32_t send_port_) {
    display_.set_lan_info(local_ip_.toString(), destination_ip_.toString(),
                          recv_port_, send_port_);
}

uint32_t SystemManager::get_udp_recv_packet_size() {
    return sizeof(st_node_cmd);
}

uint32_t SystemManager::get_udp_send_packet_size() {
    return sizeof(udp_frame);
}

bool SystemManager::check_connected_udp() {
    return system_state_->is_connected_udp;
}

void SystemManager::set_udp_recv_packet(uint8_t* packetBuffer) {
    if (!system_state_->is_connected_udp) {
        system_state_->is_connected_udp = true;
    }
    node_cmd_->cmd_stack_.push(*(st_node_cmd*)packetBuffer);
    system_state_->udp_recv_num++;
}

int SystemManager::get_udp_send_packet(uint8_t* packetBuffer) {
    if (!system_state_->is_connected_udp) {
        return 0;
    }

    // ここで、パケット混載を作る
    if (is_streaming_state) {
        set_udp_send_state(system_state_data);
        set_udp_send_state(control_state_data);
    } else if (is_requested_state_at_once) {
        set_udp_send_state(system_state_data);
        set_udp_send_state(control_state_data);
        is_requested_state_at_once = false;
    }
    if (udp_send_packet_.stack_marker_num > 0) {
        // deep copy
        int packet_size =
            udp_send_packet_.stack_num * udp_send_packet_.one_stack_size +
            udp_send_packet_.udp_frame_header_size;
        memcpy(packetBuffer, &udp_send_packet_, packet_size);
        reset_udp_send_packet(false);
        system_state_->udp_send_num++;
        return packet_size;
    }
    return 0;
}

void SystemManager::set_emergency_stop_for_control_task(bool em_stop) {
    system_state_->emergency_stop_switch_for_control_task = em_stop;
    if (em_stop == true) {
        system_state_data->state_code.state_machine =
            node_state_machine::FORCE_STOP;
    } else if (em_stop == false &&
               prev_emergency_stop_switch_for_control_task == true) {
        system_state_data->state_code.state_machine = node_state_machine::READY;
    }
    prev_emergency_stop_switch_for_control_task = em_stop;
}

void SystemManager::begin_encoder_button(TwoWire* wire, uint8_t addr) {
    encoder_button.begin(wire, addr);
    encoder_button.setLEDColor(1, 0x000000);
    encoder_button.setLEDColor(2, 0x000000);
}

void SystemManager::update_encoder_button() {
    system_state_->encoder_button_value = encoder_button.getEncoderValue();
    // 押してない状態がtrueのため、flagを反転して取得
    system_state_->encoder_button_flag = !encoder_button.getButtonStatus();
    if (system_state_->encoder_button_flag) {
        encoder_button.setLEDColor(1, 0x000010);
    } else {
        encoder_button.setLEDColor(1, 0x000000);
    }
}

void SystemManager::update_encoder_button(bool force_stop_status) {
    if (manual_operating_state_.act_encoder_button_flag_double_pressed) {
        manual_operating_state_.act_encoder_button_flag_double_pressed = false;
        manual_operating_state_.encoder_button_flag_pressed_just_before = false;
        manual_operating_state_.first_pressed_time = 0;
    }

    system_state_->encoder_button_value = encoder_button.getEncoderValue();
    // 押してない状態がtrueのため、flagを反転して取得
    system_state_->encoder_button_flag = !encoder_button.getButtonStatus();
    if (system_state_->encoder_button_flag) {
        encoder_button.setLEDColor(1, 0x000010);
    } else {
        encoder_button.setLEDColor(1, 0x000000);
    }
    if (force_stop_status) {
        encoder_button.setLEDColor(2, 0x100000);
    } else {
        encoder_button.setLEDColor(2, 0x000000);
    }

    manual_operating_state_.act_encoder_button_flag_pressed =
        system_state_->encoder_button_flag;
    manual_operating_state_.act_encoder_value =
        system_state_->encoder_button_value;

    // check double click of encoder button
    // >> ボタンが押された立ち上がりを検知
    if (manual_operating_state_.act_encoder_button_flag_pressed &&
        !manual_operating_state_.prev_encoder_button_flag_pressed) {
        // 少し前に押されたか確認し、押されてなかったら、初回押下時間を記録
        if (!manual_operating_state_.encoder_button_flag_pressed_just_before) {
            manual_operating_state_.first_pressed_time = millis();
            manual_operating_state_.encoder_button_flag_pressed_just_before =
                true;
        } else {
            // 一定時間経過したら、初回押下時間をリセット
            if (millis() - manual_operating_state_.first_pressed_time > 1000) {
                manual_operating_state_
                    .encoder_button_flag_pressed_just_before = false;
                manual_operating_state_.first_pressed_time = 0;
            } else {
                manual_operating_state_.act_encoder_button_flag_double_pressed =
                    true;
            }
        }
    }

    // 終了処理
    manual_operating_state_.prev_encoder_button_flag_pressed =
        manual_operating_state_.act_encoder_button_flag_pressed;
}

bool SystemManager::check_encoder_button_flag() {
    return system_state_->encoder_button_flag;
}
