#include "system_manager.hpp"

#include <memory>

#include "logger.hpp"

SystemManager::SystemManager() {
    node_cmd_ = std::make_shared<node_cmd>(MAX_NODE_CMD_STACK_SIZE);

    udp_send_packet_.fixed_state_header_data_size = sizeof(common_state_code);
    udp_send_packet_.fixed_cmd_header_data_size = sizeof(common_cmd_code);

    system_state_data.state_code.node_id = 1;
    control_state_data.state_code.node_id = 2;
    //
    control_state_udp_data.state_code.node_id = 3;

    system_state_data.state_code.transit_destination_node_state_machine =
        (node_state_machine)0x2525;
    control_state_data.state_code.transit_destination_node_state_machine =
        (node_state_machine)0x3636;
    control_state_udp_data.state_code.transit_destination_node_state_machine =
        (node_state_machine)0x1414;

    system_state_data.state_code.data_size = sizeof(SystemState);
    control_state_data.state_code.data_size = sizeof(ControlState);
    //
    control_state_udp_data.state_code.data_size = sizeof(ControlStateUdp);

    system_state_ = (SystemState*)system_state_data.data;
    control_state_ = (ControlState*)control_state_data.data;
    //
    control_state_udp_ = (ControlStateUdp*)control_state_udp_data.data;

    system_state_->init();
    control_state_->init();
    //
    control_state_udp_->init();

    syscmd_reg_.setup(node_cmd_, system_state_, control_state_,
                      &system_state_data.state_code,
                      &control_state_data.state_code);

    manual_operation_.setup(
        node_cmd_, system_state_, control_state_, &system_state_data.state_code,
        &control_state_data.state_code, &manual_operating_state_, &syscmd_reg_);
}

SystemManager::~SystemManager() {
}

void SystemManager::set_udp_send_state(st_node_state* state) {
    // std::lock_guard<std::mutex> lock(mtx_comm_udp_);

    int stack_marker_size;
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
        memcpy(unsent_data, state, stack_data_size);
        unsent_stack_marker = stack_marker_size;
        return;
    } else {
        memcpy(&udp_send_packet_.data[udp_send_packet_.stack_num *
                                      udp_send_packet_.one_stack_size],
               state, stack_data_size);
        udp_send_packet_.stack_marker[udp_send_packet_.stack_marker_num] =
            stack_marker_size;
        // stateの場合は、プラスをつける
        udp_send_packet_.stack_num += stack_marker_size;
        udp_send_packet_.stack_marker_num++;
    }
}

void SystemManager::set_udp_send_cmd(st_node_cmd cmd) {
    // std::lock_guard<std::mutex> lock(mtx_comm_udp_);

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

void SystemManager::cmd_executor() {
    st_node_cmd cmd;

    if (node_cmd_->cmd_stack_.size() != 0) {
        // Execute the command
        cmd = node_cmd_->cmd_stack_.pop();

        system_state_->act_cmd_type = cmd.cmd_code.cmd_type;

        switch (cmd.cmd_code.cmd_type) {
            case basic_m5stack_cmd_list::CHANGE_SM_READY:
                if (system_state_->emergency_stop_switch_for_control_task ||
                    system_state_data.state_code.state_machine ==
                        node_state_machine::FORCE_STOP) {
                    break;
                }
                set_state_machine_ready();
                break;
            case basic_m5stack_cmd_list::CHANGE_SM_STABLE:
                if (system_state_->emergency_stop_switch_for_control_task ||
                    system_state_data.state_code.state_machine ==
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
                    system_state_data.state_code.state_machine !=
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
                if (system_state_data.state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::DISCONNECT_CAN:
                if (system_state_data.state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID:
                if (system_state_data.state_code.state_machine !=
                    node_state_machine::READY) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_POWER:
                if (system_state_data.state_code.state_machine !=
                    node_state_machine::STABLE) {
                    break;
                }
                ctrl_cmd_->cmd_stack_.push(cmd);
                break;
            case basic_m5stack_cmd_list::CHANGE_SRV_CTRLMODE:
                if (system_state_data.state_code.state_machine ==
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
    if (system_state_data.state_code.state_machine !=
        node_state_machine::FORCE_STOP) {
        return false;
    }
    return true;
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
    manual_operation_.update();
}

void SystemManager::set_control_state(ControlState& state) {
    // deep copy
    control_state_->deepcopy(state);
    // Note:
    // ディスプレイ表示やUDP通信への影響は、状態マシンをReady状態にするかどうかで防ぐ。
    if (is_streaming_state_for_logging) {
        control_state_udp_->compressed_copy(state);
        set_udp_send_state(&control_state_udp_data);
    }
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
        // set_udp_send_state(&system_state_data);
        if (!is_streaming_state_for_logging) {
            // ロギング時は圧縮データを送信するため、送信しない
            set_udp_send_state(&control_state_data);
        }
    } else if (is_requested_state_at_once) {
        // set_udp_send_state(&system_state_data);
        set_udp_send_state(&control_state_data);
        is_requested_state_at_once = false;
    }
    if (udp_send_packet_.stack_marker_num > 0) {
        // deep copy
        int packet_size =
            udp_send_packet_.stack_num * udp_send_packet_.one_stack_size +
            udp_send_packet_.udp_frame_header_size;
        // mutex
        // std::lock_guard<std::mutex> lock(mtx_comm_udp_);
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
        system_state_data.state_code.state_machine =
            node_state_machine::FORCE_STOP;
    } else if (em_stop == false &&
               prev_emergency_stop_switch_for_control_task == true) {
        system_state_data.state_code.state_machine = node_state_machine::READY;
    }
    prev_emergency_stop_switch_for_control_task = em_stop;
}

/*
 * WEIGHT SCALE
 */
bool SystemManager::begin_scale(TwoWire& wire, uint8_t addr) {
    uint8_t watchdog = 0;
    while (!scale.begin(&wire, addr)) {
        M5DEV_LOGE("Scale not found");
        system_state_->is_init_scale = false;
        delay(200);
        watchdog++;
        if (watchdog > 5) {
            M5DEV_LOGE("Scale not found. Break");
            return false;
        }
    }
    M5DEV_LOGI("Scale found");
    system_state_->is_init_scale = true;

    scale.setLEDColor(0x100000);
    // scale.setLPFilter(50);
    // scale.setAvgFilter(10);
    scale.setEmaFilter(50);
    scale.setGapValue(200.0f);
    scale.setOffset();
    // scale.setGapValue(0.0f);
    return true;
}
void SystemManager::reset_scale() {
    scale.setOffset();
    // scale.setGapValue(0.0f);
}
float SystemManager::get_weight() {
    if (!system_state_->is_init_scale) {
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
    system_state_->sensor_weight = weight;
    return weight;
}
int32_t SystemManager::get_weightRawADC() {
    if (!system_state_->is_init_scale) {
        return -1.0;
    }
    int32_t raw_adc = scale.getRawADC();
    system_state_->sensor_weight_raw_adc = raw_adc;
    return raw_adc;
}
// << END WEIGHT SCALE
