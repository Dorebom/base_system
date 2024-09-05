#include "system_manager.hpp"

#include <memory>

#include "logger.hpp"

SystemManager::SystemManager() {
    node_cmd_ = std::make_shared<node_cmd>(MAX_NODE_CMD_STACK_SIZE);

    udp_send_packet_.fixed_state_header_data_size = sizeof(common_state_code);
    udp_send_packet_.fixed_cmd_header_data_size = sizeof(common_cmd_code);

    system_state_data.state_code.node_id = 1;
    control_state_data.state_code.node_id = 2;

    system_state_ = (SystemState*)system_state_data.data;
    control_state_ = (ControlState*)control_state_data.data;
}

void SystemManager::set_udp_send_state(st_node_state state) {
    uint8_t stack_marker_size;
    int stack_data_size =
        state.state_code.data_size + sizeof(common_state_code);

    if (stack_data_size > udp_send_packet_.max_stack_size_at_once) {
        M5_LOGE("Stack Data Size Over");
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
        udp_send_packet_.max_stack_marker_size - recent_stack_marker_size) {
        M5_LOGW("Stack Marker Size Over");
        is_unsent_data = true;
        memcpy(unsent_data, &state, stack_data_size);
        unsent_stack_marker = stack_marker_size;
        return;
    } else {
        memcpy(&udp_send_packet_.data[recent_stack_marker_size *
                                      udp_send_packet_.one_stack_size],
               &state, stack_data_size);
        udp_send_packet_.stack_marker[udp_send_packet_.stack_marker_num] =
            (int8_t)stack_marker_size;
        // stateの場合は、プラスをつける
        recent_stack_marker_size += stack_marker_size;
        udp_send_packet_.stack_marker_num++;
    }
}

void SystemManager::set_udp_send_cmd(st_node_cmd cmd) {
    int stack_marker_size;
    int stack_data_size = cmd.cmd_code.data_size + sizeof(common_cmd_code);

    if (stack_data_size > udp_send_packet_.max_stack_size_at_once) {
        M5_LOGE("Stack Data Size Over");
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
        udp_send_packet_.max_stack_marker_size - recent_stack_marker_size) {
        M5_LOGW("Stack Marker Size Over");
        is_unsent_data = true;
        memcpy(unsent_data, &cmd, stack_data_size);
        unsent_stack_marker = -stack_marker_size;
        return;
    } else {
        udp_send_packet_.stack_marker_num++;
        memcpy(&udp_send_packet_.data[recent_stack_marker_size *
                                      udp_send_packet_.one_stack_size],
               &cmd, stack_data_size);
        udp_send_packet_.stack_marker[udp_send_packet_.stack_marker_num] =
            -stack_marker_size;
        // commandの場合は、マイナスをつける
        recent_stack_marker_size += stack_marker_size;
        udp_send_packet_.stack_marker_num++;
    }
}

void SystemManager::reset_udp_send_packet(bool discard_unsent_data) {
    recent_stack_marker_size = 0;
    //
    udp_send_packet_.stack_marker_num = 0;
    memset(udp_send_packet_.stack_marker, 0, MAX_UDP_STACK_MARKER_NUM);
    // memset(udp_send_packet_.data, 0, MAX_UDP_SEND_STATE_DATA_SIZE);
    if (is_unsent_data && !discard_unsent_data) {
        memcpy(&udp_send_packet_.data[0], unsent_data,
               std::abs(unsent_stack_marker) * udp_send_packet_.one_stack_size);
        udp_send_packet_.stack_marker[0] = unsent_stack_marker;
        recent_stack_marker_size += std::abs(unsent_stack_marker);
        udp_send_packet_.stack_marker_num++;
        is_unsent_data = false;
    }
}

SystemManager::~SystemManager() {
}

void SystemManager::set_initialized_main_task() {
    is_init_main_task = true;
}

void SystemManager::set_initialized_ctrl_task() {
    is_init_ctrl_task = true;
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
    system_state_data.state_code.state_machine =
        node_state_machine::INITIALIZING;
}

void SystemManager::set_state_machine_ready() {
    system_state_data.state_code.state_machine = node_state_machine::READY;
}

void SystemManager::set_state_machine_stable() {
    system_state_data.state_code.state_machine = node_state_machine::STABLE;
}

void SystemManager::set_state_machine_force_stop() {
    system_state_data.state_code.state_machine = node_state_machine::FORCE_STOP;
}

void SystemManager::cmd_executor() {
    st_node_cmd cmd;

    if (node_cmd_->cmd_stack_.size() != 0) {
        // Execute the command
        cmd = node_cmd_->cmd_stack_.pop();

        system_state_->act_cmd_type = cmd.cmd_code.cmd_type;
        M5_LOGI("Cmd Source: %d", cmd.cmd_code.source);
        M5_LOGI("Act Cmd: %d", system_state_->act_cmd_type);

        is_connected_udp = true;

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
            default:
                is_connected_udp = false;
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

void SystemManager::set_canvas(M5Canvas* canvas_) {
    canvas = canvas_;
    is_init_canvas = true;
}

void SystemManager::updateDisplay() {
    // if (!is_init_canvas || !is_init_all) {
    if (!is_init_canvas) {
        return;
    }
    // Reset the canvas
    canvas->clear();
    canvas->setCursor(0, 0);

    if (system_state_data.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        canvas->setPaletteColor(1, RED);
    } else {
        canvas->setPaletteColor(1, GREEN);
    }

    // Rewrite the canvas
    canvas->startWrite(true);
    // 1.1 Heart Beat
    canvas->setTextSize(TITLE_FONT_SIZE);
    if (display_heart_beat) {
        canvas->printf("System Manager ^-^-^- \r\n");
        display_heart_beat = false;
    } else {
        canvas->printf("System Manager ------ \r\n");
        display_heart_beat = true;
    }
    // 1.2 System Status
    canvas->setTextSize(TEXT_FONT_SIZE_SMALL);
    canvas->printf("System Status\r\n");
    switch (system_state_data.state_code.state_machine) {
        case node_state_machine::INITIALIZING:
            canvas->printf("SM: Initializing\r\n");
            break;
        case node_state_machine::READY:
            canvas->printf("SM: Ready\r\n");
            break;
        case node_state_machine::STABLE:
            canvas->printf("SM: Stable\r\n");
            break;
        case node_state_machine::FORCE_STOP:
            canvas->printf("SM: Force Stop\r\n");
            break;
        default:
            break;
    }
    if (system_state_->emergency_stop_switch_for_control_task) {
        canvas->printf("EMS:ON\r\n");
    } else {
        canvas->printf("EMS:OFF\r\n");
    }

    if (system_state_->encoder_button_flag) {
        canvas->printf("Encoder: ON \t Val: %d\r\n",
                       system_state_->encoder_button_value);
    } else {
        canvas->printf("Encoder: OFF \t Val: %d\r\n",
                       system_state_->encoder_button_value);
    }

    canvas->printf("Ave: %d ms\t Max: %d ms\r\n",
                   system_state_->ave_calc_time_of_main_task,
                   system_state_->max_calc_time_of_main_task);

    canvas->printf("Control Status\r\n");
    canvas->printf("Ave: %d ms\t Max: %d ms\r\n",
                   system_state_->ave_calc_time_of_ctrl_task,
                   system_state_->max_calc_time_of_ctrl_task);
    canvas->printf("UDP Send Status\r\n");
    canvas->printf("Ave: %d ms\t Max: %d ms\r\n",
                   system_state_->ave_calc_time_of_udp_send_task,
                   system_state_->max_calc_time_of_udp_send_task);
    canvas->printf("Recent Recv Cmd: %d \r\n", system_state_->act_cmd_type);
    // 1.3 Control Status
    if (is_init_ctrl_task) {
        if (control_state_->is_init_scale) {
            canvas->setTextSize(TEXT_FONT_SIZE);
            canvas->printf("Weight: %f\r\n", control_state_->sensor_weight);
            canvas->printf("Raw ADC: %d\r\n",
                           control_state_->sensor_weight_raw_adc);
        }
    }
    if (is_init_lan) {
        canvas->setTextSize(TEXT_FONT_SIZE_SMALL);
        canvas->printf("LAN Status\r\n");
        canvas->printf("Local IP: %s\r\n", local_ip.toString().c_str());
        canvas->printf("Dst IP: %s\r\n", destination_ip.toString().c_str());
        canvas->printf("Recv Port: %d \t Send Port: %d\r\n", recv_port,
                       send_port);
        canvas->printf("Recv Num: %d \t  Send Num: %d\r\n",
                       system_state_->udp_recv_num,
                       system_state_->udp_send_num);
    } else {
        canvas->setTextSize(TEXT_FONT_SIZE_SMALL);
        canvas->printf("LAN Status\r\n");
        canvas->printf("...Not Exist...\r\n");
    }

    // <<-- END Rewrite the canvas

    // Update the canvas
    canvas->endWrite();
    canvas->pushSprite(0, 0);

    // M5DEV_LOGI("Canvas updated");
}

void SystemManager::set_control_state(ControlState& state) {
    // deep copy
    control_state_->deepcopy(state);
    // Note:
    // ディスプレイ表示やUDP通信への影響は、状態マシンをReady状態にするかどうかで防ぐ。
}

void SystemManager::set_initialized_lan() {
    is_init_lan = true;
}

bool SystemManager::check_init_lan() {
    return is_init_lan;
}

void SystemManager::set_lan_info(IPAddress local_ip_, IPAddress destination_ip_,
                                 uint32_t recv_port_, uint32_t send_port_) {
    local_ip = local_ip_;
    destination_ip = destination_ip_;
    recv_port = recv_port_;
    send_port = send_port_;
}

uint32_t SystemManager::get_udp_recv_packet_size() {
    return sizeof(st_node_cmd);
}

uint32_t SystemManager::get_udp_send_packet_size() {
    return sizeof(udp_frame);
}

bool SystemManager::check_connected_udp() {
    return is_connected_udp;
}

void SystemManager::set_udp_recv_packet(uint8_t* packetBuffer) {
    node_cmd_->cmd_stack_.push(*(st_node_cmd*)packetBuffer);
    system_state_->udp_recv_num++;
}

int SystemManager::get_udp_send_packet(uint8_t* packetBuffer) {
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
        int packet_size = udp_send_packet_.stack_marker_num *
                              udp_send_packet_.one_stack_size +
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
        system_state_data.state_code.state_machine =
            node_state_machine::FORCE_STOP;
    } else if (em_stop == false &&
               prev_emergency_stop_switch_for_control_task == true) {
        system_state_data.state_code.state_machine = node_state_machine::READY;
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
}

bool SystemManager::check_encoder_button_flag() {
    return system_state_->encoder_button_flag;
}
