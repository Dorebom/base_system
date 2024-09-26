#include "module_manual_operation.hpp"

void ModuleManualOperation::setup(
    std::shared_ptr<node_cmd> cmd, SystemState* system_state,
    ControlState* control_state, common_state_code* system_state_code,
    common_state_code* control_state_code,
    manual_operating_state* manual_operating_state,
    ModuleSystemCmdRegister* syscmd_reg) {
    system_state_ = system_state;
    control_state_ = control_state;
    system_state_code_ = system_state_code;
    control_state_code_ = control_state_code;

    manual_operating_state_ = manual_operating_state;

    syscmd_reg_ = syscmd_reg;
}

void ModuleManualOperation::begin_encoder_button(TwoWire* wire, uint8_t addr) {
    encoder_button.begin(wire, addr);
    encoder_button.setLEDColor(1, 0x000000);
    encoder_button.setLEDColor(2, 0x000000);
}

void ModuleManualOperation::update_encoder_button(bool force_stop_status) {
    if (manual_operating_state_->act_encoder_button_flag_double_pressed) {
        manual_operating_state_->act_encoder_button_flag_double_pressed = false;
        manual_operating_state_->encoder_button_flag_pressed_just_before =
            false;
        manual_operating_state_->first_pressed_time = 0;
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

    manual_operating_state_->act_encoder_button_flag_pressed =
        system_state_->encoder_button_flag;
    manual_operating_state_->act_encoder_value =
        system_state_->encoder_button_value;

    // check double click of encoder button
    // >> ボタンが押された立ち上がりを検知
    if (manual_operating_state_->act_encoder_button_flag_pressed &&
        !manual_operating_state_->prev_encoder_button_flag_pressed) {
        // 少し前に押されたか確認し、押されてなかったら、初回押下時間を記録
        if (!manual_operating_state_->encoder_button_flag_pressed_just_before) {
            manual_operating_state_->first_pressed_time = millis();
            manual_operating_state_->encoder_button_flag_pressed_just_before =
                true;
        } else {
            // 一定時間経過したら、初回押下時間をリセット
            if (millis() - manual_operating_state_->first_pressed_time > 1000) {
                manual_operating_state_
                    ->encoder_button_flag_pressed_just_before = false;
                manual_operating_state_->first_pressed_time = 0;
            } else {
                manual_operating_state_
                    ->act_encoder_button_flag_double_pressed = true;
            }
        }
    }

    // 終了処理
    manual_operating_state_->prev_encoder_button_flag_pressed =
        manual_operating_state_->act_encoder_button_flag_pressed;
}

void ModuleManualOperation::update() {
    int diff = 0;
    // >> ダブルクリックされたら、フェーズを変更
    if (manual_operating_state_->act_encoder_button_flag_double_pressed) {
        switch (manual_operating_state_->act_phase) {
            case manual_operating_phase::VALUE_CHANGE:
                manual_operating_state_->act_phase =
                    manual_operating_phase::MODE_CHANGE;
                break;
            case manual_operating_phase::MODE_CHANGE:
                manual_operating_state_->act_phase =
                    manual_operating_phase::VALUE_CHANGE;
                break;
            default:
                break;
        }
        manual_operating_state_->encoder_offest =
            manual_operating_state_->act_encoder_value;
    }

    if (manual_operating_state_->act_phase ==
        manual_operating_phase::MODE_CHANGE) {
        if (manual_operating_state_->act_encoder_value >
            manual_operating_state_->encoder_offest + 1) {
            // command stackに命令を追加
            switch (manual_operating_state_->mode) {
                case manual_operating_mode::NONE:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SM;
                    break;
                // case manual_operating_mode::CONNECT_CAN:
                //     manual_operating_state_->mode =
                //         manual_operating_mode::CHANGE_SM;
                //     break;
                case manual_operating_mode::CHANGE_SM:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_ID;
                    break;
                case manual_operating_mode::CHANGE_SERVO_ID:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_POWER;
                    break;
                case manual_operating_mode::CHANGE_SERVO_POWER:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_CONTROL_MODE;
                    break;
                case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                    manual_operating_state_->mode =
                        manual_operating_mode::CMD_SERVO_CONTROL;
                    break;
                case manual_operating_mode::CMD_SERVO_CONTROL:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_LOGGING_MODE;
                    break;
                // 循環させない
                // case manual_operating_mode::CMD_SERVO_CONTROL:
                //    manual_operating_state_->mode =
                //    manual_operating_mode::NONE; break;
                default:
                    break;
            }
            manual_operating_state_->encoder_offest =
                manual_operating_state_->act_encoder_value;
        } else if (manual_operating_state_->act_encoder_value <
                   manual_operating_state_->encoder_offest - 1) {
            // command stackに命令を追加
            switch (manual_operating_state_->mode) {
                case manual_operating_mode::CHANGE_LOGGING_MODE:
                    manual_operating_state_->mode =
                        manual_operating_mode::CMD_SERVO_CONTROL;
                    break;
                case manual_operating_mode::CMD_SERVO_CONTROL:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_CONTROL_MODE;
                    break;
                case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_POWER;
                    break;
                case manual_operating_mode::CHANGE_SERVO_POWER:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SERVO_ID;
                    break;
                case manual_operating_mode::CHANGE_SERVO_ID:
                    manual_operating_state_->mode =
                        manual_operating_mode::CHANGE_SM;
                    break;
                case manual_operating_mode::CHANGE_SM:
                    manual_operating_state_->mode = manual_operating_mode::NONE;
                    break;
                // case manual_operating_mode::CONNECT_CAN:
                //     manual_operating_state_->mode =
                //     manual_operating_mode::NONE; break;
                default:
                    break;
            }
            manual_operating_state_->encoder_offest =
                manual_operating_state_->act_encoder_value;
        }
    }

    if (manual_operating_state_->act_phase ==
        manual_operating_phase::VALUE_CHANGE) {
        switch (manual_operating_state_->mode) {
            case manual_operating_mode::CHANGE_LOGGING_MODE:
                if (manual_operating_state_->act_encoder_value >
                    manual_operating_state_->encoder_offest + 1) {
                    // command stackに命令を追加
                    if (!system_state_->is_logging) {
                        syscmd_reg_->start_logging();
                    }
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                } else if (manual_operating_state_->act_encoder_value <
                           manual_operating_state_->encoder_offest - 1) {
                    // command stackに命令を追加
                    if (system_state_->is_logging) {
                        syscmd_reg_->stop_logging();
                    }
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SM:
                if (manual_operating_state_->act_encoder_value >
                    manual_operating_state_->encoder_offest + 1) {
                    // command stackに命令を追加
                    syscmd_reg_->change_state_machine(
                        node_state_machine::STABLE);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                } else if (manual_operating_state_->act_encoder_value <
                           manual_operating_state_->encoder_offest - 1) {
                    // command stackに命令を追加
                    syscmd_reg_->change_state_machine(
                        node_state_machine::READY);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_ID:
                if (manual_operating_state_->act_encoder_value >
                    manual_operating_state_->encoder_offest + 1) {
                    if (control_state_->servo_id < 100)
                        syscmd_reg_->change_servo_id(control_state_->servo_id +
                                                     1);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                } else if (manual_operating_state_->act_encoder_value <
                           manual_operating_state_->encoder_offest - 1) {
                    if (control_state_->servo_id > 0)
                        syscmd_reg_->change_servo_id(control_state_->servo_id -
                                                     1);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_POWER:
                if (manual_operating_state_->act_encoder_value >
                    manual_operating_state_->encoder_offest + 1) {
                    syscmd_reg_->change_servo_power(control_state_->servo_id,
                                                    true);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                } else if (manual_operating_state_->act_encoder_value <
                           manual_operating_state_->encoder_offest - 1) {
                    syscmd_reg_->change_servo_power(control_state_->servo_id,
                                                    false);
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                }
                break;
            case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                if (manual_operating_state_->act_encoder_value >
                    manual_operating_state_->encoder_offest + 1) {
                    switch (control_state_->ctrl_mode) {
                        case basic_servo_ctrl_cmd_list::TORQUE:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::VELOCITY);
                            break;
                        case basic_servo_ctrl_cmd_list::VELOCITY:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::POSITION);
                            break;
                        case basic_servo_ctrl_cmd_list::POSITION:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::STAY);
                            break;
                        case basic_servo_ctrl_cmd_list::STAY:
                            break;
                        default:
                            break;
                    }
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                } else if (manual_operating_state_->act_encoder_value <
                           manual_operating_state_->encoder_offest - 1) {
                    switch (control_state_->ctrl_mode) {
                        case basic_servo_ctrl_cmd_list::STAY:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::POSITION);
                            break;
                        case basic_servo_ctrl_cmd_list::POSITION:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::VELOCITY);
                            break;
                        case basic_servo_ctrl_cmd_list::VELOCITY:
                            syscmd_reg_->change_servo_ctrl_mode(
                                control_state_->servo_id,
                                basic_servo_ctrl_cmd_list::TORQUE);
                            break;
                        case basic_servo_ctrl_cmd_list::TORQUE:
                            break;
                        default:
                            break;
                    }
                    manual_operating_state_->encoder_offest =
                        manual_operating_state_->act_encoder_value;
                }
                break;
            case manual_operating_mode::CMD_SERVO_CONTROL:
                if (manual_operating_state_->act_encoder_button_flag_pressed) {
                    // ctrl_levelを変更
                    if (manual_operating_state_->act_encoder_value >
                        manual_operating_state_->encoder_offest + 1) {
                        if (manual_operating_state_->ctrl_level < 10) {
                            manual_operating_state_->ctrl_level++;
                        }
                        manual_operating_state_->encoder_offest =
                            manual_operating_state_->act_encoder_value;
                    } else if (manual_operating_state_->act_encoder_value <
                               manual_operating_state_->encoder_offest - 1) {
                        if (manual_operating_state_->ctrl_level > 1) {
                            manual_operating_state_->ctrl_level--;
                        }
                        manual_operating_state_->encoder_offest =
                            manual_operating_state_->act_encoder_value;
                    }
                } else {
                    diff = manual_operating_state_->act_encoder_value -
                           manual_operating_state_->encoder_offest;
                    diff *= manual_operating_state_->ctrl_level;

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
                                    syscmd_reg_->position_control(
                                        control_state_->servo_id,
                                        control_state_->act_joint_position +
                                            0.005 * diff);
                                }
                                manual_operating_state_->encoder_offest =
                                    manual_operating_state_->act_encoder_value;
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
                                    syscmd_reg_->velocity_control(
                                        control_state_->servo_id,
                                        control_state_->cmd_joint_velocity +
                                            0.02 * diff);
                                }
                                manual_operating_state_->encoder_offest =
                                    manual_operating_state_->act_encoder_value;
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
                                    syscmd_reg_->torque_control(
                                        control_state_->servo_id,
                                        control_state_->cmd_joint_torque +
                                            0.001 * diff);
                                }
                                manual_operating_state_->encoder_offest =
                                    manual_operating_state_->act_encoder_value;
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
