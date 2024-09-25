#include "module_manual_operation.hpp"

#include "DataStruct/st_manual_operating.hpp"

void ModuleManualOperation::setup() {
}

void ModuleManualOperation::update(SystemState* system_state_) {
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
                /*
                case manual_operating_mode::CONNECT_CAN:
                    if (manual_operating_state_.act_encoder_value >
                        manual_operating_state_.encoder_offest + 1) {
                        // command stackに命令を追加
                        set_cmd_connect_can(connected_can_switch::CONNECT);
                        manual_operating_state_.encoder_offest =
                            manual_operating_state_.act_encoder_value;
                    } else if (manual_operating_state_.act_encoder_value <
                               manual_operating_state_.encoder_offest - 1) {
                        // command stackに命令を追加
                        set_cmd_connect_can(connected_can_switch::DISCONNECT);
                        manual_operating_state_.encoder_offest =
                            manual_operating_state_.act_encoder_value;
                    }
                    break;
                */
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
