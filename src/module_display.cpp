#include "module_display.hpp"

void ModuleDisplay::setup(M5Canvas* canvas) {
    // Serial.println("module_display::setup");
    canvas_ = canvas;
    is_init_canvas = true;
}

void ModuleDisplay::set_lan_info(String ip_address_local, String ip_address_dst,
                                 uint32_t recv_port, uint32_t send_port) {
    ip_local_ = ip_address_local;
    ip_destination_ = ip_address_dst;
    recv_port_ = recv_port;
    send_port_ = send_port;
}

void ModuleDisplay::update(manual_operating_state* manop_state_,
                           ControlState* ctrl_state_,
                           SystemState* system_state_,
                           common_state_code* system_state_code) {
    // Serial.println("module_display::update");

    if (prev_is_logging == true && system_state_->is_logging == false) {
        // Reset the canvas
        canvas_->clear();
        canvas_->setCursor(0, 0);
        canvas_->pushSprite(0, 0);

        canvas_->deleteSprite();
        canvas_->createSprite(320, 240);
        canvas_->setPaletteColor(1, GREEN);
        canvas_->setTextScroll(false);
        canvas_->setTextSize(2);
    } else if (prev_is_logging == false && system_state_->is_logging == true) {
        canvas_->clear();
        canvas_->setCursor(0, 0);
        canvas_->pushSprite(0, 0);

        canvas_->deleteSprite();
        canvas_->createSprite(240, 20);
        canvas_->setPaletteColor(1, GREEN);
        canvas_->setTextScroll(false);
        canvas_->setTextSize(2);
    }

    if (!system_state_->is_logging) {
        normal_display(manop_state_, ctrl_state_, system_state_,
                       system_state_code);
    } else {
        small_display(manop_state_, ctrl_state_, system_state_,
                      system_state_code);
    }

    prev_is_logging = system_state_->is_logging;
}

void ModuleDisplay::normal_display(manual_operating_state* manop_state_,
                                   ControlState* ctrl_state_,
                                   SystemState* system_state_,
                                   common_state_code* system_state_code) {
    // if (!is_init_canvas || !is_init_all) {
    if (!is_init_canvas) {
        return;
    }
    // Reset the canvas
    canvas_->clear();
    canvas_->setCursor(0, 0);

    switch (system_state_code->state_machine) {
        case node_state_machine::FORCE_STOP:
            canvas_->setPaletteColor(1, RED);
            break;
        case node_state_machine::READY:
            canvas_->setPaletteColor(1, WHITE);
            break;
        case node_state_machine::STABLE:
            canvas_->setPaletteColor(1, GREEN);
            break;
        default:
            canvas_->setPaletteColor(1, WHITE);
            break;
    }

    // Rewrite the canvas
    canvas_->startWrite(true);
    // 1.1 Heart Beat
    canvas_->setTextSize(TITLE_FONT_SIZE);
    if (display_heart_beat) {
        canvas_->printf("System Manager ^-^-^- \r\n");
        display_heart_beat = false;
    } else {
        canvas_->printf("System Manager ------ \r\n");
        display_heart_beat = true;
    }
    // 1.2 System Status
    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    if (manop_state_->mode == manual_operating_mode::CHANGE_SM &&
        manop_state_->act_phase == manual_operating_phase::VALUE_CHANGE &&
        display_blink_cnt % 5 == 0) {
        canvas_->printf(">> System Status \t SM: \r\n");
    } else {
        switch (system_state_code->state_machine) {
            case node_state_machine::INITIALIZING:
                canvas_->printf(">> System Status \t SM: Initializing\r\n");
                break;
            case node_state_machine::READY:
                canvas_->printf(">> System Status \t SM: Ready\r\n");
                break;
            case node_state_machine::STABLE:
                canvas_->printf(">> System Status \t SM: Stable\r\n");
                break;
            case node_state_machine::FORCE_STOP:
                if (display_heart_beat) {
                    canvas_->printf(">> System Status \t SM: Force Stop\r\n");
                } else {
                    canvas_->printf(">> System Status \t SM: \r\n");
                }
                break;
            default:
                break;
        }
    }
    /*
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
    */

    canvas_->printf("Ave: %d ms\t Max: %d ms\r\n",
                    system_state_->ave_calc_time_of_main_task,
                    system_state_->max_calc_time_of_main_task);

    // Manual Operating
    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    if (manop_state_->act_phase == manual_operating_phase::MODE_CHANGE &&
        display_blink_cnt % 5 == 0) {
        canvas_->printf("Manual OpMode: \r\n");
    } else {
        switch (manop_state_->mode) {
            case manual_operating_mode::NONE:
                canvas_->printf("Manual OpMode: NONE\r\n");
                break;
            // case manual_operating_mode::CONNECT_CAN:
            //     canvas->printf("Manual OpMode: CONNECT_CAN\r\n");
            //     break;
            case manual_operating_mode::CHANGE_SM:
                canvas_->printf("Manual OpMode: CHANGE SM\r\n");
                break;
            case manual_operating_mode::CHANGE_SERVO_ID:
                canvas_->printf("Manual OpMode: CHANGE SRV ID\r\n");
                break;
            case manual_operating_mode::CHANGE_SERVO_POWER:
                canvas_->printf("Manual OpMode: CHANGE SRV POWER\r\n");
                break;
            case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
                canvas_->printf("Manual OpMode: CHANGE SRV CTRLMODE\r\n");
                break;
            case manual_operating_mode::CMD_SERVO_CONTROL:
                canvas_->printf("Manual OpMode: CMD SRV CONTROL\r\n");
                break;
            case manual_operating_mode::CHANGE_LOGGING_MODE:
                canvas_->printf("Manual OpMode: CHANGE LOGGING MODE\r\n");
                break;
            default:
                break;
        }
    }
    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    if (manop_state_->mode == manual_operating_mode::CHANGE_LOGGING_MODE &&
        manop_state_->act_phase == manual_operating_phase::VALUE_CHANGE &&
        display_blink_cnt % 5 == 0) {
        canvas_->printf("Log: \r\n");
    } else {
        if (system_state_->is_connected_udp) {
            if (system_state_->is_logging) {
                canvas_->printf("Log: ON\r\n");
            } else {
                canvas_->printf("Log: OFF\r\n");
            }
        } else {
            canvas_->printf("Log: OFF \t (UDP is not connected !)\r\n");
        }
    }

    canvas_->print("Ctrl Level: ");
    for (int i = 0; i < manop_state_->ctrl_level; i++) {
        canvas_->print("##");
    }
    canvas_->print("\r\n");
    // << END Manual Operating
    // canvas->setTextSize(TEXT_FONT_SIZE_SMALL);
    // canvas->printf("Recent Recv Cmd: %d \r\n", system_state_->act_cmd_type);

    // 1.3 Control Status
    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    canvas_->printf(">> Control Status\r\n");
    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    canvas_->printf("Ave: %d us\t Max: %d us\r\n",
                    system_state_->ave_calc_time_of_ctrl_task,
                    system_state_->max_calc_time_of_ctrl_task);

    if (system_state_->is_init_ctrl_task) {
        if (ctrl_state_->is_init_scale) {
            canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
            canvas_->printf("Weight: %f, \t Raw ADC: %d\r\n",
                            ctrl_state_->sensor_weight,
                            ctrl_state_->sensor_weight_raw_adc);
        } else {
            // canvas->setTextSize(TEXT_FONT_SIZE_SMALL);
            // canvas->printf("Weight: Not Init, \t Raw ADC: Not Init\r\n");
            canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
            canvas_->printf("Weight: %f, \t Raw ADC: %d\r\n",
                            ctrl_state_->sensor_weight,
                            ctrl_state_->sensor_weight_raw_adc);
        }

        canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
        /*
        if (manop_state_->mode ==
                manual_operating_mode::CHANGE_SERVO_ID &&
            manop_state_->act_phase ==
                manual_operating_phase::VALUE_CHANGE &&
            display_blink_cnt % 5 == 0) {
            canvas->printf("CAN: \r\n");
        } else {
            if (control_state_->act_can_connection_status) {
                canvas->printf("CAN: Connected\r\n");
            } else {
                canvas->printf("CAN: Not connected\r\n");
            }
        }
        */
        if (manop_state_->mode == manual_operating_mode::CHANGE_SERVO_ID &&
            manop_state_->act_phase == manual_operating_phase::VALUE_CHANGE &&
            display_blink_cnt % 5 == 0) {
            canvas_->printf("SRV ID: \r\n");
        } else {
            canvas_->printf("SRV ID: %d", ctrl_state_->servo_id);
            if (ctrl_state_->act_can_connection_status) {
                canvas_->printf(" (CAN)");
            } else {
                canvas_->printf(" (DUMMY)");
            }
            if (system_state_code->state_machine != node_state_machine::READY &&
                manop_state_->mode == manual_operating_mode::CHANGE_SERVO_ID) {
                canvas_->printf("\t (SM is not Ready !)\r\n");
            } else {
                canvas_->printf("\r\n");
            }
        }

        if ((manop_state_->mode == manual_operating_mode::CHANGE_SERVO_POWER ||
             (manop_state_->mode == manual_operating_mode::CMD_SERVO_CONTROL &&
              !ctrl_state_->is_power_on)) &&
            manop_state_->act_phase == manual_operating_phase::VALUE_CHANGE &&
            display_blink_cnt % 5 == 0) {
            canvas_->printf("SRV Power: \r\n");
        } else {
            if (system_state_code->state_machine !=
                    node_state_machine::STABLE &&
                manop_state_->mode ==
                    manual_operating_mode::CHANGE_SERVO_POWER) {
                canvas_->printf("SRV Power: OFF (SM is not Stable !)\r\n");
            } else {
                if (ctrl_state_->is_power_on) {
                    canvas_->printf("SRV Power: ON\r\n");
                } else {
                    canvas_->printf("SRV Power: OFF\r\n");
                }
            }
        }

        if (manop_state_->mode ==
                manual_operating_mode::CHANGE_SERVO_CONTROL_MODE &&
            manop_state_->act_phase == manual_operating_phase::VALUE_CHANGE &&
            display_blink_cnt % 5 == 0) {
            canvas_->printf("SRV Ctrl Mode: \r\n");
        } else {
            switch (ctrl_state_->ctrl_mode) {
                case basic_servo_ctrl_cmd_list::TORQUE:
                    canvas_->printf("SRV Ctrl Mode: TORQUE\r\n");
                    break;
                case basic_servo_ctrl_cmd_list::VELOCITY:
                    canvas_->printf("SRV Ctrl Mode: VELOCITY\r\n");
                    break;
                case basic_servo_ctrl_cmd_list::POSITION:
                    canvas_->printf("SRV Ctrl Mode: POSITION\r\n");
                    break;
                case basic_servo_ctrl_cmd_list::STAY:
                    canvas_->printf("SRV Ctrl Mode: STAY\r\n");
                    break;
                default:
                    break;
            }
        }

        if (ctrl_state_->is_init_joint_pos) {
            canvas_->printf("Init Joint: True\r\n");
        } else {
            canvas_->printf("Init Joint: False\r\n");
        }

        if (ctrl_state_->is_init_joint_pos ||
            ctrl_state_->act_can_connection_status) {
            canvas_->printf("ActJ Pos: %.3f \t Vel: %.2f \t Trq: %.3f \r\n",
                            ctrl_state_->act_joint_position,
                            ctrl_state_->act_joint_velocity,
                            ctrl_state_->act_joint_torque);
            canvas_->printf("CmdJ Pos: %.3f \t Vel: %.2f \t Trq: %.3f \r\n",
                            ctrl_state_->cmd_joint_position,
                            ctrl_state_->cmd_joint_velocity,
                            ctrl_state_->cmd_joint_torque);
        }
    }

    // << END Control Status

    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);
    if (system_state_->is_init_lan) {
        canvas_->printf(">> LAN Status\r\n");
        canvas_->printf("Local IP: %s(%d), \t Num: %d\r\n", ip_local_.c_str(),
                        recv_port_, system_state_->udp_recv_num);
        canvas_->printf("Dst   IP: %s(%d), \t Num: %d\r\n",
                        ip_destination_.c_str(), send_port_,
                        system_state_->udp_send_num);
        // canvas->printf("Recv Port: %d \t Send Port: %d\r\n", recv_port,
        //                send_port);
        // canvas->printf("Recv Num: %d \t  Send Num: %d\r\n",
        //                system_state_->udp_recv_num,
        //                system_state_->udp_send_num);
    } else {
        canvas_->printf(">> LAN Status\r\n");
        canvas_->printf("...Not Exist...\r\n");
    }

    // <<-- END Rewrite the canvas

    // Update the canvas_
    canvas_->endWrite();
    canvas_->pushSprite(0, 0);

    display_blink_cnt++;
    if (display_blink_cnt > 5) {
        display_blink_cnt = 0;
    }
}

void ModuleDisplay::small_display(manual_operating_state* manop_state_,
                                  ControlState* ctrl_state_,
                                  SystemState* system_state_,
                                  common_state_code* system_state_code) {
    // if (!is_init_canvas || !is_init_all) {
    if (!is_init_canvas) {
        return;
    }

    String str_op = "";
    String str_sm = "";
    String str_ctrl_mode = "";

    // Reset the canvas
    canvas_->clear();
    canvas_->setCursor(0, 0);

    switch (system_state_code->state_machine) {
        case node_state_machine::FORCE_STOP:
            canvas_->setPaletteColor(1, RED);
            break;
        case node_state_machine::READY:
            canvas_->setPaletteColor(1, WHITE);
            break;
        case node_state_machine::STABLE:
            canvas_->setPaletteColor(1, GREEN);
            break;
        default:
            canvas_->setPaletteColor(1, WHITE);
            break;
    }
    // Rewrite the canvas
    canvas_->startWrite(true);

    canvas_->setTextSize(TEXT_FONT_SIZE_SMALL);

    switch (manop_state_->mode) {
        case manual_operating_mode::NONE:
            str_op = " -  ";
            break;
        case manual_operating_mode::CHANGE_SM:
            str_op = " SM ";
            break;
        case manual_operating_mode::CHANGE_SERVO_ID:
            str_op = " ID ";
            break;
        case manual_operating_mode::CHANGE_SERVO_POWER:
            str_op = "PWR ";
            break;
        case manual_operating_mode::CHANGE_SERVO_CONTROL_MODE:
            str_op = "MODE";
            break;
        case manual_operating_mode::CMD_SERVO_CONTROL:
            str_op = "CTRL";
            break;
        case manual_operating_mode::CHANGE_LOGGING_MODE:
            str_op = "LOG ";
            break;
        default:
            break;
    }

    switch (system_state_code->state_machine) {
        case node_state_machine::INITIALIZING:
            str_sm = "I";
            break;
        case node_state_machine::READY:
            str_sm = "R";
            break;
        case node_state_machine::STABLE:
            str_sm = "S";
            break;
        case node_state_machine::FORCE_STOP:
            str_sm = "F";
            break;
        default:
            break;
    }

    switch (ctrl_state_->ctrl_mode) {
        case basic_servo_ctrl_cmd_list::TORQUE:
            str_ctrl_mode = "T";
            break;
        case basic_servo_ctrl_cmd_list::VELOCITY:
            str_ctrl_mode = "V";
            break;
        case basic_servo_ctrl_cmd_list::POSITION:
            str_ctrl_mode = "P";
            break;
        case basic_servo_ctrl_cmd_list::STAY:
            str_ctrl_mode = "-";
            break;
        default:
            break;
    }

    if (manop_state_->act_phase == manual_operating_phase::MODE_CHANGE &&
        display_blink_cnt % 5 == 0) {
        canvas_->printf("--:%s|SM:%s|Ct:%s|Ave:%dms|Max:%dms\r\n",
                        str_op.c_str(), str_sm.c_str(), str_ctrl_mode.c_str(),
                        system_state_->ave_calc_time_of_main_task,
                        system_state_->max_calc_time_of_main_task);
    } else {
        canvas_->printf("Op:%s|SM:%s|Ct:%s|Ave:%dms|Max:%dms\r\n",
                        str_op.c_str(), str_sm.c_str(), str_ctrl_mode.c_str(),
                        system_state_->ave_calc_time_of_main_task,
                        system_state_->max_calc_time_of_main_task);
    }
    if (ctrl_state_->is_init_joint_pos ||
        ctrl_state_->act_can_connection_status) {
        canvas_->printf(
            "Lv:%d|P:%.3f|V:%.2f|T:%.3f\r\n", manop_state_->ctrl_level,
            ctrl_state_->act_joint_position, ctrl_state_->act_joint_velocity,
            ctrl_state_->act_joint_torque);
    }

    // <<-- END Rewrite the canvas

    // Update the canvas_
    canvas_->endWrite();
    canvas_->pushSprite(0, 0);

    display_blink_cnt++;
    if (display_blink_cnt > 5) {
        display_blink_cnt = 0;
    }
}
