#include "module_system_cmd_register.hpp"

void ModuleSystemCmdRegister::setup(std::shared_ptr<node_cmd> cmd,
                                    SystemState* system_state,
                                    ControlState* control_state,
                                    common_state_code* system_state_code,
                                    common_state_code* control_state_code) {
    system_cmd_ = cmd;
    system_state_ = system_state;
    control_state_ = control_state;
    system_state_code_ = system_state_code;
    control_state_code_ = control_state_code;
}

void ModuleSystemCmdRegister::change_state_machine(
    node_state_machine state_machine) {
    st_node_cmd cmd;
    cmd.default_init();

    // すでに同じ状態になっている場合は、何もしない
    if (state_machine == system_state_code_->state_machine) {
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
    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::change_servo_id(uint8_t servo_id) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_code_->state_machine != node_state_machine::READY) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_CONTROLLED_SRV_ID;
    cmd_change_controlled_servo_id* cmd_data =
        (cmd_change_controlled_servo_id*)cmd.data;
    cmd_data->servo_id = servo_id;

    cmd.cmd_code.data_size = sizeof(cmd_change_controlled_servo_id);

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::change_servo_power(uint8_t servo_id,
                                                 bool is_power_on) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_code_->state_machine != node_state_machine::STABLE) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SRV_POWER;
    cmd_change_servo_power* cmd_data = (cmd_change_servo_power*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->is_on = is_power_on;

    cmd.cmd_code.data_size = sizeof(cmd_change_servo_power);

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::change_servo_ctrl_mode(
    uint8_t servo_id, basic_servo_ctrl_cmd_list ctrl_mode) {
    st_node_cmd cmd;
    cmd.default_init();

    if (system_state_code_->state_machine == node_state_machine::FORCE_STOP) {
        return;
    }

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::CHANGE_SRV_CTRLMODE;
    cmd_change_servo_ctrl_mode* cmd_data =
        (cmd_change_servo_ctrl_mode*)cmd.data;
    cmd_data->servo_id = servo_id;
    cmd_data->ctrl_mode = ctrl_mode;

    cmd.cmd_code.data_size = sizeof(cmd_change_servo_ctrl_mode);

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::position_control(uint8_t servo_id,
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

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::velocity_control(uint8_t servo_id,
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

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::torque_control(uint8_t servo_id, double torque) {
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

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::start_logging() {
    st_node_cmd cmd;
    cmd.default_init();

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::START_LOGGING;

    system_cmd_->cmd_stack_.push(cmd);
}

void ModuleSystemCmdRegister::stop_logging() {
    st_node_cmd cmd;
    cmd.default_init();

    cmd.cmd_code.cmd_type = basic_m5stack_cmd_list::STOP_LOGGING;

    system_cmd_->cmd_stack_.push(cmd);
}
