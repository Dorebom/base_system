#include "module_system_cmd.hpp"

void ModuleSystemCmd::setup(std::shared_ptr<node_cmd> cmd,
                            std::shared_ptr<node_state> system_state_data,
                            std::shared_ptr<node_state> control_state_data) {
    system_cmd_ = cmd;
    system_state_data_ = system_state_data;
    control_state_data_ = control_state_data;
}

void ModuleSystemCmd::set_cmd_change_state_machine(
    node_state_machine state_machine) {
    st_node_cmd cmd;
    cmd.default_init();

    // すでに同じ状態になっている場合は、何もしない
    if (state_machine == system_state_data_->state_code.state_machine) {
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
