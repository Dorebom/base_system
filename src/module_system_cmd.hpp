#pragma once

#include <memory>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
#include "Common/node_state_machine.hpp"

class ModuleSystemCmd {
private:
    std::shared_ptr<node_state> system_state_data_;
    std::shared_ptr<node_state> control_state_data_;
    //  >> Stack
    std::shared_ptr<node_cmd> system_cmd_;
    // << Stack
public:
    ModuleSystemCmd(/* args */) {
    }
    ~ModuleSystemCmd() {
    }
    void setup(std::shared_ptr<node_cmd> cmd,
               std::shared_ptr<node_state> system_state_data,
               std::shared_ptr<node_state> control_state_data);

    void set_cmd_change_state_machine(node_state_machine state_machine);
};