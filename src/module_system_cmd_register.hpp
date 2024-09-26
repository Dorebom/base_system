#pragma once

#include <memory>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
#include "Common/node_state_machine.hpp"
#include "st_control_state.hpp"
#include "st_system_state.hpp"

class ModuleSystemCmdRegister {
private:
    common_state_code* system_state_code_;
    common_state_code* control_state_code_;
    ControlState* control_state_;
    SystemState* system_state_;

    //  >> Stack
    std::shared_ptr<node_cmd> system_cmd_;
    // << Stack
public:
    ModuleSystemCmdRegister(/* args */) {
    }
    ~ModuleSystemCmdRegister() {
    }

    void setup(std::shared_ptr<node_cmd> cmd, SystemState* system_state,
               ControlState* control_state,
               common_state_code* system_state_code,
               common_state_code* control_state_code);

    void change_state_machine(node_state_machine state_machine);
    void change_servo_id(uint8_t servo_id);
    void change_servo_power(uint8_t servo_id, bool is_power_on);
    void change_servo_ctrl_mode(uint8_t servo_id,
                                basic_servo_ctrl_cmd_list ctrl_mode);
    void position_control(uint8_t servo_id, double target_position);
    void velocity_control(uint8_t servo_id, double velocity);
    void torque_control(uint8_t servo_id, double torque);

    void start_logging();
    void stop_logging();
};