#pragma once

#include <memory>

#include "Common/node_cmd.hpp"
#include "Common/node_state.hpp"
#include "Common/node_state_machine.hpp"
#include "DataStruct/st_manual_operating.hpp"
#include "Sensor/unit_encoder.h"
#include "module_system_cmd_register.hpp"
#include "st_control_state.hpp"
#include "st_system_state.hpp"

class ModuleManualOperation {
private:
    common_state_code* system_state_code_;
    common_state_code* control_state_code_;
    ControlState* control_state_;
    SystemState* system_state_;

    ModuleSystemCmdRegister* syscmd_reg_;

    manual_operating_state* manual_operating_state_;

    // Encoder Button
    Unit_Encoder encoder_button;

public:
    ModuleManualOperation(/* args */) {
    }
    ~ModuleManualOperation() {
    }
    void setup(std::shared_ptr<node_cmd> cmd, SystemState* system_state,
               ControlState* control_state,
               common_state_code* system_state_code,
               common_state_code* control_state_code,
               manual_operating_state* manual_operating_state,
               ModuleSystemCmdRegister* syscmd_reg);

    // Encoder Button
    void begin_encoder_button(TwoWire* wire = &Wire,
                              uint8_t addr = ENCODER_ADDR);
    void update_encoder_button(bool force_stop_status);

    void update();
};