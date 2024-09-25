#pragma once

#include "st_system_state.hpp"

class ModuleManualOperation {
private:
public:
    ModuleManualOperation(/* args */) {
    }
    ~ModuleManualOperation() {
    }
    void setup();
    void update(SystemState* system_state_);
};