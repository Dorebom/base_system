#pragma once

#include <cstdint>

#include "Sensor/unit_scales.hpp"
#include "st_control_state.hpp"

struct control_status
{
    /* data */
    bool is_init_scale;

    control_status() {
        is_init_scale = false;
    }
};

class ControlManager {
private:
    UNIT_SCALES scale;

    // Data
    control_status status_;
    ControlState state_;
    bool is_init_all = false;

public:
    ControlManager();
    ~ControlManager();
    void update();

    void get_control_state(ControlState& state);
    bool check_init_all();

    // weight scale
    bool begin_scale(TwoWire& wire, uint8_t addr);
    void reset_scale();
    float get_weight();
    int32_t get_weightRawADC();
    // EMS Button
    void set_emergency_stop(bool em_stop);
};