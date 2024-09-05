#pragma once

#include <cstdint>

struct ControlState
{
    bool is_init_scale;
    bool dummy[7];

    float sensor_weight;
    int32_t sensor_weight_raw_adc;

    ControlState() {
        is_init_scale = false;
        sensor_weight = 0.0f;
        sensor_weight_raw_adc = 0.0;
    }

    void deepcopy(const ControlState& state) {
        is_init_scale = state.is_init_scale;
        sensor_weight = state.sensor_weight;
        sensor_weight_raw_adc = state.sensor_weight_raw_adc;
    }
};