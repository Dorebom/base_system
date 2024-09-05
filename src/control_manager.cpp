#include "control_manager.hpp"

#include "logger.hpp"

ControlManager::ControlManager() {
}

ControlManager::~ControlManager() {
}

void ControlManager::get_control_state(ControlState& state) {
    state = state_;
}

bool ControlManager::check_init_all() {
    return is_init_all;
}

/*
 * WEIGHT SCALE
 */
bool ControlManager::begin_scale(TwoWire& wire, uint8_t addr) {
    uint8_t watchdog = 0;
    while (!scale.begin(&wire, addr)) {
        M5DEV_LOGE("Scale not found");
        status_.is_init_scale = false;
        delay(200);
        watchdog++;
        if (watchdog > 5) {
            M5DEV_LOGE("Scale not found. Break");
            return false;
        }
    }
    M5DEV_LOGI("Scale found");
    status_.is_init_scale = true;

    scale.setLEDColor(0x001000);
    // scale.setLPFilter(50);
    // scale.setAvgFilter(10);
    scale.setEmaFilter(50);
    scale.setGapValue(200.0f);
    scale.setOffset();
    // scale.setGapValue(0.0f);
    return true;
}
void ControlManager::reset_scale() {
    scale.setOffset();
    // scale.setGapValue(0.0f);
}
float ControlManager::get_weight() {
    if (!status_.is_init_scale) {
        M5_LOGW("Scale not found");
        return -1.0f;
    }
    if (scale.getBtnStatus() == 1) {
        scale.setLEDColor(0x001000);
    } else {
        scale.setLEDColor(0x000010);
    }
    M5DEV_LOGI("Scale Gap Value: %f", scale.getGapValue());
    float weight = scale.getWeight();
    state_.sensor_weight = weight;
    return weight;
}
int32_t ControlManager::get_weightRawADC() {
    if (!status_.is_init_scale) {
        return -1.0;
    }
    int32_t raw_adc = scale.getRawADC();
    state_.sensor_weight_raw_adc = raw_adc;
    return raw_adc;
}

void ControlManager::set_emergency_stop(bool em_stop) {
}
// << END WEIGHT SCALE
