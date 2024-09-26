#pragma once

#include <M5GFX.h>

#include "Common/node_state.hpp"
#include "DataStruct/st_manual_operating.hpp"
#include "st_control_state.hpp"
#include "st_system_state.hpp"

#define TITLE_FONT_SIZE      2
#define TEXT_FONT_SIZE       1.5
#define TEXT_FONT_SIZE_SMALL 1.3

class ModuleDisplay {
private:
    M5Canvas* canvas_;
    bool is_init_canvas = false;
    bool display_heart_beat = false;
    uint8_t display_blink_cnt = 0;

    // >> LAN and UDP
    String ip_local_;
    String ip_destination_;
    uint32_t recv_port_;
    uint32_t send_port_;

    bool prev_is_logging = false;

    void normal_display(manual_operating_state* manop_state_,
                        ControlState* ctrl_state_, SystemState* system_state_,
                        common_state_code* system_state_code);
    void small_display(manual_operating_state* manop_state_,
                       ControlState* ctrl_state_, SystemState* system_state_,
                       common_state_code* system_state_code);

public:
    ModuleDisplay(/* args */) {
    }
    ~ModuleDisplay() {
    }
    void setup(M5Canvas* canvas);
    void set_lan_info(String ip_address_local, String ip_address_dst,
                      uint32_t recv_port, uint32_t send_port);
    void update(manual_operating_state* manop_state_, ControlState* ctrl_state_,
                SystemState* system_state_,
                common_state_code* system_state_code);
};