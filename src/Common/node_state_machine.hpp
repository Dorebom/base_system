#pragma once

#include <string>

enum class node_state_machine : int
{
    UNCONFIGURED = 0,
    INITIALIZING = 1,
    READY = 2,
    STABLE = 3,
    REPAIR = 4,
    FORCE_STOP = 5,
    TRANSITING = 6
};