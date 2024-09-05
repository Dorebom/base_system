#pragma once

#include "node_state_machine.hpp"
#include "stacker.hpp"

#define MAX_NODE_STATE_DATA_SIZE 200

struct common_state_code
{
    /* data */
    int node_id;                       // Func: user_node::_configure
    node_state_machine state_machine;  // Func: node::node_loop_processing
    node_state_machine
        transit_destination_node_state_machine;  // Func
                                                 // node::node_loop_processing
    int data_size;                               // Func: user_node::_configure
    //
    common_state_code(/* args */) {
        node_id = 0;
        state_machine = node_state_machine::UNCONFIGURED;
        transit_destination_node_state_machine =
            node_state_machine::UNCONFIGURED;
        data_size = 0;
    }
};

struct st_node_state
{
    common_state_code state_code;
    std::uint8_t data[MAX_NODE_STATE_DATA_SIZE];
};

typedef st_node_state node_state;

class node_state_stack {
private:
    /* data */
public:
    circular_stacker<st_node_state> state_stack_;
    node_state_stack(size_t stack_size) : state_stack_(stack_size) {
    }
    ~node_state_stack() {
    }
};