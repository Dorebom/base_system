#pragma once

#include <cstring>
#include <iostream>

#define MAX_UDP_STACK_MARKER_NUM     20
#define UDP_ONE_STACK_SIZE           50
#define MAX_UDP_SEND_STATE_DATA_SIZE 1200
// > MAX_STACK_MARKER_NUM * ONE_STACK_SIZE
#define MAX_STACK_SIZE_AT_ONCE 200

struct udp_frame
{
    // 固定ヘッダーのデータサイズ
    int udp_frame_header_size;  // フレームヘッダーのサイズ[byte]
    int fixed_state_header_data_size;  // 固定ヘッダーのデータサイズ[byte]
    int fixed_cmd_header_data_size;  // 固定ヘッダーのデータサイズ[byte]
    int max_stack_marker_size;       // スタックの最大数[box]
    int one_stack_size;              // 一つのスタックのサイズ[byte]
    int max_stack_size_at_once;  // 一度に送信するスタックの最大数[byte]
    // 以下が毎回可変なデータ
    // バイナリデータのスタック目印の総数
    int stack_marker_num;
    // バイナリデータのスタック目印を記録する配列
    std::int8_t stack_marker[MAX_UDP_STACK_MARKER_NUM];
    // バイナリデータをコピーする配列
    std::uint8_t data[MAX_UDP_SEND_STATE_DATA_SIZE];

    udp_frame(/* args */) {
        udp_frame_header_size =
            sizeof(udp_frame_header_size) +
            sizeof(fixed_state_header_data_size) +
            sizeof(fixed_cmd_header_data_size) + sizeof(max_stack_marker_size) +
            sizeof(one_stack_size) + sizeof(max_stack_size_at_once) +
            sizeof(stack_marker_num) +
            MAX_UDP_STACK_MARKER_NUM;  // size = 4 * 6 +
                                       // 20:MAX_UDP_STACK_MARKER_NUM = 26
        //
        fixed_state_header_data_size = 0;
        fixed_cmd_header_data_size = 0;
        max_stack_marker_size = MAX_UDP_STACK_MARKER_NUM;
        one_stack_size = UDP_ONE_STACK_SIZE;
        max_stack_size_at_once = MAX_STACK_SIZE_AT_ONCE;
        //
        stack_marker_num = 0;
        memset(stack_marker, 0, MAX_UDP_STACK_MARKER_NUM);
        memset(data, 0, MAX_UDP_SEND_STATE_DATA_SIZE);
    }
};