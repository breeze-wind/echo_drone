//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

/* 用message_data定义全部要发送给下位机的消息的结构体 */
message_data Velocity{ //0x0501
    float v_x, v_y, v_w;
};
#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H
