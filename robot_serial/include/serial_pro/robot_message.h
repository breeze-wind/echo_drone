//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

message_data openmv_info_t{
    uint8_t accurate;
    float image_x;
    float image_y;
};

#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H
