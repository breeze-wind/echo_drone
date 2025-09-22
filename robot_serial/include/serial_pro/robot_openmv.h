//
// Created by Elsa on 24-10-5.
//

#ifndef SERIALPRO_ROBOT_OPENMV_H
#define SERIALPRO_ROBOT_OPENMV_H

#include "serial_pro.h"

namespace openmv{
    message_data Head{
        uint8_t SOF = 0xA5;
        uint8_t cmd_id = 0xaa;
        uint8_t length = 0;
    };
    message_data Tail {
        uint8_t crc16 = 0xbb;
    };
    class OpenmvcamSerial : public sp::serialPro<Head, Tail>{
    public:
        OpenmvcamSerial() = default;
        OpenmvcamSerial(std::string s,int band):sp::serialPro<Head,Tail>(s,band)
    {
        registerChecker([](const Head &head)->int {return head.SOF!=0xA5;});  //返回0表示没有错误

        registerChecker([](const Tail &tail,const uint8_t *,const int &)->int{return tail.crc16!=0xbb;});

        setGetId([](const Head &head)->int{return head.cmd_id;});  //返回命令字

        setGetLength([](const Head &head)->int{return head.length;});  //返回长度
    }
    };
}



#endif //SERIALPRO_ROBOT_OPENMV_H
