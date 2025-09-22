#ifndef SERIALPRO_ROBOTREFEREE_H
#define SERIALPRO_ROBOTREFEREE_H

#include "serial_pro.h"

namespace referee {
    message_data head {
        uint8_t SOF = 0xA5;
        uint16_t length = 0;
        uint8_t seq = 0;
        uint8_t crc8 = 0;
        uint16_t cmd_id = 0;
    };
    message_data tail {
        uint16_t crc16 = 0;
    };
    class RefereeSerial : public sp::serialPro<head, tail> {
    public:
        enum error {
            lengthNotMatch = -2,        // 从下位机接受的消息长度与注册回调时传入的消息长度不匹配,90%就是上下位机的消息体定义的不一样
            rxLessThanLength = -1,      // 当前缓冲区中的消息不完整，下次解析时重试
            ok = 0,
            sofError,                   // 帧头1024不匹配
            crcError,                    // crc8校验结果错误
            crc16Error
        };
        RefereeSerial() = default;
         RefereeSerial(const std::string &port, int baud) : sp::serialPro<head, tail>(port, baud) {
        registerSetter([](head& h, int s) {
                h.length = s;
        });
        registerSetter([](head& h, int _) {
                h.crc8 = ms::crc8check((uint8_t*)&h, sizeof(head)-3);
        });
        registerSetter([](tail& t, const uint8_t* data, int s) {
                t.crc16 = ms::crc16check(data, s);
        });
        registerChecker([](const head &h) -> int {
                if (h.SOF == 0xA5) {
                    return ok;
                } else {
                    return sofError;
                }
        });
        registerChecker([](const head &h) -> int {
                if (h.crc8 == ms::crc8check((uint8_t*)&h, sizeof(head)-3)) {
                    return ok;
                } else {
                    return crcError;
                }
        });
        registerChecker([](const tail& t, const uint8_t* data, int s) {
                if (t.crc16 == ms::crc16check(data, s)) {
                    return ok;
                } else {
                    return crc16Error;
                }
        });
        setGetId([](const head& h) {
                return h.cmd_id;
        });
        setGetLength([](const head& h) {
                return h.length;
        });
        setListenerMaxSize(512);
        }
        RefereeSerial(const RefereeSerial &other) = delete;

        RefereeSerial(RefereeSerial &&other) noexcept: sp::serialPro<head, tail>(std::move(other)) {}

        using sp::serialPro<head, tail>::operator=;

        template<typename T>
        bool write(uint16_t id, uint8_t seq, const T& t) {
            return sp::serialPro<head, tail>::write(head{.seq=seq, .cmd_id=id}, t);
        }
    };
}
#endif //SERIALPRO_ROBOTREFEREE_H