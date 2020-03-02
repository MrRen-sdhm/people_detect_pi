/*
 * InfraredRemote.h
 *
 *  Created on: 2018年3月11日
 *      Author: shuixiang
 */

#ifndef INFRAREDREMOTE_H_
#define INFRAREDREMOTE_H_

#include <cstdint>

class InfraredRemote {
public:

    enum {
        ERR_OK = 0, ERR_BAD_VALUE,
    };

    enum Mode {
        MODE_AUTO = 0x00,
        MODE_COOL = 0x04,
        MODE_HEAT = 0x01,
        MODE_DRY = 0x02,
        MODE_FAN = 0x06,
    };

    enum Wind {
        WIND_AUTO = 0x00, WIND_1 = 0x02, WIND_2 = 0x01, WIND_3 = 0x03,
    };

    enum Temp {
        TEMP_16 = 0x00,
        TEMP_17 = 0x08,
        TEMP_18 = 0x04,
        TEMP_19 = 0x0C,
        TEMP_20 = 0x02,
        TEMP_21 = 0x0A,
        TEMP_22 = 0x06,
        TEMP_23 = 0x0E,
        TEMP_24 = 0x01,
        TEMP_25 = 0x09,
        TEMP_26 = 0x05,
        TEMP_27 = 0x0D,
        TEMP_28 = 0x03,
        TEMP_29 = 0x0B,
        TEMP_30 = 0x07,
    };

    enum Power {
        POWER_ON = 0x01, POWER_OFF = 0x00,
    };

    InfraredRemote(int output_pin, bool active_high=true);

    virtual ~InfraredRemote();

    int set_power(Power power) {
        this->config.power = power;
        return 0;
    }

    int set_mode(Mode mode) {
        this->config.mode = mode;
        return 0;
    }

    int set_temp(Temp temp) {
        this->config.temp = temp;
        return 0;
    }

    int set_wind(Wind wind) {
        this->config.wind_speed = wind;
        return 0;
    }

    int send(void);
private:

    int output_pin;
    bool active_high;

    union {
        struct {
            // Tips:
            // 根据平台不同, 可能不支持跨字节的位域. 需要避免跨字节
            // 先出现的位域 位于 字节中的低位
            // byte 0
            uint8_t sleep :1;           // 睡眠
            uint8_t air_swing :1;       // 扫风
            uint8_t wind_speed :2;         // 风速
            uint8_t power :1;             // 开头
            uint8_t mode :3;               // 模式

            // byte 1
            uint8_t clock_0 :4;           // 定时
            uint8_t temp :4;               // 温度

            // byte 2
            uint8_t dry :1;             // 干燥
            uint8_t health :1;          // 健康
            uint8_t light :1;           // 灯光
            uint8_t humidification :1;  // 加湿
            uint8_t clock_1 :4;          // 定时

            // byte 3
            uint8_t fixed_0_0x0A :7;
            uint8_t ventilation :1;     // 换气

            // byte 4
            uint8_t fixed_2_0x00 :3;
            uint8_t swing_h :1;         // 左右扫风
            uint8_t fixed_1_0x00 :3;
            uint8_t swing_v :1;         // 上下扫风

            // byte 5
            uint8_t fixed_3_0x04 :6;
            uint8_t temp_display :2;    //  温度显示

            // byte 6
            uint8_t fixed_4_0x00 :8;

            // byte 7
            uint8_t checksum :4;        // 校验和
            uint8_t fixed_6_0x00 :1;
            uint8_t save_power :1;      //节能
            uint8_t fixed_5_0x00 :2;
        };
        uint8_t data[8];
    } config;

    void calc_checksum(void) {
        uint8_t mode = (((this->config.mode >> 2) & 0x01) << 0)
                | (((this->config.mode >> 1) & 0x01) << 1)
                | (((this->config.mode >> 0) & 0x01) << 2);
        uint8_t temp = (((this->config.temp >> 3) & 0x01) << 0)
                | (((this->config.temp >> 2) & 0x01) << 1)
                | (((this->config.temp >> 1) & 0x01) << 2)
                | (((this->config.temp >> 0) & 0x01) << 3);
        uint8_t checksum = 5 + (mode - 1) + temp;
        checksum = (((checksum >> 3) & 0x01) << 0)
                | (((checksum >> 2) & 0x01) << 1)
                | (((checksum >> 1) & 0x01) << 2)
                | (((checksum >> 0) & 0x01) << 3);
        checksum = (checksum & 0x0E)
                | ((checksum ^ ~this->config.power) & 0x01); //逆序与开关状态同或
        this->config.checksum = checksum;
    }

    static int wave_start;
    static int wave_stop;
    static int wave_logic_0;
    static int wave_logic_1;
};

#endif /* INFRAREDREMOTE_H_ */
