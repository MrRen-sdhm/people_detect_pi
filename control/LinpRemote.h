/*
 * LinpRemote.h
 *
 *  Created on: 2018年3月12日
 *      Author: shuixiang
 */

#ifndef CONTROL_LINPREMOTE_H_
#define CONTROL_LINPREMOTE_H_

#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <thread>
#include <chrono>

struct LinpWirelessFrame {
    uint8_t type;               // 帧类型
    uint32_t addr;              // 源地址
    uint8_t sensor;             // 设备类型
    std::vector<uint8_t> data;  // 数据

    // construct frame raw data
    std::vector<uint8_t> make_frame() {
        std::vector<uint8_t> raw_data;
        raw_data.push_back(type);
        raw_data.push_back(static_cast<uint8_t>(addr >> 24));
        raw_data.push_back(static_cast<uint8_t>(addr >> 16));
        raw_data.push_back(static_cast<uint8_t>(addr >> 8));
        raw_data.push_back(static_cast<uint8_t>(addr >> 0));
        raw_data.push_back(sensor);
        raw_data.insert(raw_data.end(), data.begin(), data.end());
        return raw_data;
    }

    operator std::vector<uint8_t>() {
        return this->make_frame();
    }
};

struct LinpReceiverFrame {
    const uint8_t start = 0x55;       // 引导码
    uint16_t data_len;          // data数据字段长度
    uint8_t opt_data_len;       // opt_data可选数据字段长度
    uint8_t type;               // 帧类型
    uint8_t crc_header;         // 帧头CRC8校验
    std::vector<uint8_t> data;     // 数据
    std::vector<uint8_t> opt_data;  // 可选数据
    uint8_t crc_data;           // data字段 + opt字段的CRC8校验
private:
    static const uint8_t CRC8_TABLE[256];

    static uint8_t CRC8(uint8_t *packet, int length, uint8_t init_crc = 0x00) {
        uint8_t crc_result = init_crc;
        for (int i = 0; i < length; i++) {
            crc_result = CRC8_TABLE[crc_result ^ packet[i]];
        }
        return crc_result;
    }
public:
    bool check_header_crc() {
        std::vector<uint8_t> header = { static_cast<uint8_t>(data_len >> 8),
                static_cast<uint8_t>(data_len >> 0), opt_data_len, type };
        if (this->crc_header != CRC8(header.data(), header.size())) {
            return false;
        }
        return true;
    }

    bool check_data_crc() {
        uint8_t crc8 = CRC8(this->data.data(), this->data.size());
        if (this->crc_data
                != CRC8(this->opt_data.data(), opt_data.size(), crc8)) {
            return false;
        }
        return true;
    }

    bool check_frame() {
        if (this->data_len != this->data.size()) {
            return false;
        }
        if (this->opt_data_len != this->opt_data.size()) {
            return false;
        }

        // calc CRC of header
        if (!check_header_crc()) {
            return false;
        }
        // calc CRC of data + opt_data
        if (!check_data_crc()) {
            return false;
        }
        return true;
    }

    // 计算字段长度, 并计算CRC8校验码, 返回生成的整帧字节数组
    std::vector<uint8_t> make_frame() {

        // calc data length
        if (this->data.size() > 65535) {
            throw std::length_error(
                    "LinpWirelessFrame total length "
                            + std::to_string(this->data.size()) + " > 65535");
        }
        this->data_len = static_cast<uint16_t>(this->data.size());
        // calc opt_data length
        if (this->opt_data.size() > 255) {
            throw std::length_error(
                    "LinpReceiverFrame opt_data length "
                            + std::to_string(this->opt_data.size()) + " > 255");
        }
        this->opt_data_len = static_cast<uint8_t>(this->opt_data.size());

        // calc CRC of header
        std::vector<uint8_t> header = { static_cast<uint8_t>(data_len >> 8),
                static_cast<uint8_t>(data_len >> 0), opt_data_len, type };
        this->crc_header = CRC8(header.data(), header.size());
        // calc CRC of data + opt_data
        this->crc_data = CRC8(this->data.data(), this->data.size());
        this->crc_data = CRC8(this->opt_data.data(), opt_data.size(),
                this->crc_data);

        // construct frame raw data
        std::vector<uint8_t> raw_data;
        raw_data.push_back(this->start);
        raw_data.push_back(static_cast<uint8_t>(data_len >> 8));
        raw_data.push_back(static_cast<uint8_t>(data_len >> 0));
        raw_data.push_back(opt_data_len);
        raw_data.push_back(type);
        raw_data.push_back(crc_header);
        raw_data.insert(raw_data.end(), this->data.begin(), this->data.end());
        raw_data.insert(raw_data.end(), this->opt_data.begin(),
                this->opt_data.end());
        raw_data.push_back(crc_data);

        return raw_data;
    }

    int read_from(std::stringstream &in) {
        while (1) {
            std::vector<uint8_t> data_read;
            std::streamsize ret;

            // find frame start
            char start = 0x00;
            do {
                ret = in.readsome(&start, 1);
            } while (ret == 1 && start != this->start);
            if (start != this->start) {
                throw std::runtime_error("can't find frame start");
            }
            data_read.push_back(start);

            // get header
            char header[5];
            ret = in.readsome(header, 5);
            data_read.insert(data_read.end(), header, header + ret);
            if (ret < 5) {
                for (int i = 0; i < data_read.size(); i++) {
                    in.putback(data_read[i]);
                }
                throw std::runtime_error(
                        "unfinished frame header" + std::to_string(ret) + "/5");
            }
            // assign header
            this->data_len = static_cast<uint16_t>(data_read[1] << 8)
                    + data_read[2];
            this->opt_data_len = data_read[3];
            this->type = data_read[4];
            this->crc_header = data_read[5];
            // check header
            if (!this->check_header_crc()) {
                // crc not match
                for (int i = 1; i < data_read.size(); i++) {
                    in.putback(data_read[i]);
                }
                continue;
            }

            // get data
            std::vector<char> data(this->data_len + this->opt_data_len + 1);
            ret = in.readsome(data.data(), data.size());
            data_read.insert(data_read.end(), data.begin(), data.end());
            if (ret < data.size()) {
                for (int i = 0; i < data_read.size(); i++) {
                    in.putback(data_read[i]);
                }
                throw std::runtime_error(
                        "unfinished frame data " + std::to_string(ret) + "/"
                                + std::to_string(data.size()));
            }
            // assign data, opt_data, crc_data
            this->data = std::vector<uint8_t>(data.begin(),
                    data.begin() + this->data_len);
            this->opt_data = std::vector<uint8_t>(data.begin() + this->data_len,
                    data.end() - 1);
            this->crc_data = data[data.size() - 1];
            if (!this->check_data_crc()) {
                // crc not match
                for (int i = 1; i < data_read.size(); i++) {
                    in.putback(data_read[i]);
                }
                continue;
            }

            return 0;
        }
    }

    operator std::vector<uint8_t>() {
        return this->make_frame();
    }
};

class LinpRemote {
public:
    LinpRemote();

    virtual ~LinpRemote();

    int ping(int timeout_ms = -1) {
        std::chrono::high_resolution_clock::time_point measure_time =
                std::chrono::high_resolution_clock::now();

        LinpReceiverFrame ping_frame;
        ping_frame.type = 0x05;
        ping_frame.data = std::vector<uint8_t> { 0x01 };
        this->send(ping_frame);

        LinpReceiverFrame ack_frame;
        while (1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            try {
                int ret = this->recv(ack_frame);
                if (ack_frame.type != 0x06 || ack_frame.data.size() != 1
                        || ack_frame.data[0] != 0x01) {
                    std::cout << "bad ping ack" << std::endl;
                } else {
                    break;
                }
            } catch (const std::runtime_error &err) {
                std::cout << err.what() << std::endl;
                if (timeout_ms > 0
                        && std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::high_resolution_clock::now()
                                        - measure_time).count() > timeout_ms) {
                    return -1;
                }
            }
        }

        return 0;
    }

    int read_fw_ver(std::string &version) {
        LinpReceiverFrame read_fw_ver_frame;
        read_fw_ver_frame.type = 0x05;
        read_fw_ver_frame.data = std::vector<uint8_t> { 0x02 };
        this->send(read_fw_ver_frame);

        LinpReceiverFrame ack_frame;
        while (1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            try {
                int ret = this->recv(ack_frame);
                if (ack_frame.type != 0x06 || ack_frame.data.size() != 4
                        || ack_frame.data[0] != 0x02) {
                    std::cout << "bad read_fw_ver ack" << std::endl;
                } else {
                    break;
                }
            } catch (const std::runtime_error &err) {
                std::cout << err.what() << std::endl;
            }
        }

        version = "V" + std::to_string(ack_frame.data[1]) + "."
                + std::to_string(ack_frame.data[2]) + "."
                + std::to_string(ack_frame.data[3]);

        return 0;
    }

    int set_switch(uint32_t addr, bool state) {
        LinpWirelessFrame wireless_frame;
        wireless_frame.type = 0x5F;
        wireless_frame.addr = addr;
        wireless_frame.sensor = 0x81;
        wireless_frame.data = std::vector<uint8_t> { 0x02, 0x01 };
        if (state) {
            wireless_frame.data.push_back(0x01);
        } else {
            wireless_frame.data.push_back(0x00);
        }

        LinpReceiverFrame recver_frame;
        recver_frame.type = 0x01;
        recver_frame.data = wireless_frame;
        recver_frame.opt_data = std::vector<uint8_t> { 0x01, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00 };

        this->send(recver_frame);
    }

    int send(LinpReceiverFrame &frame);

    int recv(LinpReceiverFrame &frame);
private:
    std::string ser_path;
    int ser_port;
    std::stringstream buffer;
};

#endif /* CONTROL_LINPREMOTE_H_ */
