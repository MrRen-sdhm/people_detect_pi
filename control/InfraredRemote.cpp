/*
 * InfraredRemote.cpp
 *
 *  Created on: 2018年3月11日
 *      Author: shuixiang
 */

#include <string>
#include <stdexcept>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

#include <pigpio.h>

#include "InfraredRemote.h"

int InfraredRemote::wave_logic_0 = -1;
int InfraredRemote::wave_logic_1 = -1;
int InfraredRemote::wave_start = -1;
int InfraredRemote::wave_stop = -1;

static inline std::vector<gpioPulse_t> generateSquareWave(uint32_t output_pin,
        uint32_t usOn, uint32_t usOff, uint32_t count,
        bool active_high = true) {
    std::vector<gpioPulse_t> pulses;

    gpioPulse_t pulse_on, pulse_off;
    if (active_high) {
        pulse_on = gpioPulse_t { .gpioOn = static_cast<uint32_t>(1)
                << output_pin, .gpioOff = 0, .usDelay = usOn };
        pulse_off = gpioPulse_t { .gpioOn = 0, .gpioOff =
                static_cast<uint32_t>(1) << output_pin, .usDelay = usOff };
    } else {
        pulse_on = gpioPulse_t { .gpioOn = 0, .gpioOff =
                static_cast<uint32_t>(1) << output_pin, .usDelay = usOn };
        pulse_off = gpioPulse_t { .gpioOn = static_cast<uint32_t>(1)
                << output_pin, .gpioOff = 0, .usDelay = usOff };
    }

    for (int i = 0; i < count; i += 1) {
        pulses.push_back(pulse_on);
        pulses.push_back(pulse_off);
    }
    return pulses;
}

InfraredRemote::InfraredRemote(int output_pin, bool active_high) {
    // 配置数据初始化
    for (int i = 0; i < sizeof(this->config); i++) {
        this->config.data[i] = 0x00;
    }
    this->config.fixed_0_0x0A = 0x0A;
    this->config.fixed_1_0x00 = 0x00;
    this->config.fixed_2_0x00 = 0x00;
    this->config.fixed_3_0x04 = 0x04;
    this->config.fixed_4_0x00 = 0x00;
    this->config.fixed_5_0x00 = 0x00;
    this->config.fixed_6_0x00 = 0x00;

    this->config.light = 0x01;
    this->set_power(this->POWER_OFF);
    this->set_temp(this->TEMP_25);
    this->set_wind(this->WIND_AUTO);
    this->set_mode(this->MODE_AUTO);

    // 输出引脚初始化 BCM27, active low
    this->output_pin = output_pin;
    this->active_high = active_high;

    gpioSetMode(this->output_pin, PI_OUTPUT);
    gpioWrite(this->output_pin, active_high ? 0 : 1);

    // 初始化波形 38kHz, 50% duty
    int freq = 38000;
    float duty = 0.5;
    uint32_t usOn = static_cast<int>(1000000.0 / freq * duty);
    uint32_t usOff = static_cast<int>(1000000.0 / freq * duty);

    // start code: 9ms high -> 4.5ms low
    if (this->wave_start < 0) {
        std::vector<gpioPulse_t> pulses = generateSquareWave(output_pin, usOn,
                usOff, static_cast<int>(0.009 * freq), this->active_high);
        pulses[pulses.size() - 1].usDelay += 4500;

        int ret = gpioWaveAddGeneric(pulses.size(), pulses.data());
        if (ret < 0) {
            throw std::runtime_error(
                    std::string("gpioWaveAddGeneric creation failed with code ")
                            + std::to_string(ret));
        }

        this->wave_start = gpioWaveCreate();
        if (this->wave_start < 0) {
            throw std::runtime_error(
                    std::string("wave_start creation failed with code ")
                            + std::to_string(this->wave_start));
        }
    }

    // stop code: 0.6ms high -> 20ms low
    if (this->wave_stop < 0) {
        std::vector<gpioPulse_t> pulses = generateSquareWave(output_pin, usOn,
                usOff, static_cast<int>(0.0006 * freq), this->active_high);
        pulses[pulses.size() - 1].usDelay += 20000;

        int ret = gpioWaveAddGeneric(pulses.size(), pulses.data());
        if (ret < 0) {
            throw std::runtime_error(
                    std::string("gpioWaveAddGeneric creation failed with code ")
                            + std::to_string(ret));
        }

        this->wave_stop = gpioWaveCreate();
        if (this->wave_stop < 0) {
            throw std::runtime_error(
                    std::string("wave_stop creation failed with code ")
                            + std::to_string(this->wave_stop));
        }
    }

    // logic 1: 0.6ms high -> 0.6ms low
    if (this->wave_logic_1 < 0) {
        std::vector<gpioPulse_t> pulses = generateSquareWave(output_pin, usOn,
                usOff, static_cast<int>(0.0006 * freq), this->active_high);
        pulses[pulses.size() - 1].usDelay += 1600;

        int ret = gpioWaveAddGeneric(pulses.size(), pulses.data());
        if (ret < 0) {
            throw std::runtime_error(
                    std::string("gpioWaveAddGeneric creation failed with code ")
                            + std::to_string(ret));
        }

        this->wave_logic_1 = gpioWaveCreate();
        if (this->wave_logic_1 < 0) {
            throw std::runtime_error(
                    std::string("wave_logic_1 creation failed with code ")
                            + std::to_string(this->wave_logic_1));
        }
    }

    // logic 0: 0.6ms high -> 1.66ms low
    if (this->wave_logic_0 < 0) {
        std::vector<gpioPulse_t> pulses = generateSquareWave(output_pin, usOn,
                usOff, static_cast<int>(0.0006 * freq), this->active_high);
        pulses[pulses.size() - 1].usDelay += 600;

        int ret = gpioWaveAddGeneric(pulses.size(), pulses.data());
        if (ret < 0) {
            throw std::runtime_error(
                    std::string("gpioWaveAddGeneric creation failed with code ")
                            + std::to_string(ret));
        }

        this->wave_logic_0 = gpioWaveCreate();
        if (this->wave_logic_0 < 0) {
            throw std::runtime_error(
                    std::string("wave_logic_0 creation failed with code ")
                            + std::to_string(this->wave_logic_0));
        }
    }
}

InfraredRemote::~InfraredRemote() {
}

int InfraredRemote::send() {

    this->calc_checksum();

    // start code: 9ms high -> 4.5ms low
    // data byte 0-3
    // link code: logic 0 -> logic 1 -> logic 0 -> 0.6ms high -> 20ms low
    // data byte 4-7
    // stop code: 0.6ms high -> 20ms low

    // logic 1: 0.6ms high -> 0.6ms low
    // logic 0: 0.6ms high -> 1.66ms low

    std::vector<char> wave_chain;
    int size = 0;

    wave_chain.push_back(this->wave_start);

    for (int i = 0; i < 4; i++) {
        uint8_t data = this->config.data[i];
        for (int j = 0; j < 8; j++) {
            if (data & 0x80) {
                wave_chain.push_back(this->wave_logic_1);
            } else {
                wave_chain.push_back(this->wave_logic_0);
            }
            data <<= 1;
        }
    }

    wave_chain.push_back(this->wave_logic_0);
    wave_chain.push_back(this->wave_logic_1);
    wave_chain.push_back(this->wave_logic_0);
    wave_chain.push_back(this->wave_stop);

    for (int i = 4; i < 8; i++) {
        uint8_t data = this->config.data[i];
        for (int j = 0; j < 8; j++) {
            if (data & 0x80) {
                wave_chain.push_back(this->wave_logic_1);
            } else {
                wave_chain.push_back(this->wave_logic_0);
            }
            data <<= 1;
        }
    }

    wave_chain.push_back(this->wave_stop);

//    while (1) {
//        int ret = gpioWaveChain(wave_chain.data(), wave_chain.size());
//        while (gpioWaveTxBusy() == 1)
//            ;
//        std::cout << "Sent" << std::endl;
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//    }
    int ret = gpioWaveChain(wave_chain.data(), wave_chain.size());
    if (ret < 0) {
        throw std::runtime_error(
                std::string("gpioWaveChain failed with code ")
                        + std::to_string(ret));
    }

    return ret;
}
