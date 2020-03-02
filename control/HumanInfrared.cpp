/*
 * HumanInfrared.cpp
 *
 *  Created on: 2018年3月16日
 *      Author: shuixiang
 */

#include <pigpio.h>

#include "HumanInfrared.h"

HumanInfrared::HumanInfrared(int input_pin, bool active_high) {
    this->input_pin = input_pin;
    this->active_high = active_high;
    gpioSetMode(this->input_pin, PI_INPUT);
}

HumanInfrared::~HumanInfrared() {
}


bool HumanInfrared::get() {
    if (this->active_high) {
        return gpioRead(this->input_pin);
    } else {
        return !gpioRead(this->input_pin);
    }
}
