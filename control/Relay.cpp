/*
 * Relay.cpp
 *
 *  Created on: 2018年3月12日
 *      Author: shuixiang
 */

#include <pigpio.h>

#include "Relay.h"

Relay::Relay() {
    this->output_pin = 4;
    this->state = false;
    this->set(false);
}

int Relay::set(bool state) {
    if (state) {
        gpioSetMode(this->output_pin, PI_OUTPUT);
        gpioWrite(this->output_pin, 1);
    } else {
        gpioSetMode(this->output_pin, PI_INPUT);
    }
}

bool Relay::get() {
    return this->state;
}

Relay::~Relay() {
    this->set(false);
}

