/*
 * Relay.h
 *
 *  Created on: 2018年3月12日
 *      Author: shuixiang
 */

#ifndef CONTROL_RELAY_H_
#define CONTROL_RELAY_H_

class Relay {
public:
    Relay();
    virtual ~Relay();
    int set(bool state);
    bool get();
private:
    int output_pin;     // BCM pin
    bool state;         // current state, true: on, false: off
};

#endif /* CONTROL_RELAY_H_ */
