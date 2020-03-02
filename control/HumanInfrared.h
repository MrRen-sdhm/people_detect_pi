/*
 * HumanInfrared.h
 *
 *  Created on: 2018年3月16日
 *      Author: shuixiang
 */

#ifndef CONTROL_HUMANINFRARED_H_
#define CONTROL_HUMANINFRARED_H_

class HumanInfrared {
public:
    HumanInfrared(int input_pin, bool active_high=true);
    virtual ~HumanInfrared();

    bool get();
private:
    int input_pin;
    bool active_high;
};

#endif /* CONTROL_HUMANINFRARED_H_ */
