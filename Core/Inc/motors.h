/*
 * motors.h
 *
 *  Created on: Oct 27, 2024
 *      Author: xiaofei
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"
#include "sysTick.h"

void get_motors_speed();
void set_motors_speed();
void set_motors_output();

#endif /* INC_MOTORS_H_ */
