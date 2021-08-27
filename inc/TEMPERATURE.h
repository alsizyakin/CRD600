/*
 * TEMPERATURE.h
 *
 *  Created on: Jun 11, 2019
 *      Author: mfeurtado
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_
#include "device.h"

float32_t getCaseTemp();
float32_t getAnalogTempU();
float32_t getAnalogTempV();
float32_t getAnalogTempW();
float32_t getAnalogTempX();
float32_t getAnalogTempY();
float32_t getAnalogTempZ();

float32_t getAnalogNTCA();
float32_t getAnalogNTCB();
float32_t getAnalogNTCC();

#endif /* TEMPERATURE_H_ */
