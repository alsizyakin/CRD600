/*
 * TEMPERATURE.c
 *
 *  Created on: Sep 4, 2020
 *      Author: mfeurtado
 */

#include "TEMPERATURE.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

uint32_t beta = 0x3435;

    /*
    GPIO-94   A-RTD -> eCAP1
    GPIO-95   C-RTD -> eCAP3
    GPIO-97   B-RTD -> eCAP2
    */

/*
* ADC Input| Signal   |  Type      | SOC number
* ADC  A14  U-TEMP      Temperature    A6
* ADC  A15  V-TEMP      Temperature    A7
* ADC  C2   W-TEMP      Temperature    C0
* ADC  C3   X-TEMP      Temperature    C1
* ADC  C4   Y-TEMP      Temperature    C2
* ADC  C5   Z-TEMP      Temperature    C3
* ADC  D1   CASE-TEMP   Temperature    D1
*/

// getCaseTemp
// measure 10k NTC on controller PCB with 10k pull-down
// RETURN: temperature in Kelvin

float32_t getCaseTemp()
{
    float32_t val, t;
    val = ((float32_t)ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095); //read ADC from CASE-TEMP
    val = val * 3.0F;
    //float32_t res;
    //res = ((3.3F*10000)/val) - 10000; //ADC is 3.0V reference but the NTC is tied to 3.3V
    t = 1.0F / ((logf(3.3F/val - 1.0F)/3900)+(1.0F/298.15F));
    return t;
}

// getAnalogTempU
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempU()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempV
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempV()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempW
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempW()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempX
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempX()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempY
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempY()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempZ
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempZ()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogNTCU
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCU()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCV
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCV()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCW
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCW()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCX
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCX()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCY
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCY()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCZ
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCZ()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}
