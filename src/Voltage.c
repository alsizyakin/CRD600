/*
 * Voltage.c
 *
 *  Created on: Sep 4, 2020
 *      Author: mfeurtado
 */
#include "Voltage.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

/*ADC Input| Signal   |  Type      | SOC number
 * ADC  A0   Vsense-U    Voltage        A0
 * ADC  A1   Vsense-V    Voltage        A1
 * ADC  A2   Vsense-W    Voltage        A2
 * ADC  A3   Vsense-X    Voltage        A3
 * ADC  A4   Vsense-Y    Voltage        A4
 * ADC  A5   Vsense-Z    Voltage        A5
 * ADC  D0   Vsense-DC   Voltage        D0
 */

float32_t getVoltageU()
{
    float val;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095)-(float32_t)1200;
    return val;
}


float32_t getVoltageV()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095)-(float32_t)1200;
    return val;
}


float32_t getVoltageW()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095)-(float32_t)1200;
    return val;
}

float32_t getVoltageX()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3)/(float32_t)4095)-(float32_t)1200;
    return val;
}

float32_t getVoltageY()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4)/(float32_t)4095)-(float32_t)1200;
    return val;
}

float32_t getVoltageZ()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5)/(float32_t)4095)-(float32_t)1200;
    return val;
}


//VsenseDC is D0, scaling for Vdc = 4X
float32_t getVoltageDC()
{
    float val = 0;
    val = (float32_t)4800*((float32_t)ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095);
    return val;
}
