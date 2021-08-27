/*
 * Current.c
 *
 *  Created on: Sep 4, 2020
 *      Author: mfeurtado
 */
#include "Current.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

/*ADC Input| Signal   |  Type      | SOC number
 * ADC  B0   Current-U   Current        B0
 * ADC  B1   Current-V   Current        B1
 * ADC  B2   Current-W   Current        B2
 * ADC  B3   Current-X   Current        B3
 * ADC  B4   Current-Y   Current        B4
 * ADC  B5   Current-Z   Current        B5
 */

float32_t getCurrentU(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentV(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentW(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentX(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentY(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER4)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentZ(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER5)/(float32_t)4095)-(float32_t)800;
    return val;
}
