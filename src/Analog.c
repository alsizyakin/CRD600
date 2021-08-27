/*
 * Analog.c
 *
 *  Created on: Sep 4, 2020
 *      Author: mfeurtado
 */
#include <Analog.h>
#include "driverlib.h"
#include "device.h"
#include <math.h>

/*ADC Input| Signal   |  Type      | SOC number
 * ADC  A0   Vsense-U    Voltage        A0
 * ADC  A1   Vsense-V    Voltage        A1
 * ADC  A2   Vsense-W    Voltage        A2
 * ADC  A3   Vsense-X    Voltage        A3
 * ADC  A4   Vsense-Y    Voltage        A4
 * ADC  A5   Vsense-Z    Voltage        A5!
 * ADC  A14  U-TEMP      Temperature    A6
 * ADC  A15  V-TEMP      Temperature    A7
 * ADC  B0   Current-U   Current        B0
 * ADC  B1   Current-V   Current        B1
 * ADC  B2   Current-W   Current        B2
 * ADC  B3   Current-X   Current        B3
 * ADC  B4   Current-Y   Current        B4
 * ADC  B5   Current-Z   Current        B5!
 * ADC  C2   W-TEMP      Temperature    C0
 * ADC  C3   X-TEMP      Temperature    C1
 * ADC  C4   Y-TEMP      Temperature    C2
 * ADC  C5   Z-TEMP      Temperature    C3
 * ADC  D0   Vsense-DC   Voltage        D0
 * ADC  D1   CASE-TEMP   Temperature    D1
 * ADC  D2   SIN-A       Resolver       D2
 * ADC  D3   COS-A       Resolver       D3
 * ADC  D4   SIN-B       Resolver       D4
 * ADC  D5   COS-B       Resolver       D5
 *
 *  !   ADCASOC5 is interrupt 1 for ADCA
 *  !   ADCBSOC5 is interrupt 1 for ADCB
 */


void initADCs(void){
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0);
    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);
    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);
    ADC_enableConverter(ADCD_BASE);

    DEVICE_DELAY_US(1000);
}
void initADCSOCs(void){
    //
    // Configure SOCs of ADCA
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN4, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN5, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN14, 15);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN15, 15);


    //
    // Set SOC5 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Configure SOCs of ADCB
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN4, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN5, 15);

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    //
    // Configure SOCs of ADCC
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN5, 15);


    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    //
    // Configure SOCs of ADCD
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 15);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 15);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN5, 15);


    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}

