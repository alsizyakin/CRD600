/*
 * RESOLVER.c
 *
 *  Created on: Sep 16, 2020
 *      Author: mfeurtado
 */
#include "RESOLVER.h"
#include "driverlib.h"
#include <math.h>

float RSLV_TS;
float RSLV_RADIAN;
float RLSV_SINE;
float RSLV_MF;
#define RSLV_FS 100000
#define RSLV_FFUND 10000
#define RSLV_PERIOD 100e6/RSLV_FS // SWITCHING_FREQ is 100 kHz
#define PI 3.141592654  // Pi

void initResolverGPIO()
{

    /*
    163 RSLV-A-PWM (ePWM10A)
    164 RSLV-B-PWM (ePWM10B)
    */
    GPIO_setPadConfig(163, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO163
    GPIO_setPadConfig(164, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO164
    GPIO_setPinConfig(GPIO_163_EPWM10A);               // GPIO163 = PWM10A
    GPIO_setPinConfig(GPIO_164_EPWM10B);               // GPIO164 = PWM10B

    // PS-CONTROL +15V for resolver excitation amplifier
    //
    // Enable GPIO outputs on GPIO88 set it LOW
    /*
    88 SHUTDOWN+15V
    */
    GPIO_setPadConfig(88, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO88
    GPIO_writePin(88, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_88_GPIO88);              // GPIO88 = GPIO88
    GPIO_setDirectionMode(88, GPIO_DIR_MODE_OUT);   // GPIO88 = output


}




/*
88 SHUTDOWN+15V
*/

void enablePos15V()
{
    GPIO_writePin(88,0);
}

void disablePos15V()
{
    GPIO_writePin(88,1);
}


void initResolverPWM(void)
{

    //
    // Set-up TBCLK
    //

    EPWM_setTimeBasePeriod(EPWM10_BASE, RSLV_PERIOD);
    EPWM_setPhaseShift(EPWM10_BASE, 0); //if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM10_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM10_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO); //module EPWM10 is master

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM10_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                RSLV_PERIOD*0.5); //start with 50%

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM10_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(EPWM10_BASE); //master has no phase shift
    EPWM_setClockPrescaler(EPWM10_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM10_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);



    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM10_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM10_BASE);
    EPWM_setInterruptEventCount(EPWM10_BASE, 1U); //Interrupt every switching cycle for (required for sine math)


    RSLV_TS = 2*PI/( RSLV_FS / RSLV_FFUND );
    RSLV_RADIAN = 0;
    RSLV_MF = 0.12;
}

//
// epwm10ISR - ePWM 10 ISR
//
__interrupt void epwm10ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //

    // Sine wave math
    RLSV_SINE = ((((RSLV_MF*sin(RSLV_RADIAN))+1.0))/2);
    RSLV_RADIAN += RSLV_TS;

    if(RSLV_RADIAN > 2*PI)
        RSLV_RADIAN -= (2*PI);

    EPWM_setCounterCompareValue(EPWM10_BASE, EPWM_COUNTER_COMPARE_A, RLSV_SINE * RSLV_PERIOD );


    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM10_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


