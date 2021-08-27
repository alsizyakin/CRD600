//#############################################################################
//
// FILE:    main.c
//
// TITLE:   XM3 Dual Inverter Example Code
//
// AUTHOR: mfeurtado, Matthew Feurtado
//
// PURPOSE:
//  This software is designed for the evaluation of the XM3 Dual 3-phase Inverter
//  Reference Design (CRD600DA12E-XM3) and provides only the basic code
//  required to create an open-loop inverter. The basic 3-phase PWM parameters
//  can be controlled over a CAN interface along with rudimentary feedback from
//  the on-board sensors. It is designed as a starting point only and it is
//  left to the user any customization for a particular application.
//
////#############################################################################
// REVISION HISTORY:
//  V0.1 8/31/2020 Dual Inverter Example
//  -ported from XM3 Single Inverter Example V1.3 to Dual Controller
//  V0.2 1/14/2021 Dual Inverter Example
//      -add phase shift of 2 to compensate for EPWM_SYNCI delay
//      -add CAN control signals for XYZ
//      -remove eCAP
//      -add timer for CAN feedback
//  V1.0 2/1/2021: Dual Inverter Example
//      -Parallel operation of UVW and XYZ
//      -change VDC scaling to 4x
//      -change status LEDs for Dual Controller
//      -CAN update frequency ~600Hz for Current and Voltage
//      -CAN update frequency 2Hz for status and Temperature
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "GATEDRIVER.h"
#include "CANSetup.h"
#include "Analog.h"
#include "Temperature.h"
#include "Current.h"
#include "Voltage.h"
#include "RESOLVER.h"
#include <math.h>

//
// Globals
//
typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmPeriod;
    float epwmPwmPhase;
    uint16_t epwmDeadTime;
    float epwmRadian;
}epwmInformation;


//
// Globals to hold the ePWM information used in this example
//
epwmInformation epwm1Info;
epwmInformation epwm2Info;
epwmInformation epwm3Info;
epwmInformation epwm4Info;
epwmInformation epwm5Info;
epwmInformation epwm6Info;

//
// Function Prototypes
//
void initGPIO(void);

void TestLEDS(void);
void TestShutdown(void);
void TestLEDPulse(void);

void initTIMER1(void);

void initEPWM1(void);
void initEPWM2(void);
void initEPWM3(void);
void initEPWM4(void);
void initEPWM5(void);
void initEPWM6(void);
__interrupt void epwm1ISR(void);
__interrupt void epwm1TZISR(void);
__interrupt void epwm2TZISR(void);
__interrupt void epwm3TZISR(void);
__interrupt void epwm4TZISR(void);
__interrupt void epwm5TZISR(void);
__interrupt void epwm6TZISR(void);
__interrupt void cpuTimer1ISR(void);
__interrupt void adcAISR(void);
__interrupt void adcBISR(void);

void updatePWM(epwmInformation *epwmInfo);

void CANPacketEncode(uint16_t *PacketData);
void CANPacketDecode(uint16_t *PacketData);

uint16_t FS = 0;
uint16_t FF = 0;
uint16_t ID = 0;
uint16_t TD = 0;
uint16_t PSEN1 = 0;
uint16_t PSEN2 = 0;
uint16_t PSEN3 = 0;
uint16_t PSEN4 = 0;
uint16_t PSEN5 = 0;
uint16_t PSEN6 = 0;
uint16_t LEN1 = 0;
uint16_t LEN2 = 0;
uint16_t LEN3 = 0;
uint16_t LEN4 = 0;
uint16_t LEN5 = 0;
uint16_t LEN6 = 0;
uint16_t FAULT1 = 0;
uint16_t FAULT2 = 0;
uint16_t FAULT3 = 0;
uint16_t FAULT4 = 0;
uint16_t FAULT5 = 0;
uint16_t FAULT6 = 0;
uint16_t RESET = 0;
uint16_t FUND_FREQ;       // Fundamental frequency of sine wave
uint16_t EPwm_TBPRD;
float MF;               // Modulation factor or modulation depth (0 - 1)
uint16_t SWITCHING_FREQ;  // Switching frequency
uint16_t DEAD_TIME;       // Dead time in clock cycles (1 = 6.67 ns)

#define PI 3.141592654  // Pi
float Sine;             //
float Ts;               //
float radian;

uint16_t cpuTimer1IntCount; //interrupt count for Timer1
uint32_t TIMER1FREQ = 600; //frequency of TIMER1 for CAN update in Hz
//CAN packet buffers
uint16_t TemperatureMsgDataUVW[8];
uint16_t CurrentMsgDataUVW[8];
uint16_t VoltageMsgDataUVW[8];
uint16_t TemperatureMsgDataXYZ[8];
uint16_t CurrentMsgDataXYZ[8];
uint16_t VoltageMsgDataXYZ[8];

//
// Main
//
void main(void)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            {
    //
    // Initializes system control, device clock, and peripherals
    //
    Device_init();
    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();
    //
    // Initializes PIE and clear PIE registers. Disables CPU interrupts.
    // and clear all CPU interrupt flags.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
    //
    // Assign the interrupt service routines to ePWM interrupts
    //
    Interrupt_register(INT_EPWM1, &epwm1ISR);

    Interrupt_register(INT_EPWM10, &epwm10ISR); //resolver

    Interrupt_register(INT_EPWM1_TZ, &epwm1TZISR);
    Interrupt_register(INT_EPWM2_TZ, &epwm2TZISR);
    Interrupt_register(INT_EPWM3_TZ, &epwm3TZISR);
    Interrupt_register(INT_EPWM4_TZ, &epwm4TZISR);
    Interrupt_register(INT_EPWM5_TZ, &epwm5TZISR);
    Interrupt_register(INT_EPWM6_TZ, &epwm6TZISR);

    Interrupt_register(INT_TIMER1, &cpuTimer1ISR); //timer

    Interrupt_register(INT_ADCA1, &adcAISR); //ADCA
    Interrupt_register(INT_ADCB1, &adcBISR); //ADCB
    //
    // This example is a basic pinout
    //
    initGateDriverGPIO();
    GD_ALL_PSDisable(); //disable all gate drivers for startup
    initCANGPIO();
    initGPIO();
    initResolverGPIO();

    disablePos15V();


    //initialize switching parameters
    SWITCHING_FREQ = 10e3;   // Default of 20 kHz switching frequency
    DEAD_TIME = 100;         // 1.3us of dead time by default
    FUND_FREQ = 300;         // Default of 300 Hz fundamental frequency
    MF = 0.01;                // Default of 0.9 modulation depth
    radian = 0;              // Initialize radian to 0
    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);  // x

    LEN1 = 0;
    LEN2 = 0;
    LEN3 = 0;
    LEN4 = 0;
    LEN5 = 0;
    LEN6 = 0;
    PSEN1 = 0;
    PSEN2 = 0;
    PSEN3 = 0;
    PSEN4 = 0;
    PSEN5 = 0;
    PSEN6 = 0;
    FAULT1 = 0;
    FAULT2 = 0;
    FAULT3 = 0;
    FAULT4 = 0;
    FAULT5 = 0;
    FAULT6 = 0;


    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    initEPWM1();
    initEPWM2();
    initEPWM3();
    initEPWM4();
    initEPWM5();
    initEPWM6();

    //Setup Resolver Excitation PWM
    initResolverPWM();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable ePWM interrupts
    //
    Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_EPWM10);

    //Setup Timer
    initTIMER1();
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);

    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCB1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //enable power to all gate drivers
    GD_ALL_PSEnable();
    //allow time for GD PSU to startup
    DEVICE_DELAY_US(200);
    //reset all gate drivers and set OC_EN high (desat enabled)
    GD_ALL_Reset();





    //
    // Set up ADCs, initializing the SOCs to be triggered by software
    //
    initADCs();
    initADCSOCs();

    //enable Resolver Excitation Power Supply, 10ms starup delay on power suppliy
    enablePos15V();


    initCAN();
    uint16_t txMsgData[8], rxMsgData[8];
    *(uint16_t *)rxMsgData = 0;

    CPUTimer_startTimer(CPUTIMER1_BASE); //starts timer for CAN feedback update freq

    while(1){
        NOP;
        GPIO_togglePin(68); //blink green LED for heartbeat

        DEVICE_DELAY_US(500000);
        //
        // Read CAN message object 2 and check for new data
        //
        if (CAN_readMessage(CANA_BASE, 2, rxMsgData))
        {
            GPIO_togglePin(66); //toggle yellow LED for CAN packet
            CANPacketDecode(rxMsgData);
            CANPacketEncode(txMsgData);
            CAN_sendMessage(CANA_BASE, 1, 8, txMsgData);
        }

        //check fault status
        //note fast response is done in Tripzone this is for UI status

        if(GD_Global_getFault()) //faults are are combined together, active low
        {

            GD_ALL_LogicDisable();
            LEN1 = 0;
            LEN2 = 0;
            LEN3 = 0;
            LEN4 = 0;
            LEN5 = 0;
            LEN6 = 0;
            FAULT1 = GD_U_getFault();
            FAULT2 = GD_V_getFault();
            FAULT3 = GD_W_getFault();
            FAULT4 = GD_X_getFault();
            FAULT5 = GD_Y_getFault();
            FAULT6 = GD_Z_getFault();
            GPIO_writePin(64, 1); //set red LED on
        }

        //send status update
        CANPacketEncode(txMsgData);
        CAN_sendMessage(CANA_BASE, 1, 8, txMsgData);

        //Read analogs that are not PWM synced
        //
        // Convert, wait for completion, and store results
        //
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER6);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER7);

        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER2);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER3);

        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER1);
        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER2);
        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER3);
        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER4);
        ADC_forceSOC(ADCD_BASE, ADC_SOC_NUMBER5);

 /*       //
        // Wait for ADCA to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        //
        // Wait for ADCB to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
*/        //
        // Wait for ADCC to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
        //
        // Wait for ADCD to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

        //
        // Store results
        //

        TemperatureMsgDataUVW[0] = (uint16_t)getAnalogTempU()>>8; //U-Temp
        TemperatureMsgDataUVW[1] = (uint16_t)getAnalogTempU();

        TemperatureMsgDataUVW[2] = (uint16_t)getAnalogTempV()>>8; //V-Temp
        TemperatureMsgDataUVW[3] = (uint16_t)getAnalogTempV();

        TemperatureMsgDataUVW[4] = (uint16_t)getAnalogTempW()>>8; //W-Temp
        TemperatureMsgDataUVW[5] = (uint16_t)getAnalogTempW();

        TemperatureMsgDataUVW[6] = (uint16_t)getCaseTemp()>>8; //CASE-Temp
        TemperatureMsgDataUVW[7] = (uint16_t)getCaseTemp();

/*
        CurrentMsgDataUVW[0] = (int16_t)getCurrentU()>>8; //U-Current
        CurrentMsgDataUVW[1] = (int16_t)getCurrentU();

        CurrentMsgDataUVW[2] = (int16_t)getCurrentV()>>8; //V-Current
        CurrentMsgDataUVW[3] = (int16_t)getCurrentV();

        CurrentMsgDataUVW[4] = (int16_t)getCurrentW()>>8; //W-Current
        CurrentMsgDataUVW[5] = (int16_t)getCurrentW();

        CurrentMsgDataUVW[6] = (int16_t)getCurrentX()>>8; //X-Current
        CurrentMsgDataUVW[7] = (int16_t)getCurrentX();


        VoltageMsgDataUVW[0] = (int16_t)getVoltageU()>>8; //Vsense-U
        VoltageMsgDataUVW[1] = (int16_t)getVoltageU();

        VoltageMsgDataUVW[2] = (int16_t)getVoltageV()>>8; //Vsense-V
        VoltageMsgDataUVW[3] = (int16_t)getVoltageV();

        VoltageMsgDataUVW[4] = (int16_t)getVoltageW()>>8; //Vsense-W
        VoltageMsgDataUVW[5] = (int16_t)getVoltageW();
*/
        VoltageMsgDataUVW[6] = (int16_t)getVoltageDC()>>8; //Vsense-DC
        VoltageMsgDataUVW[7] = (int16_t)getVoltageDC();


        CAN_sendMessage(CANA_BASE, 3, 8, TemperatureMsgDataUVW); //transmit temperature feedback
        //CAN_sendMessage(CANA_BASE, 4, 8, CurrentMsgDataUVW); //transmit current feedback
        //CAN_sendMessage(CANA_BASE, 5, 8, VoltageMsgDataUVW); //transmit voltage feedback


        //send the feedback for the 2nd inverter

        TemperatureMsgDataXYZ[0] = (uint16_t)getAnalogTempX()>>8; //X-Temp
        TemperatureMsgDataXYZ[1] = (uint16_t)getAnalogTempX();

        TemperatureMsgDataXYZ[2] = (uint16_t)getAnalogTempY()>>8; //Y-Temp
        TemperatureMsgDataXYZ[3] = (uint16_t)getAnalogTempY();

        TemperatureMsgDataXYZ[4] = (uint16_t)getAnalogTempZ()>>8; //Z-Temp
        TemperatureMsgDataXYZ[5] = (uint16_t)getAnalogTempZ();


        //TemperatureMsgDataXYZ[6] = (uint16_t)getCaseTemp()>>8; //CASE-Temp
        //TemperatureMsgDataXYZ[7] = (uint16_t)getCaseTemp();

/*
        CurrentMsgDataXYZ[0] = (int16_t)getCurrentX()>>8; //X-Current
        CurrentMsgDataXYZ[1] = (int16_t)getCurrentX();

        CurrentMsgDataXYZ[2] = (int16_t)getCurrentY()>>8; //Y-Current
        CurrentMsgDataXYZ[3] = (int16_t)getCurrentY();

        CurrentMsgDataXYZ[4] = (int16_t)getCurrentZ()>>8; //Z-Current
        CurrentMsgDataXYZ[5] = (int16_t)getCurrentZ();

        //CurrentMsgData[6] = (int16_t)getCurrentX()>>8; //X-Current
        //CurrentMsgData[7] = (int16_t)getCurrentX();


        VoltageMsgDataXYZ[0] = (int16_t)getVoltageX()>>8; //Vsense-X
        VoltageMsgDataXYZ[1] = (int16_t)getVoltageX();

        VoltageMsgDataXYZ[2] = (int16_t)getVoltageY()>>8; //Vsense-Y
        VoltageMsgDataXYZ[3] = (int16_t)getVoltageY();

        VoltageMsgDataXYZ[4] = (int16_t)getVoltageZ()>>8; //Vsense-Z
        VoltageMsgDataXYZ[5] = (int16_t)getVoltageZ();

        //VoltageMsgData[6] = (int16_t)getVoltageDC()>>8; //Vsense-DC
        //VoltageMsgData[7] = (int16_t)getVoltageDC();
*/
        CAN_sendMessage(CANA_BASE, 6, 8, TemperatureMsgDataXYZ); //transmit temperature feedback
        //CAN_sendMessage(CANA_BASE, 7, 8, CurrentMsgDataXYZ); //transmit current feedback
        //CAN_sendMessage(CANA_BASE, 8, 8, VoltageMsgDataXYZ); //transmit voltage feedback
    }
}

//
// epwm1ISR - ePWM 1 ISR
//
__interrupt void epwm1ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //
    updatePWM(&epwm1Info);
    updatePWM(&epwm2Info);
    updatePWM(&epwm3Info);

    updatePWM(&epwm4Info);
    updatePWM(&epwm5Info);
    updatePWM(&epwm6Info);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//
// epwm1TZISR - ePWM1 TZ ISR
//
__interrupt void epwm1TZISR(void)
{
    //FAULT1 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM1_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// epwm2TZISR - ePWM2 TZ ISR
//
__interrupt void epwm2TZISR(void)
{
    //FAULT2 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM2_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// epwm3TZISR - ePWM3 TZ ISR
//
__interrupt void epwm3TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}


//
// epwm4TZISR - ePWM4 TZ ISR
//
__interrupt void epwm4TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// epwm5TZISR - ePWM5 TZ ISR
//
__interrupt void epwm5TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}


//
// epwm6TZISR - ePWM6 TZ ISR
//
__interrupt void epwm6TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}


//
// cpuTimer1ISR - ISR for CpuTimer1 Counter
//
__interrupt void cpuTimer1ISR(void)
{
    //
    // The CPU acknowledges the interrupt.
    //
    cpuTimer1IntCount++;
    CAN_sendMessage(CANA_BASE, 4, 8, CurrentMsgDataUVW); //transmit current feedback
    CAN_sendMessage(CANA_BASE, 7, 8, CurrentMsgDataXYZ); //transmit current feedback
    CAN_sendMessage(CANA_BASE, 5, 8, VoltageMsgDataUVW); //transmit voltage feedback
    CAN_sendMessage(CANA_BASE, 8, 8, VoltageMsgDataXYZ); //transmit voltage feedback
}


//
// ADC ISR read currents and send over CAN
//
//TODO
__interrupt void adcAISR(void)
{

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    VoltageMsgDataUVW[0] = (int16_t)getVoltageU()>>8; //Vsense-U
    VoltageMsgDataUVW[1] = (int16_t)getVoltageU();

    VoltageMsgDataUVW[2] = (int16_t)getVoltageV()>>8; //Vsense-V
    VoltageMsgDataUVW[3] = (int16_t)getVoltageV();

    VoltageMsgDataUVW[4] = (int16_t)getVoltageW()>>8; //Vsense-W
    VoltageMsgDataUVW[5] = (int16_t)getVoltageW();

    VoltageMsgDataUVW[6] = (int16_t)getVoltageDC()>>8; //Vsense-DC
    VoltageMsgDataUVW[7] = (int16_t)getVoltageDC();

    VoltageMsgDataXYZ[0] = (int16_t)getVoltageX()>>8; //Vsense-X
    VoltageMsgDataXYZ[1] = (int16_t)getVoltageX();

    VoltageMsgDataXYZ[2] = (int16_t)getVoltageY()>>8; //Vsense-Y
    VoltageMsgDataXYZ[3] = (int16_t)getVoltageY();

    VoltageMsgDataXYZ[4] = (int16_t)getVoltageZ()>>8; //Vsense-Z
    VoltageMsgDataXYZ[5] = (int16_t)getVoltageZ();

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

//
// ADC ISR read currents and send over CAN
//
//TODO
__interrupt void adcBISR(void)
{

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    CurrentMsgDataUVW[0] = (int16_t)getCurrentU()>>8; //U-Current
    CurrentMsgDataUVW[1] = (int16_t)getCurrentU();

    CurrentMsgDataUVW[2] = (int16_t)getCurrentV()>>8; //V-Current
    CurrentMsgDataUVW[3] = (int16_t)getCurrentV();

    CurrentMsgDataUVW[4] = (int16_t)getCurrentW()>>8; //W-Current
    CurrentMsgDataUVW[5] = (int16_t)getCurrentW();

    CurrentMsgDataUVW[6] = (int16_t)getCurrentX()>>8; //X-Current
    CurrentMsgDataUVW[7] = (int16_t)getCurrentX();

    CurrentMsgDataXYZ[0] = (int16_t)getCurrentX()>>8; //X-Current
    CurrentMsgDataXYZ[1] = (int16_t)getCurrentX();

    CurrentMsgDataXYZ[2] = (int16_t)getCurrentY()>>8; //Y-Current
    CurrentMsgDataXYZ[3] = (int16_t)getCurrentY();

    CurrentMsgDataXYZ[4] = (int16_t)getCurrentZ()>>8; //Z-Current
    CurrentMsgDataXYZ[5] = (int16_t)getCurrentZ();
    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

//
// initTIMER1 - Configure Timer1 for CAN feedback update frequency
//
void initTIMER1()
{
    //
    // Initialize timer period to update frequency
    //
    //  Timer count = 200Mhz / timer frequency, 200000 = 1Khz
    //
    CPUTimer_setPeriod(CPUTIMER1_BASE, 200000000/TIMER1FREQ); //1khz timer

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);

    //
    // Reset interrupt counter
    //
    cpuTimer1IntCount = 0;

}


//
// initEPWM1 - Configure ePWM1
//
void initEPWM1()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0); //if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO); //module EPWM1 is master

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE); //master has no phase shift
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);


    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM1_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM1_BASE, EPWM_TZ_INTERRUPT_OST);

    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U); //Interrupt every switching cycle for (required for sine math)


//TODO

    // Setting up link from EPWM to ADC
    //Single sampling per period
    // Select SOC from counter at ctr = 0
    EPWM_setADCTriggerSource(EPWM1_BASE,
                             EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);

    //(SAMPLING_METHOD == DOUBLE_SAMPLING)
    // Select SOC from counter at ctr = 0 or ctr = prd
    //EPWM_setADCTriggerSource(obj->pwmHandle[0], EPWM_SOC_A,
    //                         EPWM_SOC_TBCTR_ZERO_OR_PERIOD);

    // Generate pulse on 1st event
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 15);

    // Enable SOC on A group
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm1Info.epwmModule = EPWM1_BASE;
    epwm1Info.epwmPeriod = EPwm_TBPRD;
    epwm1Info.epwmPwmPhase = 0;
    epwm1Info.epwmDeadTime = DEAD_TIME;
    epwm1Info.epwmRadian = 0;

}

//
// initEPWM2 - Configure ePWM2
//
void initEPWM2()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPwm_TBPRD);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setCountModeAfterSync(EPWM2_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC); //if in up-down mode start counting up when sync occurs
    EPWM_setPhaseShift(EPWM2_BASE, 2);//add 2 to compensate for sync delay between master and slave
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM2_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM2_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM2_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM2_BASE);
   // EPWM_setInterruptEventCount(EPWM2_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    epwm2Info.epwmModule = EPWM2_BASE;
    epwm2Info.epwmPeriod = EPwm_TBPRD;
    epwm2Info.epwmPwmPhase = 2;
    epwm2Info.epwmDeadTime = DEAD_TIME;
    epwm2Info.epwmRadian = 2*PI/3;
}

//
// initEPWM3 - Configure ePWM3
//
void initEPWM3()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPwm_TBPRD);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setCountModeAfterSync(EPWM3_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);//if in up-down mode start counting up when sync occurs
    EPWM_setPhaseShift(EPWM3_BASE, 2);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3 //add 2 to compensate for sync delay between master and slave
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM3_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setClockPrescaler(EPWM3_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM3_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);


    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM3_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM3_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM3_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM3_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM3_BASE);
   // EPWM_setInterruptEventCount(EPWM3_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm3Info.epwmModule = EPWM3_BASE;
    epwm3Info.epwmPeriod = EPwm_TBPRD;
    epwm3Info.epwmPwmPhase = 2;
    epwm3Info.epwmDeadTime = DEAD_TIME;
    epwm3Info.epwmRadian = 4*PI/3;
}


//
// initEPWM4 - Configure ePWM4
//
void initEPWM4()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM4_BASE, EPwm_TBPRD);
    EPWM_enablePhaseShiftLoad(EPWM4_BASE);
    EPWM_setCountModeAfterSync(EPWM4_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);//if in up-down mode start counting up when sync occurs
    EPWM_setPhaseShift(EPWM4_BASE, 2);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3 //add 2 to compensate for sync delay between master and slave
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set EPWM4 sync input to be sync-out from EPWM1 cascading PWM456 from PWM123
    //
  //  SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT); //EPWM4SYNCIN MUX set to EPWM1
    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM4_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM4_BASE);
    EPWM_setClockPrescaler(EPWM4_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM4_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM4_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM4_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM4_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);


    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM4_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM4_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM4_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM4_BASE);
   // EPWM_setInterruptEventCount(EPWM4_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm4Info.epwmModule = EPWM4_BASE;
    epwm4Info.epwmPeriod = EPwm_TBPRD;
    epwm4Info.epwmPwmPhase = 2;
    epwm4Info.epwmDeadTime = DEAD_TIME;
    epwm4Info.epwmRadian = 0;
}

//
// initEPWM5 - Configure ePWM5
//
void initEPWM5()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM5_BASE, EPwm_TBPRD);
    EPWM_enablePhaseShiftLoad(EPWM5_BASE);
    EPWM_setCountModeAfterSync(EPWM5_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);//if in up-down mode start counting up when sync occurs
    EPWM_setPhaseShift(EPWM5_BASE, 2);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3 //add 2 to compensate for sync delay between master and slave
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM5_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM5_BASE);
    EPWM_setClockPrescaler(EPWM5_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM5_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM5_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM5_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM5_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);


    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM5_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM5_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM5_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM5_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM5_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM5_BASE);
   // EPWM_setInterruptEventCount(EPWM5_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm5Info.epwmModule = EPWM5_BASE;
    epwm5Info.epwmPeriod = EPwm_TBPRD;
    epwm5Info.epwmPwmPhase = 2;
    epwm5Info.epwmDeadTime = DEAD_TIME;
    epwm5Info.epwmRadian = 2*PI/3;
}

//
// initEPWM6 - Configure ePWM6
//
void initEPWM6()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPwm_TBPRD);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setCountModeAfterSync(EPWM6_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);//if in up-down mode start counting up when sync occurs
    EPWM_setPhaseShift(EPWM6_BASE, 2);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3 //add 2 to compensate for sync delay between master and slave
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM6_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM6_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM6_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM6_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM6_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM6_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);


    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM6_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM6_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM6_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM6_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM6_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM6_BASE);
   // EPWM_setInterruptEventCount(EPWM6_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm6Info.epwmModule = EPWM6_BASE;
    epwm6Info.epwmPeriod = EPwm_TBPRD;
    epwm6Info.epwmPwmPhase = 2;
    epwm6Info.epwmDeadTime = DEAD_TIME;
    epwm6Info.epwmRadian = 4*PI/3;
}

//
// updateCompare - Function to update the frequency
//
void updatePWM(epwmInformation *epwmInfo)
{
    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(epwmInfo->epwmModule, EPwm_TBPRD);
    //EPWM_setPhaseShift(epwmInfo->epwmModule, EPwm_TBPRD*epwmInfo->epwmPwmPhase);
    epwmInfo->epwmDeadTime = DEAD_TIME;
    EPWM_setRisingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);
    EPWM_setFallingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);

    // Sine wave math
    Sine = ((((MF*sin(epwmInfo->epwmRadian))+1.0))/2);
    epwmInfo->epwmRadian += Ts;

    if(epwmInfo->epwmRadian > 2*PI)
        epwmInfo->epwmRadian -= (2*PI);

    EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                               EPWM_COUNTER_COMPARE_A,
                                               Sine*EPwm_TBPRD);

}


void CANPacketEncode( uint16_t *PacketData)
{
    // Concatenate data values into 8 byte message (make sure all are Uint32 data types)
    FS = SWITCHING_FREQ/1000;
    ID = (MF * 1000);
    TD = DEAD_TIME*10;
    FF = FUND_FREQ;

    PacketData[0] = (uint16_t)FS;
    PacketData[1] = (uint16_t)(ID>>2);
    PacketData[2] = (uint16_t)(ID<<6 | TD >>6);
    PacketData[3] = (uint16_t)(TD<<2 | FF>>8);
    PacketData[4] = (uint16_t)(FF);
    PacketData[5] = (uint16_t)(PSEN1 << 6 | PSEN2 << 5 | PSEN3 << 4 | LEN1 << 2 | LEN2 << 1  | LEN3 );
    PacketData[6] = (uint16_t)(RESET << 7 | FAULT1 <<6 | FAULT2 <<5 | FAULT3 << 4 | PSEN4 << 2 | PSEN5 << 1 | PSEN6 );
    PacketData[7] = (uint16_t)(LEN4 << 6 | LEN5 << 5 | LEN6 << 4 | FAULT4 << 2 | FAULT5 << 1 | FAULT6 );

}

void CANPacketDecode(uint16_t *PacketData)
{
    FS = (PacketData[0])&0x000000FF;
    ID = ((PacketData[1]<<2) | (PacketData[2]>>6))&0x000003FF;
    TD = (((PacketData[2]&0x3F)<<6) | (PacketData[3]>>2))&0x00000FFF;
    FF = (((PacketData[3]&0x03)<<8) | PacketData[4])&0x000003FF;
    PSEN1 = (PacketData[5] & 0x40)>>6;
    PSEN2 = (PacketData[5] & 0x20)>>5;
    PSEN3 = (PacketData[5] & 0x10)>>4;
    LEN1 = (PacketData[5] & 0x04)>>2;
    LEN2 = (PacketData[5] & 0x02)>>1;
    LEN3 = PacketData[5] & 0x01;
    FAULT1 = (PacketData[6] & 0x40)>>6;
    FAULT2 = (PacketData[6] & 0x20)>>5;
    FAULT3 = (PacketData[6] & 0x10)>>4;
    RESET = (PacketData[6] & 0x80)>>7;

    PSEN4 = (PacketData[6] & 0x04)>>2;
    PSEN5 = (PacketData[6] & 0x02)>>1;
    PSEN6 = PacketData[6] & 0x01;
    LEN4 = (PacketData[7] & 0x40)>>6;;
    LEN5 = (PacketData[7] & 0x20)>>5;;
    LEN6 = (PacketData[7] & 0x10)>>4;;
    FAULT4 = (PacketData[7] & 0x04)>>2;
    FAULT5 = (PacketData[7] & 0x02)>>2;
    FAULT6 = PacketData[7] & 0x01;

    SWITCHING_FREQ = (FS) * 1000;
    MF = (ID) / 1000.0;
    DEAD_TIME = (TD) / 10;
    FUND_FREQ = FF;
    if(FUND_FREQ >500)
    {
        FUND_FREQ = 500;
    }

    if (PSEN1 == 1)
    {
        GD_U_PSEnable();
    }
    if (PSEN1 == 0)
    {
        GD_U_PSDisable();
    }

    if (PSEN2 == 1)
    {
        GD_V_PSEnable();
    }
    if (PSEN2 == 0)
    {
        GD_V_PSDisable();
    }

    if (PSEN3 == 1)
    {
        GD_W_PSEnable();
    }
    if (PSEN3 == 0)
    {
        GD_W_PSDisable();
    }

    if (LEN1 == 1)
    {
        GD_U_LogicEnable();
    }
    if (LEN1 == 0)
    {
        GD_U_LogicDisable();
    }

    if (LEN2 == 1)
    {
        GD_V_LogicEnable();
    }
    if (LEN2 == 0)
    {
        GD_V_LogicDisable();
    }
    if (LEN3 == 1)
    {
        GD_W_LogicEnable();
    }
    if (LEN3 == 0)
    {
        GD_W_LogicDisable();
    }

    //Control Bits for XYZ
    if (PSEN4 == 1)
    {
        GD_X_PSEnable();
    }
    if (PSEN4 == 0)
    {
        GD_X_PSDisable();
    }

    if (PSEN5 == 1)
    {
        GD_Y_PSEnable();
    }
    if (PSEN5 == 0)
    {
        GD_Y_PSDisable();
    }

    if (PSEN6 == 1)
    {
        GD_Z_PSEnable();
    }
    if (PSEN6 == 0)
    {
        GD_Z_PSDisable();
    }

    if (LEN4 == 1)
    {
        GD_X_LogicEnable();
    }
    if (LEN4 == 0)
    {
        GD_X_LogicDisable();
    }

    if (LEN5 == 1)
    {
        GD_Y_LogicEnable();
    }
    if (LEN5 == 0)
    {
        GD_Y_LogicDisable();
    }
    if (LEN6 == 1)
    {
        GD_Z_LogicEnable();
    }
    if (LEN6 == 0)
    {
        GD_Z_LogicDisable();
    }


    if (RESET == 1)
    {
        // Reset gate drivers
        GD_ALL_Reset();
        GPIO_writePin(64, 0); //set red LED off

        // Reset Trip-Zone and Interrupt flag
        //TODO
        //
        // To re-enable the OST Interrupt, uncomment the below code:
        //
         EPWM_clearTripZoneFlag(EPWM1_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM2_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM3_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM4_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM5_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM6_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

        FAULT1 = 0;
        FAULT2 = 0;
        FAULT3 = 0;
        FAULT4 = 0;
        FAULT5 = 0;
        FAULT6 = 0;
        RESET = 0;
    }
}



//
// TestLEDS - cycle led pins to test functionality
//
void TestLEDS(){
    GPIO_togglePin(34);
    GPIO_togglePin(64);
    GPIO_togglePin(66);
    GPIO_togglePin(68);

    DEVICE_DELAY_US(200000);
}//TestLEDS

//
// initGPIO - this initializes the GPIO for controller not in external files.
//
void initGPIO(void)
{
    //
    // LEDS
    //
    // Enable GPIO outputs on GPIO 34,64,66,68, set it LOW
    /*
    34  ControlCard-LED-RED
    64  LED-R
    66  LED-Y
    68  LED-G
    */


    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(34, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(64, GPIO_PIN_TYPE_STD);
    GPIO_writePin(64, 0);
    GPIO_setPinConfig(GPIO_64_GPIO64);
    GPIO_setDirectionMode(64, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(66, GPIO_PIN_TYPE_STD);
    GPIO_writePin(66, 0);
    GPIO_setPinConfig(GPIO_66_GPIO66);
    GPIO_setDirectionMode(66, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(68, GPIO_PIN_TYPE_STD);
    GPIO_writePin(68, 0);
    GPIO_setPinConfig(GPIO_68_GPIO68);
    GPIO_setDirectionMode(68, GPIO_DIR_MODE_OUT);


    //
    // Enable EQEP1 on GPIO's 20,21,23
    /*
    20  QEA_A
    21  QEA_B
    23  QEA_I
    */
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO20
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO21
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO23
    GPIO_setQualificationMode(20, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(21, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(23, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_20_EQEP1A);              // GPIO20 = EQEP1A
    GPIO_setPinConfig(GPIO_21_EQEP1B);              // GPIO21 = EQEP1B
    GPIO_setPinConfig(GPIO_23_EQEP1I);              // GPIO99 = EQEP1I

    //
    // Enable EQEP2 on GPIO's 54,55,57
    /*
    54  QEB_A
    55  QEB_B
    57  QEB_I
    */
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO54
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO55
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO57
    GPIO_setQualificationMode(54, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(55, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(57, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_54_EQEP2A);              // GPIO54 = EQEP1A
    GPIO_setPinConfig(GPIO_55_EQEP2B);              // GPIO55 = EQEP1B
    GPIO_setPinConfig(GPIO_57_EQEP2I);              // GPIO57 = EQEP1I


}
