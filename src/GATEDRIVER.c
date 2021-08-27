/*
 * GATEDRIVER.c
 *
 *  Created on: Sep 4, 2020
 *      Author: mfeurtado
 *
 *      expanded from single inverter to support 6 gate drivers
 */
#include "GATEDRIVER.h"
#include "driverlib.h"
#include "device.h"

void initGateDriverGPIO()
{
    //
    // Enable PWM1-3 on GPIO0-GPIO5
    /*
    0   UHS-PWM
    1   ULS-PWM
    2   VHS-PWM
    3   VLS-PWM
    4   WHS-PWM
    5   WLS-PWM
    */
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO0
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO1
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO2
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO3
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO4
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO5
    GPIO_setPinConfig(GPIO_0_EPWM1A);               // GPIO0 = PWM1A
    GPIO_setPinConfig(GPIO_1_EPWM1B);               // GPIO1 = PWM1B
    GPIO_setPinConfig(GPIO_2_EPWM2A);               // GPIO2 = PWM2A
    GPIO_setPinConfig(GPIO_3_EPWM2B);               // GPIO3 = PWM2B
    GPIO_setPinConfig(GPIO_4_EPWM3A);               // GPIO4 = PWM3A
    GPIO_setPinConfig(GPIO_5_EPWM3B);               // GPIO5 = PWM3B

    //
    // Enable PWM4-6 on GPIO6-GPI11
    /*
    6   XHS-PWM
    7   XLS-PWM
    8   YHS-PWM
    9   YLS-PWM
    10  ZHS-PWM
    11  ZLS-PWM
    */
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO6
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO7
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO8
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO9
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO10
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);     // Enable push-pull on GPIO11
    GPIO_setPinConfig(GPIO_6_EPWM4A);               // GPIO6 = PWM4A
    GPIO_setPinConfig(GPIO_7_EPWM4B);               // GPIO7 = PWM4B
    GPIO_setPinConfig(GPIO_8_EPWM5A);               // GPIO8 = PWM5A
    GPIO_setPinConfig(GPIO_9_EPWM5B);               // GPIO9 = PWM5B
    GPIO_setPinConfig(GPIO_10_EPWM6A);               // GPIO10 = PWM6A
    GPIO_setPinConfig(GPIO_11_EPWM6B);               // GPIO11 = PWM6B




    // FAULTS
    //
    // Enable a GPIO input on GPIO35-38,60,61,65,67,69

    //36   U-FAULT
    //38   V-FAULT
    //61   W-FAULT
    //35   X-FAULT
    //37   Y-FAULT
    //60   Z-FAULT
    //65   GLOBAL-FAULT-UVW
    //67   GLOBAL-FAULT-XYZ
    //69   GLOBAL-FAULT *TripZone

    GPIO_setPadConfig(36, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_36_GPIO36);
    GPIO_setDirectionMode(36, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(36, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(38, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_38_GPIO38);
    GPIO_setDirectionMode(38, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(38, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_61_GPIO61);
    GPIO_setDirectionMode(61, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(61, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(35, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(37, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_60_GPIO60);
    GPIO_setDirectionMode(60, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_65_GPIO65);
    GPIO_setDirectionMode(65, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(65, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(67, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_67_GPIO67);
    GPIO_setDirectionMode(67, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(67, GPIO_QUAL_ASYNC);

    GPIO_setPadConfig(69, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_69_GPIO69);
    GPIO_setDirectionMode(69, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(69, GPIO_QUAL_ASYNC);
    XBAR_setInputPin(XBAR_INPUT1, 69);

//TODO
    // RTD TEMPs
    // Digital temperature measurement either PWM or FM using eCAP
    // Enable a GPIO input on GPIO 94,95,97

    //94   A-RTD -> eCAP1
    //95   C-RTD -> eCAP3
    //97   B-RTD -> eCAP2
/*
    XBAR_setInputPin(XBAR_INPUT7, 94); //config XBAR to route eCAP1 input from GPIO 94
    GPIO_setPinConfig(GPIO_94_GPIO94);
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(94, GPIO_QUAL_ASYNC);

    XBAR_setInputPin(XBAR_INPUT9, 95); //config XBAR to route eCAP3 input from GPIO 95
    GPIO_setPinConfig(GPIO_95_GPIO95);
    GPIO_setDirectionMode(95, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(95, GPIO_QUAL_ASYNC);

    XBAR_setInputPin(XBAR_INPUT8, 97); //config XBAR to route eCAP2 input from GPIO 97
    GPIO_setPinConfig(GPIO_97_GPIO97);
    GPIO_setDirectionMode(97, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(97, GPIO_QUAL_ASYNC);
*/
    // GD-Control
    //
    // Enable GPIO outputs on GPIO74-94, set it LOW

    /*
     * 75   U-OCEN-OUT
     * 77   U-LEN-OUT
     * 79   U-PSDIS-OUT
     * 83   V-OCEN-OUT
     * 85   V-LEN-OUT
     * 87   V-PSDIS-OUT
     * 91   W-OCEN-OUT
     * 93   W-LEN-OUT
     * 133  W-PSDIS-OUT
     *
     * 74   X-OCEN-OUT
     * 76   X-LEN-OUT
     * 78   X-PSDIS-OUT
     * 82   Y-OCEN-OUT
     * 84   Y-LEN-OUT
     * 86   Y-PSDIS-OUT
     * 90   Z-OCEN-OUT
     * 92   Z-LEN-OUT
     * 94  Z-PSDIS-OUT
     */


    GPIO_setPadConfig(75, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(75, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_75_GPIO75);              // GPIO = GPIO
    GPIO_setDirectionMode(75, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(77, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(77, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_77_GPIO77);              // GPIO = GPIO
    GPIO_setDirectionMode(77, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(79, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(79, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_79_GPIO79);              // GPIO = GPIO
    GPIO_setDirectionMode(79, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(83, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(83, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_83_GPIO83);              // GPIO = GPIO
    GPIO_setDirectionMode(83, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(85, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(85, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_85_GPIO85);              // GPIO = GPIO
    GPIO_setDirectionMode(85, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(87, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(87, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_87_GPIO87);              // GPIO = GPIO
    GPIO_setDirectionMode(87, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(91, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(91, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_91_GPIO91);              // GPIO = GPIO
    GPIO_setDirectionMode(93, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(133, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(133, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_133_GPIO133);              // GPIO = GPIO
    GPIO_setDirectionMode(133, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(74, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(74, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_74_GPIO74);              // GPIO = GPIO
    GPIO_setDirectionMode(74, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(76, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(76, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_76_GPIO76);              // GPIO = GPIO
    GPIO_setDirectionMode(76, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(78, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(78, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_78_GPIO78);              // GPIO = GPIO
    GPIO_setDirectionMode(78, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(82, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(82, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_82_GPIO82);              // GPIO = GPIO
    GPIO_setDirectionMode(82, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(84, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(84, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_84_GPIO84);              // GPIO = GPIO
    GPIO_setDirectionMode(84, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(86, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(86, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_86_GPIO86);              // GPIO = GPIO
    GPIO_setDirectionMode(86, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(90, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(90, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_90_GPIO90);              // GPIO = GPIO
    GPIO_setDirectionMode(90, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(92, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(92, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_92_GPIO92);              // GPIO = GPIO
    GPIO_setDirectionMode(92, GPIO_DIR_MODE_OUT);   // GPIO = output

    GPIO_setPadConfig(94, GPIO_PIN_TYPE_STD);    // Enable push-pull
    GPIO_writePin(94, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_94_GPIO94);              // GPIO = GPIO
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_OUT);   // GPIO = output

}

//phase U gate driver control
void GD_U_PSEnable(void)
{
    GPIO_writePin(79, 1); //U-PSDIS
}
void GD_U_PSDisable(void)
{
    GPIO_writePin(79, 0);
}
void GD_U_LogicEnable(void)
{
    GPIO_writePin(77, 1);
}
void GD_U_LogicDisable(void)
{
    GPIO_writePin(77, 0);
}
void GD_U_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(75, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(75, 0);
#endif

}
void GD_U_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(75, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(75, 1);
#endif
}


//phase V gate driver control
void GD_V_PSEnable(void)
{
    GPIO_writePin(87, 1); //V-PSDIS
}
void GD_V_PSDisable(void)
{
    GPIO_writePin(87, 0); //V-PSDIS
}
void GD_V_LogicEnable(void)
{
    GPIO_writePin(85, 1); //V-LEN
}
void GD_V_LogicDisable(void)
{
    GPIO_writePin(85, 0); //V-LEN
}
void GD_V_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(83, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(83, 0);
#endif
}
void GD_V_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(83, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(83, 1);
#endif
}

//phase W gate driver control
void GD_W_PSEnable(void)
{
    GPIO_writePin(133, 1); //W-PSDIS
}
void GD_W_PSDisable(void)
{
    GPIO_writePin(133, 0); //W-PSDIS
}
void GD_W_LogicEnable(void)
{
    GPIO_writePin(93, 1);
}
void GD_W_LogicDisable(void)
{
    GPIO_writePin(93, 0);
}
void GD_W_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(91, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(91, 0);
#endif
}
void GD_W_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(91, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(91, 1);
#endif
}


//phase X gate driver control
void GD_X_PSEnable(void)
{
    GPIO_writePin(78, 1); //X-PSDIS
}
void GD_X_PSDisable(void)
{
    GPIO_writePin(78, 0);
}
void GD_X_LogicEnable(void)
{
    GPIO_writePin(76, 1);
}
void GD_X_LogicDisable(void)
{
    GPIO_writePin(76, 0);
}
void GD_X_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(74, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(74, 0);
#endif

}
void GD_X_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(74, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(74, 1);
#endif
}

//phase Y gate driver control
void GD_Y_PSEnable(void)
{
    GPIO_writePin(86, 1); //Y-PSDIS
}
void GD_Y_PSDisable(void)
{
    GPIO_writePin(86, 0); //Y-PSDIS
}
void GD_Y_LogicEnable(void)
{
    GPIO_writePin(84, 1); //Y-LEN
}
void GD_Y_LogicDisable(void)
{
    GPIO_writePin(84, 0); //Y-LEN
}
void GD_Y_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(82, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(82, 0);
#endif
}
void GD_Y_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(82, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(82, 1);
#endif
}


//phase Z gate driver control
void GD_Z_PSEnable(void)
{
    GPIO_writePin(94, 1); //Z-PSDIS
}
void GD_Z_PSDisable(void)
{
    GPIO_writePin(94, 0); //Z-PSDIS
}
void GD_Z_LogicEnable(void)
{
    GPIO_writePin(92, 1);
}
void GD_Z_LogicDisable(void)
{
    GPIO_writePin(92, 0);
}
void GD_Z_OCEnable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(90, 1);
#endif
#ifdef XM3GDV2
    GPIO_writePin(90, 0);
#endif
}
void GD_Z_OCDisable(void)
{
#ifdef XM3GDV1
    GPIO_writePin(90, 0);
#endif
#ifdef XM3GDV2
    GPIO_writePin(90, 1);
#endif
}

//ALL gate driver control
void GD_ALL_PSEnable(void)
{
    GD_U_PSEnable();
    GD_V_PSEnable();
    GD_W_PSEnable();
    GD_X_PSEnable();
    GD_Y_PSEnable();
    GD_Z_PSEnable();
}
void GD_ALL_PSDisable(void)
{
    GD_U_PSDisable();
    GD_V_PSDisable();
    GD_W_PSDisable();
    GD_X_PSDisable();
    GD_Y_PSDisable();
    GD_Z_PSDisable();
}
void GD_ALL_LogicEnable(void)
{
    GD_U_LogicEnable();
    GD_V_LogicEnable();
    GD_W_LogicEnable();
    GD_X_LogicEnable();
    GD_Y_LogicEnable();
    GD_Z_LogicEnable();
}
void GD_ALL_LogicDisable(void)
{
    GD_U_LogicDisable();
    GD_V_LogicDisable();
    GD_W_LogicDisable();
    GD_X_LogicDisable();
    GD_Y_LogicDisable();
    GD_Z_LogicDisable();
}
void GD_ALL_OCEnable(void)
{
    GD_U_OCEnable();
    GD_V_OCEnable();
    GD_W_OCEnable();
    GD_X_OCEnable();
    GD_Y_OCEnable();
    GD_Z_OCEnable();
}
void GD_ALL_OCDisable(void)
{
    GD_U_OCDisable();
    GD_V_OCDisable();
    GD_W_OCDisable();
    GD_X_OCDisable();
    GD_Y_OCDisable();
    GD_Z_OCDisable();
}
void GD_ALL_Reset(void)
{
    GD_ALL_OCDisable();
    DEVICE_DELAY_US(100);
    GD_ALL_OCEnable();
}

//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_U_getFault(void)
{
    if(GPIO_readPin(36)) //U-FAULT is High then no fault
        return false;
    return true;
}

bool GD_V_getFault(void)
{
    if(GPIO_readPin(38)) //V-FAULT is High then no fault
        return false;
    return true;
}

bool GD_W_getFault(void)
{
    if(GPIO_readPin(61)) //W-FAULT is are High then no fault
        return false;
    return true;
}

//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_X_getFault(void)
{
    if(GPIO_readPin(35)) //X-FAULT is High then no fault
        return false;
    return true;
}

bool GD_Y_getFault(void)
{
    if(GPIO_readPin(37)) //Y-FAULT is High then no fault
        return false;
    return true;
}

bool GD_Z_getFault(void)
{
    if(GPIO_readPin(60)) //Z-FAULT is High then no fault
        return false;
    return true;
}


bool GD_Global_getFault(void)
{
    if(GPIO_readPin(69)) //GLOBAL-FAULT is High then no Fault
        return false;
    return true;
}
