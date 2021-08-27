/*
 * GATEDRIVER.h
 *
 *  Created on: Feb 22, 2019
 *      Author: mfeurtado
 */

#ifndef GATEDRIVER_H_
#define GATEDRIVER_H_
#include "device.h"

//changeds the polarity of the OCEN (DESAT) pin from active high to active low
//for the appropriate revision of the XM3 gate driver
#define XM3GDV2
//#define XM3GDV1

void initGateDriverGPIO(void);



//phase U gate driver control
void GD_U_PSEnable(void);
void GD_U_PSDisable(void);
void GD_U_LogicEnable(void);
void GD_U_LogicDisable(void);
void GD_U_OCEnable(void);
void GD_U_OCDisable(void);

//phase V gate driver control
void GD_V_PSEnable(void);
void GD_V_PSDisable(void);
void GD_V_LogicEnable(void);
void GD_V_LogicDisable(void);
void GD_V_OCEnable(void);
void GD_V_OCDisable(void);

//phase W gate driver control
void GD_W_PSEnable(void);
void GD_W_PSDisable(void);
void GD_W_LogicEnable(void);
void GD_W_LogicDisable(void);
void GD_W_OCEnable(void);
void GD_W_OCDisable(void);

//ALL gate driver control
void GD_ALL_PSEnable(void);
void GD_ALL_PSDisable(void);
void GD_ALL_LogicEnable(void);
void GD_ALL_LogicDisable(void);
void GD_ALL_OCEnable(void);
void GD_ALL_OCDisable(void);
void GD_ALL_Reset(void);

//phase X gate driver control
void GD_X_PSEnable(void);
void GD_X_PSDisable(void);
void GD_X_LogicEnable(void);
void GD_X_LogicDisable(void);
void GD_X_OCEnable(void);
void GD_X_OCDisable(void);

//phase Y gate driver control
void GD_Y_PSEnable(void);
void GD_Y_PSDisable(void);
void GD_Y_LogicEnable(void);
void GD_Y_LogicDisable(void);
void GD_Y_OCEnable(void);
void GD_Y_OCDisable(void);

//phase Z gate driver control
void GD_Z_PSEnable(void);
void GD_Z_PSDisable(void);
void GD_Z_LogicEnable(void);
void GD_Z_LogicDisable(void);
void GD_Z_OCEnable(void);
void GD_Z_OCDisable(void);


//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_U_getFault(void);
bool GD_V_getFault(void);
bool GD_W_getFault(void);
bool GD_X_getFault(void);
bool GD_Y_getFault(void);
bool GD_Z_getFault(void);
bool GD_Global_getFault(void);

#endif /* GATEDRIVER_H_ */
