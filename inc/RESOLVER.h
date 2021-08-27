/*
 * RESOLVER.h
 *
 *  Created on: Sep 16, 2020
 *      Author: mfeurtado
 */

#ifndef RESOLVER_H_
#define RESOLVER_H_

void initResolverGPIO(void);
void initResolverPWM(void);
void initResolverADC(void);
void enablePos15V(void); //power supply for resolver excitation circuit
void disablePos15V(void);

__interrupt void epwm10ISR(void);

#endif /* RESOLVER_H_ */
