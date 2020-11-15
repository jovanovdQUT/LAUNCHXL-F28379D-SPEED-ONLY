/*
 * pwm.h
 *
 *  Created on: 11/02/2020
 *      Author: Dejan Jovanovic
 */

#ifndef PWM_H_
#define PWM_H_


// If LAUNCHXL-F28379D is used then two full bridge topology is required. In this
// case two PWM modules are reserved (PWM1 and PWM2)
#define CONFIG_AS_FULL_BRIDGE       (0)


// Configure PWM modules depending on underlying hardaware topology
void ConfigureEPWM(void);

#endif /* PWM_H_ */
