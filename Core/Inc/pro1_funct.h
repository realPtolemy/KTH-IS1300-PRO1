/*
 * pro1_funct.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "stdint.h"

#ifndef INC_IS1300_H_
#define INC_IS1300_H_

// System Initialization
void system_init(void);

// LED lights
void stageReg(void);
void latchReg(void);
void setReg(void);
void traffic_NS(uint8_t status);
void traffic_EW(uint8_t status);
void pedestrian_N(uint8_t status);
void pedestrian_W(uint8_t status);
void pedestrianWarning_N(void);
void pedestrianWarning_W(void);
void pedestrianPending_N(void);
void pedestrianPending_W(void);

// Traffic logic
//void checkTraffic(void);
void activateTraffic_NS(void);
void disableTraffic_NS(void);
void activateTraffic_EW(void);
void disableTraffic_EW(void);
void staticTraffic_NS(void);
void staticTraffic_EW(void);


// Tasks

#endif /* INC_IS1300_H_ */
