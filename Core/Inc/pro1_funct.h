/*
 * pro1_funct.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "stdint.h"

#ifndef INC_IS1300_H_
#define INC_IS1300_H_

/*** Defined enumerations ***/
// Traffic light color enumeration
enum LED {
	RED,
	ORANGE,
	GREEN,
	BLUE
};

// Street direction enumeration
enum Street {
	T_NORTHSOUTH,
	T_EASTWEST,
	P_NORTH = 1,
	P_WEST = 0,

};

/*** Defined Functions ***/
// System Initialization
void system_init(void);

// LED lights
void stageReg(void);
void latchReg(void);
void setReg(void);
void trafficLight(enum LED status, enum Street t_dir);
void pedestrianLight(enum LED status, enum Street p_dir);
void pedestrianWarning(enum Street p_dir);
void pedestrianPending(enum Street p_dir);

// Traffic logic
//void checkTraffic(void);
void activateTraffic(enum Street t_dir, enum Street p_dir);
void disableTraffic(enum Street t_dir, enum Street p_dir);
void staticTraffic(void);

// Read traffic from switches
uint8_t checkTraffic(void);


// Tasks

#endif /* INC_IS1300_H_ */
