/*
 * pro1_test.h
 *
 *  Created on: Dec 4, 2023
 *      Author: Love MItteregger
 */
#include "stdint.h"

#ifndef INC_SRC_TEST_LED_H_
#define INC_SRC_TEST_LED_H_

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
// LED functions - test_led.c
void trafficLED_Test_NS(void);
void trafficLED_Test_EW(void);
void pedestrianLED_Test_N(void);
void pedestrianLED_Test_W(void);

// SWITCH and BUTTONS functions - test_switches.c
void trafficSwitch_Test_NS(void);
void trafficSwitch_Test_EW(void);
void pedestrianSwitch_Test_N(void);
void pedestrianSwitch_Test_W(void);

// Initialization functions - test_init.c
void init_Test(void);

// Enhanced shift register bit staging and latching
void setReg_Test(void);
void stageReg_Test(void);
void latchReg_Test(void);
void activateLED_Test(void);
void activateMultiLED_Test(void);
void trafficRed_NS_Test(void);

// Set traffic lights
void traffic_NS_Test(uint8_t status);
void traffic_EW_Test(uint8_t status);
void pedestrian_N_Test(uint8_t status);
void pedestrian_W_Test(uint8_t status);
void pedestrianWarning_N_Test(void);
void pedestrianWarning_W_Test(void);
void pedestrianPending_N_Test(void);
void pedestrianPending_W_Test(void);

// Activate and dissable traffic or pedestrian crossings
void activateTraffic_NS_Test(void);
void disableTraffic_NS_Test(void);
void activateTraffic_EW_Test(void);
void disableTraffic_EW_Test(void);
void activatePedestrian_N_Test(void);
void disablePedestrian_N_Test(void);
void activatePedestrian_W_Test(void);
void disablePedestrian_W_Test(void);
void staticTraffic_NS_Test(void);
void staticTraffic_EW_Test(void);

// Read traffic from switches
uint8_t checkTraffic_Test(void);

#endif /* INC_SRC_TEST_LED_H_ */
