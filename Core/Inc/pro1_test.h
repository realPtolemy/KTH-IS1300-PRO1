/*
 * pro1_test.h
 *
 *  Created on: Dec 4, 2023
 *      Author: Love MItteregger
 */

#ifndef INC_SRC_TEST_LED_H_
#define INC_SRC_TEST_LED_H_
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
void stageReg_Test(void);
void latchReg_Test(void);
void activateLED_Test(void);
void activateMultiLED_Test(void);

/* Change NORTH & SOUTH TRAFFIC lights */
void traffic_NS_Test(int status);
void trafficRed_NS_Test(void);
void trafficYellow_NS_Test(void);
void trafficGreen_NS_Test(void);

/* Set bits for EAST & WEST TRAFFIC lights */
void trafficRed_EW_Test(void);
void trafficYellow_EW_Test(void);
void trafficGreen_EW_Test(void);

/* Set bits for NORTH PEDESTRIAN lights */
void pedestrianRed_N_Test(void);
void pedestrianYellow_N_Test(void);
void pedestrianGreen_N_Test(void);

/* Set bits for West Pedestrian Lights */
void pedestrianRed_W_Test(void);
void pedestrianYellow_W_Test(void);
void pedestrianGreen_W_Test(void);


#endif /* INC_SRC_TEST_LED_H_ */
