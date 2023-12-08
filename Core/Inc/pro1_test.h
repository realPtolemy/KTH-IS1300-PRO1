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

#endif /* INC_SRC_TEST_LED_H_ */
