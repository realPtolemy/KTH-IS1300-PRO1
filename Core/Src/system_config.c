/*
 * system_config.c
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 *
 */
#include "pro1_funct.h"
#include "spi.h"
#include "gpio.h"

/* Instantiate global variables */

// Required program delays, in ms.
static uint16_t greenDelay = 100;
static uint16_t orangeDelay = 100;
static uint16_t redDelayMax = 100;
static uint16_t pedestrianDelay = 100;
static uint16_t toggleFreq = 100;

/*
 * Shift register bits array, each element represents the
 * bits stored in each respective shift register.
 *
 * Used to conveniently store and push bits to the serially
 * connected shift registers
 *
 * REG[3] = { U1, U2, U3 }
 *
 */
uint8_t REG[3] = {0x00, 0x00, 0x00};

// Set all LEDs to their initial stage
void system_init(){

}

