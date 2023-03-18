/*
 * pca9685.h
 *
 *  Created on: Mar 12, 2023
 *      Author: nazar
 */

#ifndef INC_pca9685_H_
#define INC_pca9685_H_

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALER 0xFE

#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD

#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define PCA9685_MODE1_SLEEP 0b00010000
#define PCA9685_MODE1_RESTART 0b10000000
#define PCA9685_MODE1_AI 0b00100000
#define PCA9685_MODE1_ALLCALL 0b00000001

typedef struct
{
	I2C_HandleTypeDef *i2c_handle;
	uint8_t device_address;
} pca9685_handle;

bool PCA9685_Init(pca9685_handle *handle);
bool PCA9685_setDutyCycle(pca9685_handle *handle, uint16_t duty_cycle, uint32_t channel);
bool PCA9685_setFrequency(pca9685_handle *handle, float frequency);
bool PCA9685_sleep(pca9685_handle *handle);
bool PCA9685_wakeUp(pca9685_handle *handle);


#endif /* INC_PCA9685_H_ */
