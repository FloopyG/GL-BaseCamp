/*
 * pca9685.c
 *
 *  Created on: Mar 12, 2023
 *      Author: nazar
 */
#include "pca9685.h"

// Write 1 byte of data to register
static bool PCA9685_writeRegister(pca9685_handle *handle, uint8_t address, uint8_t value)
{
  uint8_t data[] = {address, value};
  return HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, HAL_MAX_DELAY) == HAL_OK;

}

// Read 1 byte of data from register
static bool PCA9685_readRegister(pca9685_handle *handle, uint8_t address, uint8_t *dest)
{
  if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, &address, 1, HAL_MAX_DELAY) != HAL_OK)
  {
	  return false;
  }

  return HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address, dest, 1, HAL_MAX_DELAY) == HAL_OK;
}

/*
 * Init PCA9685
 * Mode 1 AI bit set
 * Mode 1 ALLCALL bit set
 * Duty cycle for all channels 50%
 * Frequency 500HZ
 *
 */
bool PCA9685_Init(pca9685_handle *handle)
{
  bool success = true;

  uint8_t mode1_reg = 0;
  mode1_reg |= PCA9685_MODE1_AI;
  mode1_reg |= PCA9685_MODE1_ALLCALL;
  success &= PCA9685_writeRegister(handle, PCA9685_MODE1, mode1_reg);

  success &= PCA9685_setDutyCycle(handle, 50, 16);
  success &= PCA9685_setFrequency(handle, 500.0f);

  return success;
}


/*
 * Set duty cycle for specific channel or for all channels
 * Specific channels values can be between 0 and 15
 * To select all channels value msut be set to 16
 */
bool PCA9685_setDutyCycle(pca9685_handle *handle, uint16_t duty_cycle, uint32_t channel)
{
  if(duty_cycle < 0 || duty_cycle > 100 || channel < 0 || channel > 16)
	  return false;

  bool success = true;
  // Calculate the LED_ON and LED_OFF values based on the duty cycle
  uint16_t on_value = 0;
  uint16_t off_value = (uint16_t)(duty_cycle / 100.0 * 4095.0);

  uint8_t on_l = (uint8_t)(on_value & 0xFF);
  uint8_t on_h = (uint8_t)((on_value >> 8) & 0x0F);
  uint8_t off_l = (uint8_t)(off_value & 0xFF);
  uint8_t off_h = (uint8_t)((off_value >> 8) & 0x0F);


  if (channel == 16)
  {
	  // Set the LED_ON and LED_OFF registers for all channels to the calculated values
	  success &= PCA9685_writeRegister(handle, ALL_LED_ON_L, on_l);
	  success &= PCA9685_writeRegister(handle, ALL_LED_ON_H, on_h);
	  success &= PCA9685_writeRegister(handle, ALL_LED_OFF_L, off_l);
	  success &= PCA9685_writeRegister(handle, ALL_LED_OFF_H, off_h);
  }
  else
  {
	  // Set the LED_ON and LED_OFF registers for chosen channel to the calculated values
	  success &= PCA9685_writeRegister(handle, LED0_ON_L + (4 * channel), on_l);
	  success &= PCA9685_writeRegister(handle, LED0_ON_H + (4 * channel), on_h);
	  success &= PCA9685_writeRegister(handle, LED0_OFF_L + (4 * channel), off_l);
	  success &= PCA9685_writeRegister(handle, LED0_OFF_H + (4 * channel), off_h);
  }

  return success;
}

/*
 * Set frequency for PCA9685 by changing prescaler value
 * Frequency can be set between 24HZ and 1526HZ
 * Notice that to set frequency PCA9685 will be set into sleep mode
 */
bool PCA9685_setFrequency(pca9685_handle *handle, float frequency)
{
  if (frequency < 24.0f || frequency > 1526.0f)
	  return false;

  bool success = true;

  // Calculate the prescaler value based on the desired frequency
  uint8_t prescaler = (uint8_t)(25000000.0 / (4096 * frequency) - 1);

  // Set the PCA9685 to sleep mode before setting the prescaler
  success &= PCA9685_sleep(handle);

  // Set the prescaler value
  success &= PCA9685_writeRegister(handle, PCA9685_PRESCALER, prescaler);

  // Wake up the PCA9685
  success &= PCA9685_wakeUp(handle);

  return success;
}

// Enables Sleep mode
bool PCA9685_sleep(pca9685_handle *handle)
{
  bool success = true;

  uint8_t mode1_reg = 0;
  success &= PCA9685_readRegister(handle, PCA9685_MODE1, &mode1_reg);

  mode1_reg &= ~PCA9685_MODE1_RESTART;
  mode1_reg |= PCA9685_MODE1_SLEEP;

  success &= PCA9685_writeRegister(handle, PCA9685_MODE1, mode1_reg);

  return success;
}

// Disables Sleep mode
bool PCA9685_wakeUp(pca9685_handle *handle)
{
  bool success = true;

  uint8_t mode1_reg;
  success &= PCA9685_readRegister(handle, PCA9685_MODE1, &mode1_reg);

  mode1_reg &= ~PCA9685_MODE1_RESTART;
  mode1_reg &= ~PCA9685_MODE1_SLEEP;
  success &= PCA9685_writeRegister(handle, PCA9685_MODE1, mode1_reg);

  HAL_Delay(1);

  mode1_reg |= PCA9685_MODE1_RESTART;
  success &= PCA9685_writeRegister(handle, PCA9685_MODE1, mode1_reg);

  return success;
}
