/*
 * spi_flash.h
 *
 *  Created on: 18 мар. 2023 г.
 *      Author: nazar
 */

#ifndef INC_SPI_FLASH_H_
#define INC_SPI_FLASH_H_

#include "stm32f4xx_hal.h"

#define FLASH_SS_PIN GPIO_PIN_7

#define FLASH_WRITE_ENABLE 0x06
#define FLASH_WRITE_DISABLE 0x04

#define FLASH_READ 0x03
#define FLASH_WRITE 0x02

#define ENABLE_WRITE_STATUS_REGISTER 0x50
#define WRITE_STATUS_REGISTER 0x01

#define WRSR_NO_BLOCK_PROTECTION 0x00

extern SPI_HandleTypeDef hspi1;

void editWRSR(uint8_t *data);
void flashRead(uint32_t address, uint8_t *data, uint16_t size);
void flashWrite(uint32_t address, uint8_t *data, uint16_t size);

#endif /* INC_SPI_FLASH_H_ */
