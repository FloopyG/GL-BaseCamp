/*
 * spi_flash.c
 *
 *  Created on: 18 мар. 2023 г.
 *      Author: nazar
 */

#include "spi_flash.h"

void flashSelect(void)
{
	HAL_GPIO_WritePin(GPIOD, FLASH_SS_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
}
void flashDeselect(void)
{
	HAL_GPIO_WritePin(GPIOD, FLASH_SS_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
}

void flashWriteEnable(void)
{
	flashSelect();

	uint8_t addr = FLASH_WRITE_ENABLE;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);

	flashDeselect();
}

void flashWriteDisable(void)
{
	flashSelect();

	uint8_t addr = FLASH_WRITE_DISABLE;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);

	flashDeselect();
}

void flashRead(uint32_t address, uint8_t *data, uint16_t size)
{
	flashSelect();

	uint8_t addr[4] = {FLASH_READ, (uint8_t) (address >> 16), (uint8_t) (address >> 8), (uint8_t) address};
	HAL_SPI_TransmitReceive(&hspi1, addr, data, sizeof(addr) + size, HAL_MAX_DELAY);

	flashDeselect();
}

void flashWrite(uint32_t address, uint8_t *data, uint16_t size)
{
	flashWriteEnable();
	flashSelect();

	uint8_t addr[4] = {FLASH_WRITE, (uint8_t) (address >> 16), (uint8_t) (address >> 8), (uint8_t) address};
	HAL_SPI_Transmit(&hspi1, addr, sizeof(addr), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
	HAL_Delay(2000);
	flashDeselect();
	flashWriteDisable();
}
