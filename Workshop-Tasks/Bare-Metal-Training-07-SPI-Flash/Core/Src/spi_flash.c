/*
 * spi_flash.c
 *
 *  Created on: 18 мар. 2023 г.
 *      Author: nazar
 */

#include "spi_flash.h"

static void flashSelect(void)
{
	HAL_GPIO_WritePin(GPIOD, FLASH_SS_PIN, GPIO_PIN_RESET);
}
static void flashDeselect(void)
{
	HAL_GPIO_WritePin(GPIOD, FLASH_SS_PIN, GPIO_PIN_SET);
}

static void flashWriteEnable(void)
{
	flashSelect();

	uint8_t addr = FLASH_WRITE_ENABLE;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);

	flashDeselect();
}

static void flashWriteDisable(void)
{
	flashSelect();

	uint8_t addr = FLASH_WRITE_DISABLE;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);

	flashDeselect();
}

void editWRSR(uint8_t *data)
{

	flashSelect();

	uint8_t addr = ENABLE_WRITE_STATUS_REGISTER;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);

	flashDeselect();

	flashWriteEnable();
	flashSelect();

	addr = WRITE_STATUS_REGISTER;
	HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, data, 1, HAL_MAX_DELAY);

	flashDeselect();
	flashWriteDisable();
}

void flashRead(uint32_t address, uint8_t *data, uint16_t size)
{
	flashSelect();

	uint8_t addr[4] = {FLASH_READ, (uint8_t) (address >> 16), (uint8_t) (address >> 8), (uint8_t) address};
	HAL_SPI_Transmit(&hspi1, addr, sizeof(addr), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);

	flashDeselect();
}

void flashWrite(uint32_t address, uint8_t *data, uint16_t size)
{
	for (int i = 0; i < size; ++i)
	{
		flashWriteEnable();
		flashSelect();
		uint8_t byte = data[i];
		uint8_t addr[4] = {FLASH_WRITE, (uint8_t) (address >> 16), (uint8_t) (address >> 8), (uint8_t) address};
		HAL_SPI_Transmit(&hspi1, addr, sizeof(addr), HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi1, &byte, 1, HAL_MAX_DELAY);
		flashDeselect();
		address += 1;
	}

	flashWriteDisable();
}
