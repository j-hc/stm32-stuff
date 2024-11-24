#include "at45db.h"
#include "gpio.h"
#include "spi.h"
#include <assert.h>

#define AT45DB_SPI_HANDLE hspi2
#define PG_SHIFT 9
#define PG_SIZE 267
#define STATUS_REG 0xD7

#define MNTHRUBF1 0x82
#define RDARRAYHF 0x0B

void at45db_transmit_receive(uint8_t *tx_data, uint16_t tx_size,
		uint8_t *rx_data, uint16_t rx_size) {
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_RESET);
	assert(
			HAL_SPI_Transmit(&AT45DB_SPI_HANDLE, tx_data, tx_size, 100) == HAL_OK);
	assert(
			HAL_SPI_Receive(&AT45DB_SPI_HANDLE, rx_data, rx_size, 100) == HAL_OK);
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
}

void at45db_transmit_byte(uint8_t data) {
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_RESET);
	assert(HAL_SPI_Transmit(&AT45DB_SPI_HANDLE, &data, 1, 100) == HAL_OK);
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
}

void at45db_transmit(uint8_t *data, uint16_t size) {
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_RESET);
	assert(HAL_SPI_Transmit(&AT45DB_SPI_HANDLE, data, size, 100) == HAL_OK);
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
}

uint8_t at45db_read_status(void) {
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_RESET);
	uint8_t data = 0xd7, ret = 0;
	assert(
			HAL_SPI_TransmitReceive(&AT45DB_SPI_HANDLE, &data, &ret, 1, 100) == HAL_OK);
	data = 0;
	assert(
			HAL_SPI_TransmitReceive(&AT45DB_SPI_HANDLE, &data, &ret, 1, 100) == HAL_OK);
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
	return ret;
}

void at45db_wait_busy(void) {
	uint8_t stat_reg = STATUS_REG;
	uint8_t status;
	at45db_transmit_receive(&stat_reg, 1, &status, 1);
	while ((status & 0x80) == 0) {
		HAL_Delay(1);
		at45db_transmit_receive(&stat_reg, 1, &status, 1);
	}
}

void at45db_write_page(uint8_t *data, uint16_t size, uint16_t page) {
	page = page << PG_SHIFT;
	at45db_transmit_byte(AT45DB_RESUME);
	at45db_wait_busy();

	uint8_t data_p[] = { MNTHRUBF1, (page >> 16) & 0xff, (page >> 8) & 0xff,
			page & 0xff };

	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_RESET);
	assert(HAL_SPI_Transmit(&AT45DB_SPI_HANDLE, data_p, 4, 100) == HAL_OK);
	assert(HAL_SPI_Transmit(&AT45DB_SPI_HANDLE, data, size, 100) == HAL_OK);
	HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
	at45db_wait_busy();
}

void at45db_read_page(uint8_t *data, uint16_t size, uint16_t page) {
	page = page << PG_SHIFT;
	assert(size <= PG_SIZE);

	at45db_transmit_byte(AT45DB_RESUME);
	at45db_wait_busy();

	uint8_t data_p[] = { RDARRAYHF, (page >> 16) & 0xff, (page >> 8) & 0xff,
			page & 0xff, 0 };
	at45db_transmit_receive(data_p, 5, data, size);
}
