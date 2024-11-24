#ifndef __AT45DB_H
#define __AT45DB_H

#include <stdint.h>

#define AT45DB_RESUME 0xAB
#define AT45DB_DEVID 0x9F

void at45db_transmit_receive(uint8_t *tx_data, uint16_t tx_size,
		uint8_t *rx_data, uint16_t rx_size);
uint8_t at45db_read_status(void);
void at45db_transmit_byte(uint8_t data);
void at45db_transmit(uint8_t *data, uint16_t size);

void at45db_write_page(uint8_t *data, uint16_t size, uint16_t page);
void at45db_read_page(uint8_t *data, uint16_t size, uint16_t page);

#endif //__AT45DB_H
