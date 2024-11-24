#ifndef __MODISH_BUS_H
#define __MODISH_BUS_H

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AREG_TY uint32_t
#define AREG_SZ sizeof(AREG_TY)

#define ERROR_TY uint8_t
#define ERROR_SZ sizeof(uint8_t)

#define CRC_SZ sizeof(uint16_t)

typedef struct {
	AREG_TY *r;
	AREG_TY **reg_lookup_table;
	uint32_t size;
} Analog;

typedef struct RegContainer {
	Analog AO;
	Analog AI;
} RegContainer;

typedef enum Cmd {
	R_AO = 0, R_AI, W_AI, W_AO,
} Cmd;

typedef struct {
	uint8_t dev_id;
	uint8_t cmd;
} ReqHeader;

typedef struct {
	ReqHeader header;
	uint16_t first_reg_idx;
	uint16_t reg_count;
	uint16_t crc;
} R_Req;

typedef struct {
	ReqHeader header;
	uint16_t first_reg_idx;
	uint16_t reg_count;
	uint8_t *data;
	uint16_t crc;
} W_Req;

typedef void (*w_reg_callback_t)(Analog *prev, W_Req*);
typedef void (*r_reg_callback_t)(Analog *prev, R_Req*);

#define FIRST_READ_SZ (sizeof(ReqHeader) + 2*sizeof(uint16_t))

size_t next_read_sz(uint8_t *buf);

typedef struct {
	uint8_t dev_id;
} Dev;

typedef enum {
	NO_ERROR = 0, SOME_ERROR = 1,
} Error;

typedef void (*dev_write_slave_t)(uint8_t *data, size_t data_size);

RegContainer new_reg_container(size_t AI_sz, size_t AO_sz, AREG_TY **reg_lookup_table_i, AREG_TY **reg_lookup_table_o);
uint8_t* serialize_wreq(W_Req *w_req, size_t *query_size);
uint8_t* serialize_rreq(R_Req *r_req, size_t *query_size);

void query(r_reg_callback_t r_reg_cb, w_reg_callback_t w_reg_cb, dev_write_slave_t, Dev *dev, RegContainer *rc,
		uint8_t *query);

#endif  // __MODISH_BUS_H
