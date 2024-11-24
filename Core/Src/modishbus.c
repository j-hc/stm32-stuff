#include "modishbus.h"
#include "main.h"

void* malloc_zeroed(size_t sz) {
	void *addr = malloc(sz);
	memset(addr, 0, sz);
	return addr;
}

RegContainer new_reg_container(size_t AI_sz, size_t AO_sz, AREG_TY **reg_lookup_table_i, AREG_TY **reg_lookup_table_o) {
	RegContainer rc;
	rc.AO.r = (AREG_TY*) malloc_zeroed(AO_sz * AREG_SZ);
	rc.AO.size = AO_sz;
	rc.AO.reg_lookup_table = reg_lookup_table_o;
	rc.AI.r = (AREG_TY*) malloc_zeroed(AI_sz * AREG_SZ);
	rc.AI.size = AI_sz;
	rc.AI.reg_lookup_table = reg_lookup_table_i;
	return rc;
}

uint8_t* serialize_wreq(W_Req *w_req, size_t *query_size) {
	size_t data_sz = sizeof(W_Req) + w_req->reg_count - sizeof(void*);
	uint8_t *data = (uint8_t*) malloc(data_sz);
	size_t cur = 0;
	memcpy(data, w_req, sizeof(ReqHeader) + sizeof(uint16_t) * 2);
	cur += sizeof(ReqHeader) + sizeof(uint16_t) * 2;
	memcpy(data + cur, w_req->data, w_req->reg_count * AREG_SZ);
	cur += w_req->reg_count * AREG_SZ;
	*(uint16_t*) (data + cur) = w_req->crc;
	cur += sizeof(uint16_t);
	*query_size = cur;
	return data;
}

uint8_t* serialize_rreq(R_Req *r_req, size_t *query_size) {
	*query_size = sizeof(R_Req);
	return (uint8_t*) r_req;
}

size_t next_read_sz(uint8_t *query) {
	size_t cur = 0;
	uint8_t dev_id = query[cur++];
	(void) dev_id;
	uint8_t cmd = query[cur++];
	uint16_t first_reg_idx = *((uint16_t*) (query + cur));
	(void) first_reg_idx;
	cur += sizeof(uint16_t);
	uint16_t reg_count = *((uint16_t*) (query + cur));
	switch (cmd) {
	case R_AO:
	case R_AI:
		return CRC_SZ;
	case W_AO:
		return reg_count * AREG_SZ + CRC_SZ;
	case W_AI:
		return reg_count * AREG_SZ + CRC_SZ;
	default:
		assert(0 && "unreachable");
	}
}

void query_read(r_reg_callback_t reg_cb, dev_write_slave_t dev_write_slave, Analog reg, uint8_t *query, size_t cur,
		uint16_t reg_count, uint8_t dev_id, uint8_t cmd, uint16_t first_reg_idx) {
	uint16_t crc = (uint16_t*) (query + cur); // TODO: crc check

	assert(reg.size >= reg_count + first_reg_idx);
	size_t data_size = sizeof(ReqHeader) + reg_count * AREG_SZ + ERROR_SZ + CRC_SZ;
	uint8_t *data = malloc(data_size);
	size_t cur_data = 0;
	data[cur_data++] = dev_id;
	data[cur_data++] = cmd;
	for (size_t reg_i = 0; reg_i < reg_count; reg_i++) {
		uint32_t *data_u32 = (AREG_TY*) (data + cur_data);
		uint32_t *addr = reg.reg_lookup_table[first_reg_idx + reg_i];
		data_u32[reg_i] = *addr;
	}
	cur_data += reg_count * AREG_SZ;
	data[cur_data] = SOME_ERROR;  // TODO: error check
	cur_data += ERROR_SZ;
	*(uint16_t*) (data + cur_data) = 0xCFCF;  // TODO: calculate crc

	if (reg_cb != NULL) {
		R_Req r_req = (R_Req ) { { dev_id, cmd }, first_reg_idx, reg_count, crc };
		reg_cb(&reg, &r_req);
	}

	dev_write_slave(data, data_size);
	free(data);
}

void query_write(w_reg_callback_t reg_cb, dev_write_slave_t dev_write_slave, Analog reg, uint8_t *query, size_t cur,
		uint16_t reg_count, uint8_t dev_id, uint8_t cmd, uint16_t first_reg_idx) {
	assert(reg.size >= reg_count + first_reg_idx);

	AREG_TY *data_u32 = (AREG_TY*) (query + cur);
	for (size_t reg_i = 0; reg_i < reg_count; reg_i++) {
		AREG_TY new_reg_val = data_u32[reg_i];
		uint32_t *addr = reg.reg_lookup_table[first_reg_idx + reg_i];
		*addr = new_reg_val;
	}
	uint16_t crc = (uint16_t*) (query + cur + reg_count * AREG_SZ); // TODO: crc check

	size_t query_echo_size = sizeof(ReqHeader) + sizeof(uint16_t) * 2 + reg_count * AREG_SZ + ERROR_SZ + CRC_SZ;
	uint8_t *query_echo = malloc(query_echo_size);
	memcpy(query_echo, query, query_echo_size);
	*(uint8_t*) (query_echo + query_echo_size - ERROR_SZ - CRC_SZ) = SOME_ERROR; // TODO: get error
	*(uint16_t*) (query_echo + query_echo_size - ERROR_SZ) = 0xCCCC; // TODO: calculate crc

	if (reg_cb != NULL) {
		W_Req w_req = (W_Req ) { { dev_id, cmd }, first_reg_idx, reg_count, (uint8_t*) data_u32, crc };
		reg_cb(&reg, &w_req);
	}

	dev_write_slave(query_echo, query_echo_size);
	free(query_echo);
}

void query(r_reg_callback_t r_reg_cb, w_reg_callback_t w_reg_cb, dev_write_slave_t dev_write_slave, Dev *dev_slave,
		RegContainer *rc, uint8_t *query) {
	size_t cur = 0;
	uint8_t dev_id = query[cur++];
	if (dev_slave->dev_id != dev_id) {
		logd("device id wrong\n");
		return;
	}

	logd("query -> ");
	for (size_t i = 0; i < 8; i++) {
		logd("0x%X ", query[i]);
	}
	logd("\n");

	uint8_t cmd = query[cur++];
	uint16_t first_reg_idx = *((uint16_t*) (query + cur));

	cur += sizeof(uint16_t);
	uint16_t reg_count = *((uint16_t*) (query + cur));
	cur += sizeof(uint16_t);
	switch (cmd) {
	break;
case R_AO: {
	query_read(r_reg_cb, dev_write_slave, rc->AO, query, cur, reg_count, dev_id, cmd, first_reg_idx);
}
	break;
case R_AI: {
	query_read(r_reg_cb, dev_write_slave, rc->AI, query, cur, reg_count, dev_id, cmd, first_reg_idx);
}
	break;
case W_AI: {
	query_write(w_reg_cb, dev_write_slave, rc->AI, query, cur, reg_count, dev_id, cmd, first_reg_idx);
}
	break;
case W_AO: {
	query_write(w_reg_cb, dev_write_slave, rc->AO, query, cur, reg_count, dev_id, cmd, first_reg_idx);
}
	break;
default: {
	assert(0 && "unreachable");
}
	break;
	}
}
