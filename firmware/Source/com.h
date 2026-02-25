#ifndef _COM_H_
#define _COM_H_

#include "com_485.h"
#include "com_can.h"

// function code
#define FUNC_DFU_ENTER      0xFA
#define FUNC_DFU_START      0xFB
#define FUNC_DFU_DATA       0xFC
#define FUNC_DFU_END        0xFD
#define FUNC_SYSTEM_REBOOT  0xFE
#define FUNC_GET_FW_VERSION 0xFF

// return state
#define RSP_STATE_OK        0x00
#define RSP_STATE_ERR       0xEE

void com_init(int node_id, int com_baudrate_idx, int can_nominal_idx, int can_data_idx);
void com_handler(uint8_t func, uint8_t *rx_data, uint8_t *tx_data, uint8_t *tx_size);

#endif