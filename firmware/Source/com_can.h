#ifndef __COM_CAN_H__
#define __COM_CAN_H__

#include "main.h"

/*
    can id:   slaveID(11 bit)
    can data: [function] + {data N bytes}

    FUNC_DFU_ENTER
        rx: [function]
        tx: [function] + [state]

    FUNC_DFU_START
        rx: [function]
        tx: [function] + [state]

    FUNC_DFU_DATA
        rx: [function] + {data 16 bytes}
        tx: [function] + [state]

    FUNC_DFU_END
        rx: [function]
        tx: [function] + [state]

    FUNC_GET_FW_VERSION
        rx: [function]
        tx: [function] + [major] + [minor] + [patch]
*/

void com_can_init(int can_nominal_idx, int can_data_idx);
void com_can_rx_confirmation(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

#endif