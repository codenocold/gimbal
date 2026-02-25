#ifndef __COM_485_H__
#define __COM_485_H__

#include "main.h"

/*
    usr protocol: [slaveID] + [function] + {data N bytes} + {crc16 2 bytes}

    FUNC_DFU_ENTER
        rx: [slaveID] + [function] + {crc16 2 bytes}
        tx: [slaveID] + [function] + [size] + [state] + {crc16 2 bytes}

    FUNC_DFU_START
        rx: [slaveID] + [function] + {crc16 2 bytes}
        tx: [slaveID] + [function] + [size] + [state] + {crc16 2 bytes}

    FUNC_DFU_DATA
        rx: [slaveID] + [function] + {data 16 bytes} + {crc16 2 bytes}
        tx: [slaveID] + [function] + [size] + [state] + {crc16 2 bytes}

    FUNC_DFU_END
        rx: [slaveID] + [function] + {crc16 2 bytes}
        tx: [slaveID] + [function] + [size] + [state] + {crc16 2 bytes}

    FUNC_GET_FW_VERSION
        rx: [slaveID] + [function] + {crc16 2 bytes}
        tx: [slaveID] + [function] + [size] + [major] + [minor] + [patch] + {crc16 2 bytes}
*/

void com_485_init(int baudrate_idx);
void com_485_rx_confirmation(uint8_t *rx_data, uint8_t rx_count);

#endif
