#include "com.h"
#include "util.h"
#include "dfu.h"

uint8_t g_node_id = 1;

void com_init(int node_id, int com_baudrate_idx, int can_nominal_idx, int can_data_idx)
{
    g_node_id = node_id;
    com_485_init(com_baudrate_idx);
    com_can_init(can_nominal_idx, can_data_idx);
}

void com_handler(uint8_t func, uint8_t *rx_data, uint8_t *tx_data, uint8_t *tx_size)
{
    int ret;

    (*tx_size) = 0;

    switch (func) {
    case FUNC_DFU_ENTER:
        tx_data[(*tx_size)++] = RSP_STATE_OK;
        break;

    case FUNC_DFU_START:
        ret                   = dfu_start();
        tx_data[(*tx_size)++] = (ret == 0) ? RSP_STATE_OK : RSP_STATE_ERR;
        break;

    case FUNC_DFU_DATA:
        ret                   = dfu_data(&rx_data[0]);
        tx_data[(*tx_size)++] = (ret == 0) ? RSP_STATE_OK : RSP_STATE_ERR;
        break;

    case FUNC_DFU_END:
        ret                   = dfu_end();
        tx_data[(*tx_size)++] = (ret == 0) ? RSP_STATE_OK : RSP_STATE_ERR;
        break;

    case FUNC_GET_FW_VERSION:
        tx_data[(*tx_size)++] = 0;
        tx_data[(*tx_size)++] = 0;
        tx_data[(*tx_size)++] = 0;
        break;

    default:
        break;
    }
}