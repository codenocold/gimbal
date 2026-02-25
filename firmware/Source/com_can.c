#include "com_can.h"
#include "util.h"
#include "soc.h"
#include "com.h"
#include "dfu.h"

void com_can_init(int can_nominal_idx, int can_data_idx)
{
    soc_can_set_baudrate(can_nominal_idx, can_data_idx);
}

void com_can_rx_confirmation(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    extern uint8_t g_node_id;

    uint32_t slave_id = (rx_header->Identifier & 0x000007FFU);
    uint8_t  function = rx_data[0];

    // slave id check
    if (slave_id != g_node_id && slave_id != 0) {
        return;
    }

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = rx_header->Identifier;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    uint8_t rsp[64];
    uint8_t rsp_length = 0;

    uint8_t tx_size;
    com_handler(function, &rx_data[1], &rsp[1], &tx_size);

    rsp[0]     = function;
    rsp_length = 1 + tx_size;

    // response
    if (tx_size) {
        if (rsp_length <= 8) {
            tx_header.DataLength = rsp_length;
        } else if (rsp_length <= 12) {
            tx_header.DataLength = FDCAN_DLC_BYTES_12;
        } else if (rsp_length <= 16) {
            tx_header.DataLength = FDCAN_DLC_BYTES_16;
        } else if (rsp_length <= 20) {
            tx_header.DataLength = FDCAN_DLC_BYTES_20;
        } else if (rsp_length <= 24) {
            tx_header.DataLength = FDCAN_DLC_BYTES_24;
        } else if (rsp_length <= 32) {
            tx_header.DataLength = FDCAN_DLC_BYTES_32;
        } else if (rsp_length <= 48) {
            tx_header.DataLength = FDCAN_DLC_BYTES_48;
        } else if (rsp_length <= 64) {
            tx_header.DataLength = FDCAN_DLC_BYTES_64;
        } else {
            tx_header.DataLength = FDCAN_DLC_BYTES_8;
        }

        soc_can_tx(&tx_header, rsp);

        if (function == FUNC_DFU_END) {
            if (rsp[1] == RSP_STATE_OK) {
                soc_delay_ms(50);
                dfu_app_run();
            }
        }
    }
}
