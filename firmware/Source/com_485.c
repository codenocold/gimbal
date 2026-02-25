#include "com_485.h"
#include "util.h"
#include "com.h"
#include "soc.h"
#include "dfu.h"

void com_485_init(int baudrate_idx)
{
    soc_uart_set_baudrate(baudrate_idx);
}

void com_485_rx_confirmation(uint8_t *rx_data, uint8_t rx_count)
{
    extern uint8_t g_node_id;

    uint8_t slave_id = rx_data[0];
    uint8_t function = rx_data[1];

    // slave id check
    if (slave_id != g_node_id && slave_id != 0) {
        return;
    }

    // length check
    if (rx_count < 4) {
        return;
    }

    // crc check
    uint16_t crc_rx = data_to_uint16(&rx_data[rx_count - 2]);
    if (crc_rx != crc16(0xFFFF, rx_data, rx_count - 2)) {
        return;
    }

    uint8_t *rsp        = soc_uart_get_tx_buffer();
    uint8_t  rsp_length = 0;

    uint8_t tx_size = 0;
    com_handler(function, &rx_data[2], &rsp[3], &tx_size);
    rsp[0]     = slave_id;
    rsp[1]     = function;
    rsp[2]     = 5 + tx_size;
    rsp_length = 3 + tx_size;

    // response
    if (tx_size) {
        // fill crc16
        uint16_to_data(crc16(0xFFFF, rsp, rsp_length), &rsp[rsp_length]);
        rsp_length += 2;

        // send ack
        soc_uart_tx_start(rsp_length);

        if (function == FUNC_DFU_END) {
            if (rsp[3] == RSP_STATE_OK) {
                soc_delay_ms(50);
                dfu_app_run();
            }
        }
    }
}