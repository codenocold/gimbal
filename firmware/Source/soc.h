/* Define to prevent recursive inclusion ------------------------------------- */
/* Includes ------------------------------------------------------------------ */
/* Exported types ------------------------------------------------------------ */
/* Exported constants -------------------------------------------------------- */
/* Exported defines ---------------------------------------------------------- */
/* Exported functions prototypes --------------------------------------------- */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef __soc_H__
#define __soc_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------ */
#include "main.h"

/* Exported defines ---------------------------------------------------------- */
#ifdef __DEBUG__
    // clang-format off
    #include <stdio.h>
    #include "SEGGER_RTT.h"
    #define DEBUG(format, ...) do { \
        char _debug_buffer[128]; \
        int _debug_len = snprintf(_debug_buffer, sizeof(_debug_buffer), format, ##__VA_ARGS__); \
        if (_debug_len > 0) { \
            unsigned _write_len = (unsigned) ((_debug_len < (int) sizeof(_debug_buffer)) ? _debug_len : ((int) sizeof(_debug_buffer) - 1)); \
            SEGGER_RTT_Write(0, _debug_buffer, _write_len); \
        } \
    } while (0)
    inline void DEBUG_PLOT(float f1, float f2, float f3) { float value[3] = {f1, f2, f3}; SEGGER_RTT_Write(1, &value, sizeof(value)); }
    // clang-format on
#else
    #define DEBUG(format, ...)
    #define DEBUG_PLOT(value)
#endif

#define SYSTEM_CLOCK    160000000
#define SYSTICK_US_TICK (SYSTEM_CLOCK / 1000000)
#define SYSTICK_MS_TICK (SYSTEM_CLOCK / 1000)

/* Exported types ------------------------------------------------------------ */

/* Exported functions prototypes --------------------------------------------- */
int32_t soc_init(void);
void    soc_delay_us(uint32_t us);
void    soc_delay_ms(uint32_t ms);

void     soc_tick_inc(void);
uint32_t soc_get_tick(void);
uint32_t soc_get_ms_since(uint32_t tick);

void soc_can_set_baudrate(int nominal_baudrate_idx, int data_baudrate_idx);
void soc_can_tx(FDCAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data);

void     soc_uart_set_baudrate(int baudrate_idx);
int      soc_uart_tx_start(int count);
uint8_t *soc_uart_get_tx_buffer(void);
void     soc_uart_tx_error_callback(void);
void     soc_uart_tx_complete_callback(void);
void     soc_uart_rx_complete_callback(void);
void     soc_uart_reset_rx_count(void);
uint8_t  soc_uart_get_rx_count(void);
uint8_t *soc_uart_get_rx_buffer(void);

void soc_led_on(void);
void soc_led_off(void);
void soc_led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __soc_H__ */