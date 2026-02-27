/* Includes ------------------------------------------------------------------ */
/* Extern variables ---------------------------------------------------------- */
/* Private types ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
/* Private constants --------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
/* Private function prototypes ----------------------------------------------- */
/* Private functions --------------------------------------------------------- */
/* Exported functions -------------------------------------------------------- */
/* Overloaded functions ------------------------------------------------------ */

/* Includes ------------------------------------------------------------------ */
#include "soc.h"
#include "util.h"
#include <stdbool.h>

/* Private types ------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan1;

/* Private define ------------------------------------------------------------ */
#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256

/* Private variables --------------------------------------------------------- */
static volatile uint32_t m_tick = 0;

static volatile bool    m_tx_busy = false;
static uint8_t          m_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t m_rx_buffer_idx   = 0;
static volatile uint8_t m_rx_buffer_count = 0;
static uint8_t          m_rx_buffer_0[UART_RX_BUFFER_SIZE];
static uint8_t          m_rx_buffer_1[UART_RX_BUFFER_SIZE];

/* Private constants --------------------------------------------------------- */

/* Private function prototypes ----------------------------------------------- */

/* Private functions --------------------------------------------------------- */

/* Exported functions -------------------------------------------------------- */
int32_t soc_init(void)
{
#ifdef __DEBUG__
    // rtt init
    SEGGER_RTT_Init();

    // JScope rtt init
    static char JS_RTT_UpBuffer[512];
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_f4f4f4", &JS_RTT_UpBuffer[0], sizeof(JS_RTT_UpBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif

    // systick init
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFUL;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    // 1ms timer
    LL_TIM_EnableIT_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);

    // lpuart init
    LL_LPUART_Disable(LPUART1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t) m_tx_buffer, LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_RECEIVE), (uint32_t) m_rx_buffer_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_LPUART_EnableDMAReq_TX(LPUART1);
    LL_LPUART_EnableDMAReq_RX(LPUART1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
    LL_LPUART_EnableIT_IDLE(LPUART1);
    LL_LPUART_EnableIT_ERROR(LPUART1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, UART_RX_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_LPUART_Enable(LPUART1);

    return 0;
}

void soc_delay_us(uint32_t us)
{
    uint32_t elapsed    = 0;
    uint32_t last_count = SysTick->VAL;

    for (;;) {
        uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_uS;

        /* measure the time elapsed since the last time we checked */
        if (last_count > current_count) {
            elapsed += last_count - current_count;
        } else if (last_count < current_count) {
            elapsed += last_count + (0xFFFFFF - current_count);
        }

        last_count = current_count;

        /* convert to microseconds */
        elapsed_uS = elapsed / SYSTICK_US_TICK;
        if (elapsed_uS >= us) {
            break;
        }

        /* reduce the delay by the elapsed time */
        us -= elapsed_uS;

        /* keep fractional microseconds for the next iteration */
        elapsed %= SYSTICK_US_TICK;
    }
}

void soc_delay_ms(uint32_t ms)
{
    while (ms--) {
        soc_delay_us(1000);
    }
}

inline void soc_tick_inc(void)
{
    m_tick++;
}

inline uint32_t soc_get_tick(void)
{
    return m_tick == 0 ? 1 : m_tick;
}

inline uint32_t soc_get_ms_since(uint32_t tick)
{
    return (uint32_t) (m_tick - tick);
}

void soc_can_set_baudrate(int nominal_baudrate_idx, int data_baudrate_idx)
{
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);

    // can clk = 160M
    switch (nominal_baudrate_idx) {
    case 0: // 250k 80%
        hfdcan1.Init.NominalPrescaler = 32;
        hfdcan1.Init.NominalTimeSeg1  = 15;
        hfdcan1.Init.NominalTimeSeg2  = 4;
        break;

    case 1: // 500k 60%
        hfdcan1.Init.NominalPrescaler = 16;
        hfdcan1.Init.NominalTimeSeg1  = 11;
        hfdcan1.Init.NominalTimeSeg2  = 8;
        break;

    case 2: // 800K 68%
        hfdcan1.Init.NominalPrescaler = 8;
        hfdcan1.Init.NominalTimeSeg1  = 16;
        hfdcan1.Init.NominalTimeSeg2  = 8;
        break;

    case 3: // 1M 75%
        hfdcan1.Init.NominalPrescaler = 8;
        hfdcan1.Init.NominalTimeSeg1  = 14;
        hfdcan1.Init.NominalTimeSeg2  = 5;
        break;

    default:
        // 1M 75%
        hfdcan1.Init.NominalPrescaler = 8;
        hfdcan1.Init.NominalTimeSeg1  = 14;
        hfdcan1.Init.NominalTimeSeg2  = 5;
        break;
    }

    switch (data_baudrate_idx) {
    case 0: // 1M 75%
        hfdcan1.Init.DataPrescaler = 8;
        hfdcan1.Init.DataTimeSeg1  = 14;
        hfdcan1.Init.DataTimeSeg2  = 5;
        break;

    case 1: // 2M 75%
        hfdcan1.Init.DataPrescaler = 4;
        hfdcan1.Init.DataTimeSeg1  = 14;
        hfdcan1.Init.DataTimeSeg2  = 5;
        break;

    case 2: // 4M 75%
        hfdcan1.Init.DataPrescaler = 2;
        hfdcan1.Init.DataTimeSeg1  = 14;
        hfdcan1.Init.DataTimeSeg2  = 5;
        break;

    case 3: // 5M 75%
        hfdcan1.Init.DataPrescaler = 2;
        hfdcan1.Init.DataTimeSeg1  = 11;
        hfdcan1.Init.DataTimeSeg2  = 4;
        break;

    default:
        // 4M 75%
        hfdcan1.Init.DataPrescaler = 2;
        hfdcan1.Init.DataTimeSeg1  = 14;
        hfdcan1.Init.DataTimeSeg2  = 5;
        break;
    }

    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.AutoRetransmission   = DISABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.DataSyncJumpWidth    = 4;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0;
    sFilterConfig.FilterID2    = 0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Configure global filter on both FDCAN instances:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    /* Configure and enable Tx Delay Compensation, required for BRS mode.
     TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
     TdcFilter default recommended value: 0 */
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler, 0) != HAL_OK) {
        Error_Handler();
    }

    if (data_baudrate_idx > 0) {
        if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
            Error_Handler();
        }
    } else {
        if (HAL_FDCAN_DisableTxDelayCompensation(&hfdcan1) != HAL_OK) {
            Error_Handler();
        }
    }

    /* Start the FDCAN module on both FDCAN instances */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

void soc_can_tx(FDCAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data)
{
    /* add message to TX FIFO of FDCAN instance 1 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, tx_header, tx_data) != HAL_OK) {
        Error_Handler();
    }
}

void soc_uart_set_baudrate(int baudrate_idx)
{
    uint32_t baudrate = 115200;

    switch (baudrate_idx) {
    case 0:
        baudrate = 115200;
        break;

    case 1:
        baudrate = 1000000;
        break;

    case 2:
        baudrate = 2000000;
        break;

    case 3:
        baudrate = 3000000;
        break;

    case 4:
        baudrate = 4000000;
        break;

    case 5:
        baudrate = 6000000;
        break;

    default:
        break;
    }

    LL_LPUART_Disable(LPUART1);
    LL_LPUART_SetBaudRate(LPUART1, LL_RCC_GetLPUARTClockFreq(LL_RCC_LPUART1_CLKSOURCE), LL_LPUART_PRESCALER_DIV1, baudrate);
    LL_LPUART_Enable(LPUART1);
}

inline int soc_uart_tx_start(int count)
{
    if (count == 0) {
        return 0;
    }

    if (m_tx_busy) {
        return -1;
    }

    m_tx_busy = true;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, count);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    return 0;
}

inline uint8_t *soc_uart_get_tx_buffer(void)
{
    return m_tx_buffer;
}

inline void soc_uart_tx_error_callback(void)
{
    m_tx_busy = false;
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
}

inline void soc_uart_tx_complete_callback(void)
{
    m_tx_busy = false;
}

inline void soc_uart_rx_complete_callback(void)
{
    m_rx_buffer_count = (uint8_t) (UART_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2));

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    if (m_rx_buffer_idx == 0) {
        LL_DMA_ConfigAddresses(DMA1,
                               LL_DMA_CHANNEL_2,
                               LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_RECEIVE),
                               (uint32_t) m_rx_buffer_1,
                               LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        m_rx_buffer_idx = 1;
    } else {
        LL_DMA_ConfigAddresses(DMA1,
                               LL_DMA_CHANNEL_2,
                               LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_RECEIVE),
                               (uint32_t) m_rx_buffer_0,
                               LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        m_rx_buffer_idx = 0;
    }

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, UART_RX_BUFFER_SIZE);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

inline void soc_uart_reset_rx_count(void)
{
    m_rx_buffer_count = 0;
}

inline uint8_t soc_uart_get_rx_count(void)
{
    return m_rx_buffer_count;
}

inline uint8_t *soc_uart_get_rx_buffer(void)
{
    return m_rx_buffer_idx == 0 ? m_rx_buffer_1 : m_rx_buffer_0;
}

void soc_led_on(void)
{
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
}

void soc_led_off(void)
{
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
}

void soc_led_toggle(void)
{
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* Overloaded functions ------------------------------------------------------ */
