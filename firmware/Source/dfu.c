#include "dfu.h"
#include "aes.h"
#include "util.h"
#include "flash.h"
#include "eeprom_emul.h"
#include <string.h>

static uint32_t m_rec_app_size = 0;
static uint32_t m_app_size     = 0;
static uint32_t m_app_crc      = 0;

static struct AES_ctx ctx;
static const uint8_t  aes_key[] = {0x11, 0x45, 0x15, 0x74, 0x28, 0xae, 0xd7, 0xa6, 0xab, 0xf7, 0x47, 0x04, 0x09, 0xcf, 0x4f, 0x3c};

int dfu_start(void)
{
    int ret;

    // erase app
    flash_unlock();
    ret = flash_erase_pages(APP_PAGE_START, APP_PAGE_NB);
    flash_lock();
    if (ret) {
        return ret;
    }

    m_rec_app_size = 0;
    m_app_size     = 0;
    m_app_crc      = 0;
    AES_init_ctx(&ctx, aes_key);

    return 0;
}

int dfu_data(uint8_t *data)
{
    if (m_rec_app_size > ((APP_PAGE_NB * FLASH_PAGE_SIZE) - 16)) {
        return -1;
    }

    uint8_t buf[16];
    memcpy(buf, data, 16);

    AES_ECB_decrypt(&ctx, buf);

    if (m_rec_app_size == 0) {
        m_app_size = REG32(buf);
        m_app_crc  = REG32(buf + 8);
    } else {
        uint64_t w_data;

        flash_unlock();

        memcpy(&w_data, buf, 8);
        flash_program((FLASH_BASE + APP_PAGE_START * FLASH_PAGE_SIZE + m_rec_app_size - 16), w_data);

        memcpy(&w_data, buf + 8, 8);
        flash_program((FLASH_BASE + APP_PAGE_START * FLASH_PAGE_SIZE + m_rec_app_size - 8), w_data);

        flash_lock();
    }

    m_rec_app_size += 16;

    return 0;
}

#include "soc.h"

int dfu_end(void)
{
    if (m_rec_app_size == 0) {
        return -1;
    }

    if (m_app_size != (m_rec_app_size - 16)) {
        return -2;
    }

    if (dfu_app_check(m_app_size, m_app_crc)) {
        return -3;
    }

    EE_Status status = EE_OK;

    // write app size
    status = EE_WriteVariable32bits(APP_SIZE_ADDR, m_app_size);
    if (status & EE_STATUSMASK_CLEANUP) {
        EE_CleanUp();
    }
    if (status & EE_STATUSMASK_ERROR) {
        return -4;
    }

    // write app crc
    status = EE_WriteVariable32bits(APP_CRC_ADDR, m_app_crc);
    if (status & EE_STATUSMASK_CLEANUP) {
        EE_CleanUp();
    }
    if (status & EE_STATUSMASK_ERROR) {
        return -5;
    }

    // clear dfu flag
    status = EE_WriteVariable32bits(DFU_FLAG_ADDR, DFU_FLAG_NONE);
    if (status & EE_STATUSMASK_CLEANUP) {
        EE_CleanUp();
    }
    if (status & EE_STATUSMASK_ERROR) {
        return -6;
    }

    return 0;
}

int dfu_app_check(uint32_t app_size, uint32_t app_crc)
{
    if (app_size == 0) {
        return 0;
    }

    if (app_size > (APP_PAGE_NB * FLASH_PAGE_SIZE)) {
        return -1;
    }

    uint32_t crc_calc = crc32(0xFFFFFFFF, (uint8_t *) (FLASH_BASE + APP_PAGE_START * FLASH_PAGE_SIZE), app_size);
    if (crc_calc != app_crc) {
        return -2;
    }

    return 0;
}

void dfu_app_run(void)
{
    for (int i = 0; i < 8; i++) {
        // Disable all interrupts
        NVIC->ICER[i] = 0xFFFFFFFF;
        // Clear pending interrupts
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* Initialize user application's Stack Pointer */
    __set_MSP(*(uint32_t *) (FLASH_BASE + APP_PAGE_START * FLASH_PAGE_SIZE));

    /* Jump to the bootloader */
    (*(void (*)(void))(*(uint32_t *) (FLASH_BASE + APP_PAGE_START * FLASH_PAGE_SIZE + 4)))();

    while (1)
        ;
}
