#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"

#define FLASH_BASE (0x08000000UL)
#ifndef FLASH_PAGE_NB
    #define FLASH_PAGE_NB (64U)
#endif
#ifndef FLASH_PAGE_SIZE
    #define FLASH_PAGE_SIZE (2048U)
#endif

// flash map  NOTE: edit STM32G431CBUx_FLASH.ld file、production.py
#define BOOT_PAGE_START    (0)
#define BOOT_PAGE_NB       (13) // 26 kb

#define APP_PAGE_START     (13)
#define APP_PAGE_NB        (42) // 84 kb

#define EEPROM_PAGE_START  (55)
#define EEPROM_PAGE_NB     (8) // 16 kb

#define UID_KEY_PAGE_START (63)
#define UID_KEY_PAGE_NB    (1) // 2 kb

int flash_lock(void);
int flash_unlock(void);
int flash_program(uint32_t address, uint64_t data);
int flash_erase_pages(uint32_t page_start, uint32_t nb_page);

#endif
