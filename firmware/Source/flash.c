#include "flash.h"
#include "soc.h"

#ifndef HAL_FLASH_MODULE_ENABLED
typedef enum
{
    FLASH_CACHE_DISABLED = 0,
    FLASH_CACHE_ICACHE_ENABLED,
    FLASH_CACHE_DCACHE_ENABLED,
    FLASH_CACHE_ICACHE_DCACHE_ENABLED
} FLASH_CacheTypeDef;
#endif

#define FLASH_TIMEOUT_MS       1000U

#define FLASH_KEY1             0x45670123U
#define FLASH_KEY2             0xCDEF89ABU

#define FLASH_FLAG_EOP         FLASH_SR_EOP     /*!< FLASH End of operation flag */
#define FLASH_FLAG_OPERR       FLASH_SR_OPERR   /*!< FLASH Operation error flag */
#define FLASH_FLAG_PROGERR     FLASH_SR_PROGERR /*!< FLASH Programming error flag */
#define FLASH_FLAG_WRPERR      FLASH_SR_WRPERR  /*!< FLASH Write protection error flag */
#define FLASH_FLAG_PGAERR      FLASH_SR_PGAERR  /*!< FLASH Programming alignment error flag */
#define FLASH_FLAG_SIZERR      FLASH_SR_SIZERR  /*!< FLASH Size error flag  */
#define FLASH_FLAG_PGSERR      FLASH_SR_PGSERR  /*!< FLASH Programming sequence error flag */
#define FLASH_FLAG_MISERR      FLASH_SR_MISERR  /*!< FLASH Fast programming data miss error flag */
#define FLASH_FLAG_FASTERR     FLASH_SR_FASTERR /*!< FLASH Fast programming error flag */
#define FLASH_FLAG_RDERR       FLASH_SR_RDERR   /*!< FLASH PCROP read error flag */
#define FLASH_FLAG_OPTVERR     FLASH_SR_OPTVERR /*!< FLASH Option validity error flag  */
#define FLASH_FLAG_BSY         FLASH_SR_BSY     /*!< FLASH Busy flag */
#define FLASH_FLAG_ECCC        FLASH_ECCR_ECCC  /*!< FLASH ECC correction in 64 LSB bits */
#define FLASH_FLAG_ECCD        FLASH_ECCR_ECCD  /*!< FLASH ECC detection in 64 LSB bits */

#define FLASH_FLAG_ECCR_ERRORS (FLASH_FLAG_ECCC | FLASH_FLAG_ECCD)
#define FLASH_FLAG_SR_ERRORS                                                                                                                                        \
    (FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR \
     | FLASH_FLAG_RDERR | FLASH_FLAG_OPTVERR)

#define FLASH_INSTRUCTION_CACHE_ENABLE()  SET_BIT(FLASH->ACR, FLASH_ACR_ICEN)
#define FLASH_DATA_CACHE_DISABLE()        CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define FLASH_DATA_CACHE_ENABLE()         SET_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define FLASH_INSTRUCTION_CACHE_DISABLE() CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN)
#define FLASH_INSTRUCTION_CACHE_RESET()         \
    do {                                        \
        SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);   \
        CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST); \
    } while (0)

#define FLASH_DATA_CACHE_RESET()                \
    do {                                        \
        SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);   \
        CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST); \
    } while (0)

#define FLASH_GET_FLAG(__FLAG__) \
    ((((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) ? (READ_BIT(FLASH->ECCR, (__FLAG__)) == (__FLAG__)) : (READ_BIT(FLASH->SR, (__FLAG__)) == (__FLAG__)))
#define FLASH_CLEAR_FLAG(__FLAG__)                                          \
    do {                                                                    \
        if (((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) {                  \
            SET_BIT(FLASH->ECCR, ((__FLAG__) & FLASH_FLAG_ECCR_ERRORS));    \
        }                                                                   \
        if (((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS)) != 0U) {               \
            WRITE_REG(FLASH->SR, ((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS))); \
        }                                                                   \
    } while (0)

static void page_erase(uint32_t page);
static void program_double_word(uint32_t address, uint64_t data);
static int  wait_for_last_operation(uint32_t timeout);

int flash_lock(void)
{
    int error = -1;

    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    /* verify Flash is locked */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U) {
        error = 0;
    }

    return error;
}

int flash_unlock(void)
{
    int error = 0;

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U) {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* verify Flash is unlocked */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U) {
            error = -1;
        }
    }

    return error;
}

int flash_program(uint32_t address, uint64_t data)
{
    int error;

    /* Wait for last operation to be completed */
    error = wait_for_last_operation(FLASH_TIMEOUT_MS);

    if (error == 0) {
        FLASH_CacheTypeDef cache;

        /* Deactivate the data cache if they are activated to avoid data misbehavior */
        if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U) {
            /* Disable data cache  */
            FLASH_DATA_CACHE_DISABLE();
            cache = FLASH_CACHE_DCACHE_ENABLED;
        } else {
            cache = FLASH_CACHE_DISABLED;
        }

        /* Program double-word (64-bit) at a specified address */
        program_double_word(address, data);
        uint32_t prog_bit = FLASH_CR_PG;

        /* Wait for last operation to be completed */
        error = wait_for_last_operation(FLASH_TIMEOUT_MS);

        /* If the program operation is completed, disable the PG or FSTPG Bit */
        if (prog_bit != 0U) {
            CLEAR_BIT(FLASH->CR, prog_bit);
        }

        /* Flush instruction cache  */
        if ((cache == FLASH_CACHE_ICACHE_ENABLED) || (cache == FLASH_CACHE_ICACHE_DCACHE_ENABLED)) {
            /* Disable instruction cache */
            FLASH_INSTRUCTION_CACHE_DISABLE();
            /* Reset instruction cache */
            FLASH_INSTRUCTION_CACHE_RESET();
            /* Enable instruction cache */
            FLASH_INSTRUCTION_CACHE_ENABLE();
        }

        /* Flush data cache */
        if ((cache == FLASH_CACHE_DCACHE_ENABLED) || (cache == FLASH_CACHE_ICACHE_DCACHE_ENABLED)) {
            /* Reset data cache */
            FLASH_DATA_CACHE_RESET();
            /* Enable data cache */
            FLASH_DATA_CACHE_ENABLE();
        }
    }

    /* return status */
    return error;
}

int flash_erase_pages(uint32_t page_start, uint32_t nb_page)
{
    int error;

    /* Wait for last operation to be completed */
    error = wait_for_last_operation(FLASH_TIMEOUT_MS);

    if (error == 0) {
        FLASH_CacheTypeDef cache;

        /* Deactivate the cache if they are activated to avoid data misbehavior */
        if (READ_BIT(FLASH->ACR, FLASH_ACR_ICEN) != 0U) {
            if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U) {
                /* Disable data cache  */
                FLASH_DATA_CACHE_DISABLE();
                cache = FLASH_CACHE_ICACHE_DCACHE_ENABLED;
            } else {
                cache = FLASH_CACHE_ICACHE_ENABLED;
            }
        } else if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U) {
            /* Disable data cache  */
            FLASH_DATA_CACHE_DISABLE();
            cache = FLASH_CACHE_DCACHE_ENABLED;
        } else {
            cache = FLASH_CACHE_DISABLED;
        }

        for (int page_index = page_start; page_index < (page_start + nb_page); page_index++) {
            page_erase(page_index);

            /* Wait for last operation to be completed */
            error = wait_for_last_operation(FLASH_TIMEOUT_MS);

            /* If the erase operation is completed, disable the PER Bit */
            CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));

            if (error != 0) {
                break;
            }
        }

        /* Flush instruction cache  */
        if ((cache == FLASH_CACHE_ICACHE_ENABLED) || (cache == FLASH_CACHE_ICACHE_DCACHE_ENABLED)) {
            /* Disable instruction cache */
            FLASH_INSTRUCTION_CACHE_DISABLE();
            /* Reset instruction cache */
            FLASH_INSTRUCTION_CACHE_RESET();
            /* Enable instruction cache */
            FLASH_INSTRUCTION_CACHE_ENABLE();
        }

        /* Flush data cache */
        if ((cache == FLASH_CACHE_DCACHE_ENABLED) || (cache == FLASH_CACHE_ICACHE_DCACHE_ENABLED)) {
            /* Reset data cache */
            FLASH_DATA_CACHE_RESET();
            /* Enable data cache */
            FLASH_DATA_CACHE_ENABLE();
        }
    }

    return error;
}

static void page_erase(uint32_t page)
{
    if (page < FLASH_PAGE_NB) {
        /* Proceed to erase the page */
        MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((page & 0xFFU) << FLASH_CR_PNB_Pos));
        SET_BIT(FLASH->CR, FLASH_CR_PER);
        SET_BIT(FLASH->CR, FLASH_CR_STRT);
    }
}

static void program_double_word(uint32_t address, uint64_t data)
{
    /* Set PG bit */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /* Program first word */
    *(uint32_t *) address = (uint32_t) data;

    /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
    __ISB();

    /* Program second word */
    *(uint32_t *) (address + 4U) = (uint32_t) (data >> 32U);
}

static int wait_for_last_operation(uint32_t timeout)
{
    int error;

    uint32_t tick_start = soc_get_tick();

    while (FLASH_GET_FLAG(FLASH_SR_BSY)) {
        if (soc_get_ms_since(tick_start) > timeout) {
            return -1;
        }
    }

    /* Check FLASH operation error flags */
    error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
    if (error != 0u) {
        /* Clear error programming flags */
        FLASH_CLEAR_FLAG(error);
        return error;
    }

    /* Check FLASH End of Operation flag  */
    if (FLASH_GET_FLAG(FLASH_FLAG_EOP)) {
        /* Clear FLASH End of Operation pending bit */
        FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    /* If there is an error flag set */
    return error;
}