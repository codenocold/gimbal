/* Includes ------------------------------------------------------------------*/
#include "eeprom_emul.h"
#include "flash_interface.h"
#include "flash.h"

/**
  * @brief  Write a double word at the given address in Flash
  * @param  Address Where to write
  * @param  Data What to write
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE_WRITE_ERROR: if an error occurs
  */
HAL_StatusTypeDef FI_WriteDoubleWord(uint32_t Address, uint64_t Data)
{
    int ret;

    flash_unlock();
    ret = flash_program(Address, Data);
    flash_lock();

    return ret == 0 ? EE_OK : EE_WRITE_ERROR;
}

/**
  * @brief  Erase a page in polling mode
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_PageErase(uint32_t Page, uint16_t NbPages)
{
    int ret;

    flash_unlock();
    ret = flash_erase_pages(Page, NbPages);
    flash_lock();

    return ret == 0 ? EE_OK : EE_ERASE_ERROR;
}

/**
  * @brief  Erase a page with interrupt enabled
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_PageErase_IT(uint32_t Page, uint16_t NbPages)
{
    return EE_OK;
}

/**
  * @brief  Delete corrupted Flash address, can be called from NMI. No Timeout.
  * @param  Address Address of the FLASH Memory to delete
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_DeleteCorruptedFlashAddress(uint32_t Address)
{
    uint32_t  dcachetoreactivate = 0U;
    EE_Status status             = EE_OK;

    /* Deactivate the data cache if they are activated to avoid data misbehavior */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != RESET) {
        /* Disable data cache  */
        __HAL_FLASH_DATA_CACHE_DISABLE();
        dcachetoreactivate = 1U;
    }

    /* Set FLASH Programmation bit */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /* Program double word of value 0 */
    *(__IO uint32_t *) (Address)      = (uint32_t) 0U;
    *(__IO uint32_t *) (Address + 4U) = (uint32_t) 0U;

    /* Wait programmation completion */
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
    }

    /* Check if error occured */
    if ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR))
        || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_SIZERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR))) {
        status = EE_DELETE_ERROR;
    }

    /* Check FLASH End of Operation flag  */
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) {
        /* Clear FLASH End of Operation pending bit */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    /* Clear FLASH Programmation bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

    /* Flush the caches to be sure of the data consistency */
    if (dcachetoreactivate == 1U) {
        /* Reset data cache */
        __HAL_FLASH_DATA_CACHE_RESET();
        /* Enable data cache */
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }

    /* Clear FLASH ECCD bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);

    return status;
}

/**
  * @brief  Check if the configuration is 128-bits bank or 2*64-bits bank
  * @param  None
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_CheckBankConfig(void)
{
    return EE_OK;
}

/**
  * @brief  Flush the caches if needed to keep coherency when the flash content is modified
  */
void FI_CacheFlush()
{
    /* To keep its coherency, flush the D-Cache: its content is not updated after a flash erase. */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    __HAL_FLASH_DATA_CACHE_ENABLE();
}