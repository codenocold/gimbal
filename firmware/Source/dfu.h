#ifndef __DFU_H__
#define __DFU_H__

#include "main.h"

int  dfu_start(void);
int  dfu_data(uint8_t *data);
int  dfu_end(void);
int  dfu_app_check(uint32_t app_size, uint32_t app_crc);
void dfu_app_run(void);

#endif
