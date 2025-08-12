
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#ifdef HOST_HAL_MOCK
#include "host_mocks.h"
#else
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal.h"
#endif

#define MAX_QUEUE_SIZE 9

extern uint32_t start_time, ft8_time;

extern int QSO_xmit;

extern int slot_state;
extern int target_slot;
extern int target_freq;
extern int decode_flag;
extern int was_txing;
extern bool clr_pressed;
extern bool tx_pressed;
extern bool free_text;

extern const char *test_data_file;

void logger(const char *message, const char* file, int line);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
