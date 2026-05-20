#ifndef SD_HARDWARE_INIT_H
#define SD_HARDWARE_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Configure PLL2, GPIO, SDMMC1 clock, and NVIC.
 *        Call BEFORE MX_SDMMC1_SD_Init().
 */
void sd_pre_init(void);

/**
 * @brief Switch to High Speed mode (50 MHz SDMMC clock).
 *        Call AFTER successful HAL_SD_Init().
 */
void sd_post_init(SD_HandleTypeDef *hsd);

#ifdef __cplusplus
}
#endif

#endif /* SD_HARDWARE_INIT_H */
