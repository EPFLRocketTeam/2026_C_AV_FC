/**
 * @file sd_hardware_init.c
 * @brief SDMMC1 hardware initialization for SD card access.
 *
 * This file is tracked by git (unlike stm32h7xx_hal_msp.c / stm32h7xx_it.c
 * which are CubeMX-generated and gitignored).
 *
 * Call sd_pre_init() from main.c BEFORE MX_SDMMC1_SD_Init().
 * This configures PLL2 (200 MHz), SDMMC1 peripheral clock, GPIO pins,
 * and NVIC for DMA interrupts — everything that HAL_SD_MspInit would
 * normally do, but in a reproducible, version-controlled file.
 *
 * After HAL_SD_Init completes, call sd_post_init() to switch to
 * High Speed mode (SDR25, 50 MHz).
 */

#include "main.h"
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Pre-init: clock, GPIO, NVIC — call BEFORE MX_SDMMC1_SD_Init()     */
/* ------------------------------------------------------------------ */
void sd_pre_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* ---- PLL2 → 200 MHz kernel clock for SDMMC1 ---- */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
    PeriphClkInitStruct.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL2;
    PeriphClkInitStruct.PLL2.PLL2M = 32;     /* HSI 64 MHz / 32 = 2 MHz VCO input */
    PeriphClkInitStruct.PLL2.PLL2N = 200;    /* 2 MHz × 200 = 400 MHz VCO */
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;      /* 400 / 2 = 200 MHz */
    PeriphClkInitStruct.PLL2.PLL2RGE   = RCC_PLL2VCIRANGE_0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN  = 0;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        /* Non-fatal: HAL_SD_Init will fail gracefully */
    }

    /* ---- SDMMC1 peripheral clock ---- */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    /* ---- GPIO ---- */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*  PC8  = D0   PC9  = D1   PC10 = D2   PC11 = D3   (AF12, pull-up)
     *  PC12 = CK   (AF12, no pull)
     *  PD2  = CMD  (AF12, pull-up)
     */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_12;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ---- NVIC for DMA / interrupt-driven writes ---- */
    /* Priority 1 (was 2): reduce ISR preemption that causes 100ms+ DMA
     * transfer outliers.  Must remain below SPI DMA priority (0). */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

/* ------------------------------------------------------------------ */
/* Post-init: HS mode switch — call AFTER successful HAL_SD_Init()    */
/* ------------------------------------------------------------------ */
void sd_post_init(SD_HandleTypeDef *hsd)
{
    if (hsd->State != HAL_SD_STATE_READY) return;

    /* Switch card to High Speed (SDR25, up to 50 MHz) */
    if (HAL_SD_ConfigSpeedBusOperation(hsd, SDMMC_SPEED_MODE_HIGH) == HAL_OK)
    {
        /* PLL2R = 200 MHz, ClockDiv = 2 → SDMMC_CK = 200/(2×2) = 50 MHz */
        MODIFY_REG(hsd->Instance->CLKCR, SDMMC_CLKCR_CLKDIV, 2U);
        hsd->Init.ClockDiv = 2;
    }
}

/* ------------------------------------------------------------------ */
/* SDMMC1 IRQ handler — needed for HAL_SD_WriteBlocks_DMA()           */
/* Overrides the weak Default_Handler from startup_stm32h743zitx.s    */
/* ------------------------------------------------------------------ */
extern SD_HandleTypeDef hsd1;   /* defined in main.c */

void SDMMC1_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd1);
}
