//    FILE: bootloader.cpp
// PURPOSE: Software entry into the STM32H743 ROM DFU bootloader.
//
// Sources:
//  * ST AN2606 "STM32 microcontroller system memory boot mode", the
//    STM32H74xxx/75xxx bootloader section: DFU runs on USB OTG_FS only
//    (PA11 DM, PA12 DP), and the bootloader only starts from a reset.
//    The entry address 0x1FF09800 (initial MSP at +0, reset vector at +4)
//    is the value ST's community article "How to jump to system bootloader
//    from application code on STM32 microcontrollers" lists for the
//    STM32H743/753; the AN2606 table itself could not be read when this was
//    written, so check it against the current revision before trusting a
//    new silicon revision.
//  * RM0433 (STM32H742/743/750/753 reference manual):
//    - RTC chapter, "RTC backup registers": RTC_BKPxR sit in the backup
//      domain, survive a system reset (NVIC_SystemReset), are reset only by
//      a backup domain reset or a tamper event, and are accessed through the
//      RTC APB interface, so RCC_APB4ENR.RTCAPBEN (__HAL_RCC_RTC_CLK_ENABLE
//      in this HAL) must be set to read them.
//      The RTC kernel clock (RCC_BDCR.RTCEN) does not need to run.
//    - PWR chapter, PWR_CR1.DBP: backup domain write protection; set it
//      before writing RTC_BKPxR.
//    - RCC chapter: after reset the core runs from HSI and no peripheral
//      clock is enabled, which is the state the jump below hands over.
//  * system_stm32h7xx.c here leaves USER_VECT_TAB_ADDRESS undefined, so
//    neither SystemInit nor the HAL writes SCB->VTOR; it is still the boot
//    address (flash). The jump sets VTOR to the ROM table it enters, as a
//    hardware boot into that table would.
//
// Bootloader_CheckAndJump runs before MPU_Config, cache enable, HAL_Init and
// SystemClock_Config, so the MPU is off and the caches are normally off
// already; the cleanup below is defensive.

#include "bootloader.h"

#include "stm32h7xx_hal.h"

namespace {

constexpr uint32_t kDfuMagic = 0xDF0B007u;
constexpr uint32_t kRomBootloader = 0x1FF09800u;

void OpenBackupDomain() {
  __HAL_RCC_RTC_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  for (uint32_t i = 0; i < 100000u && (PWR->CR1 & PWR_CR1_DBP) == 0u; ++i) {
  }
}

}  // namespace

extern "C" void Bootloader_RequestDfu(void) {
  OpenBackupDomain();
  RTC->BKP0R = kDfuMagic;
  __DSB();
  NVIC_SystemReset();
}

extern "C" void Bootloader_Reboot(void) {
  NVIC_SystemReset();
}

extern "C" void Bootloader_CheckAndJump(void) {
  __HAL_RCC_RTC_CLK_ENABLE();
  if (RTC->BKP0R != kDfuMagic) {
    __HAL_RCC_RTC_CLK_DISABLE();
    return;
  }

  // Clear first, so a power cycle or a reset out of the bootloader boots
  // the application instead of re-entering DFU.
  OpenBackupDomain();
  RTC->BKP0R = 0u;
  __DSB();
  HAL_PWR_DisableBkUpAccess();
  __HAL_RCC_RTC_CLK_DISABLE();

  __disable_irq();

  SysTick->CTRL = 0u;
  SysTick->LOAD = 0u;
  SysTick->VAL = 0u;

  for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFu;
    NVIC->ICPR[i] = 0xFFFFFFFFu;
  }

  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u) {
    SCB_DisableDCache();
  }
  if ((SCB->CCR & SCB_CCR_IC_Msk) != 0u) {
    SCB_DisableICache();
  }

  SCB->VTOR = kRomBootloader;
  __DSB();
  __ISB();

  // Every NVIC line is disabled and SysTick is off, so nothing can fire;
  // the ROM bootloader expects PRIMASK clear, as after a hardware reset.
  __enable_irq();

  const uint32_t initial_sp = *reinterpret_cast<volatile uint32_t*>(kRomBootloader);
  void (*const rom_reset)(void) =
      reinterpret_cast<void (*)(void)>(*reinterpret_cast<volatile uint32_t*>(kRomBootloader + 4u));
  __set_MSP(initial_sp);
  rom_reset();

  for (;;) {
  }
}
