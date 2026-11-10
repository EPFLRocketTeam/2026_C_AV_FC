//    FILE: bootloader.cpp
// PURPOSE: Software entry into the STM32H743 ROM DFU bootloader.
//
// Sources:
//  * ST AN2606 Rev 61 (January 2024), section 52 "STM32H74xxx/75xxx
//    devices bootloader":
//    - Section 52.1, Table 113: "The bootloader start address is
//      0x1FF09800." The jump takes the initial MSP from [0x1FF09800] and
//      the reset handler from [0x1FF09804].
//    - Table 113, DFU: "USB FS configured in forced device mode", USB_DM
//      PA11, USB_DP PA12, "No external pull-up resistor is required."
//    - Table 113, RCC: the bootloader runs at 64 MHz on HSI and enables the
//      CRS so USB runs on HSI48; no HSE crystal is needed.
//    - Table 114, V9.0 (0x90) limitation: "Application stack pointer must
//      be lower than (RAM end @ - 16 bytes)". The jump below loads the
//      bootloader's own initial MSP, so the application stack never
//      reaches it. V9.1 (0x91) fixes the V9.0 limitations.
//    - Table 177: bootloader ID 0x91 at 0x1FF1E7FE (kBootloaderIdAddress).
//    - Figure 64: after its reset the bootloader disables all interrupt
//      sources and peripheral clocks and runs its own System Init (clock,
//      GPIOs, IWDG, SysTick), which is why jumping from the reset state at
//      the top of main() is enough.
//  * RM0433 (STM32H742/743/750/753 reference manual):
//    - RTC chapter, "RTC backup registers": RTC_BKPxR sit in the backup
//      domain, survive a system reset (NVIC_SystemReset), are reset only by
//      a backup domain reset or a tamper event, and are accessed through the
//      RTC APB interface, so RCC_APB4ENR.RTCAPBEN (__HAL_RCC_RTC_CLK_ENABLE
//      in this HAL) must be set to read them.
//      The RTC kernel clock (RCC_BDCR.RTCEN) does not need to run.
//      Per the HAL, reading and writing RTC_BKP0R needs only RTCAPBEN
//      (__HAL_RCC_RTC_CLK_ENABLE) plus PWR_CR1.DBP; both .ioc files leave
//      the RTC and its kernel clock source (RCC_BDCR.RTCSEL) unselected.
//      Not yet confirmed on a board: if `system dfu` does not bring up
//      0483:df11, select LSI as the RTC clock source in the .ioc (README,
//      bench checklist).
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
constexpr uint32_t kBootloaderIdAddress = 0x1FF1E7FEu;

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

extern "C" uint8_t Bootloader_RomId(void) {
  return *reinterpret_cast<volatile const uint8_t*>(kBootloaderIdAddress);
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
