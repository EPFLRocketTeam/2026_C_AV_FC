//    FILE: bootloader.h
// PURPOSE: Reboot the board, or reboot it into the STM32H743 ROM DFU
//          bootloader so it can be flashed over USB without an ST-Link.
//          C linkage so Core/Src/main.c can call it.
//
//          Bootloader_RequestDfu() leaves a magic word in RTC backup
//          register 0 and resets. Bootloader_CheckAndJump(), the first
//          statement of main(), finds the word, clears it and jumps to the
//          ROM bootloader before HAL_Init() touches any peripheral, so the
//          bootloader starts from an almost reset state. See bootloader.cpp
//          for the sources behind the addresses.

#ifndef APP_SYSTEM_BOOTLOADER_H
#define APP_SYSTEM_BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

// Arms DFU entry for the next boot and resets the MCU. Never returns.
void Bootloader_RequestDfu(void);

// Call as the very first statement of main(), before HAL_Init() and
// SystemClock_Config(). Returns at once on a normal boot; jumps to the ROM
// bootloader and never returns when Bootloader_RequestDfu() armed it.
void Bootloader_CheckAndJump(void);

// Plain reset into the application. Never returns.
void Bootloader_Reboot(void);

#ifdef __cplusplus
}
#endif

#endif  // APP_SYSTEM_BOOTLOADER_H
