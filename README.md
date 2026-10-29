# 2026_C_AV_FC

Firehorn II flight computer firmware (STM32H743ZI).

## Flashing over USB (DFU)

The FC flashes over USB without an ST-Link, through the STM32H743 ROM bootloader. DFU is the ROM bootloader only: the application starts no DFU device of its own, so the OTG_FS core stays idle until the bootloader claims it.

The board has two USB ports. The VCP port (OTG_HS in full speed mode, PB14/PB15) is the serial console; it enumerates as "ERT Firehorn II FC" with the chip's 96 bit unique id as its serial number, so each board keeps a stable port name. The DFU port is the dedicated mini USB connector on OTG_FS (PA11/PA12), the only USB port the ROM bootloader listens on.

1. Enter the bootloader. On the VCP port, type `system dfu`. The FC only accepts it in INIT, CALIBRATION, FILLING, LANDED or ABORT_ON_GROUND and prints the reason when it refuses; its replies show after `logs usb fc on`. Fallback: set the BOOT0 switch high, press NRST (the green LED shows BOOT0), and set BOOT0 back low once flashed.
2. Plug the DFU port. `dfu-util -l` lists the bootloader as `0483:df11`.
3. Flash a raw binary (`arm-none-eabi-objcopy -O binary firmware.elf firmware.bin` turns the CubeIDE output into one):

   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
   ```

   The STM32CubeProgrammer equivalent:

   ```bash
   STM32_Programmer_CLI -c port=USB1 -w firmware.bin 0x08000000 -v --start 0x08000000
   ```

   On Windows, dfu-util needs the WinUSB driver bound to the DFU device (Zadig); STM32CubeProgrammer ships ST's own driver.

What to expect:

* The ROM bootloader only runs at reset. `system dfu` stores a magic word in RTC backup register 0 and resets; the first statement of `main()` sees it and jumps to the bootloader before the HAL starts (`Application/System/bootloader.cpp`).
* The word is cleared before the jump, so a power cycle or a reset out of the bootloader never enters DFU again.
* After flashing, `:leave` (or `--start`) exits the bootloader and the board reboots into the new image. With the BOOT0 fallback, every reset enters the bootloader until BOOT0 is back low.
* `system reboot` on the VCP port resets into the application.

What the ROM bootloader needs (ST AN2606 Rev 61, section 52 "STM32H74xxx/75xxx devices bootloader"):

* Entry point: "The bootloader start address is 0x1FF09800" (section 52.1, Table 113). The jump loads the initial stack pointer from 0x1FF09800 and the reset handler from 0x1FF09804.
* USB: DFU runs on "USB FS configured in forced device mode" on PA11 (DM) and PA12 (DP), and "No external pull-up resistor is required" (Table 113).
* Clock: the bootloader runs at 64 MHz on HSI and uses the CRS to clock USB from HSI48 (Table 113), so no crystal is involved.
* Supply: "DFU mode does not support USBREGEN mode. If STM32 is powered by 1.8 V source, it is not possible to use the BL DFU unless 3.3 V is provided" (Table 113).
* Version: the bootloader ID byte at 0x1FF1E7FE is 0x91 for V9.1 (Table 177); `system dfu` prints it. A part reading 0x90 has V9.0, whose known limitations (Table 114) are a jump issue unless the application stack pointer is below RAM end minus 16 bytes (the jump here hands over the bootloader's own initial stack pointer, which avoids it) and an extra reset needed after a power cycle before the bootloader answers (ST's workaround is a pull-up on PA11). V9.1 fixes both.
* After its own reset the bootloader disables every interrupt source and peripheral clock and initialises its clock, GPIOs, IWDG and SysTick itself (Figure 64), so it starts cleanly from the reset state the jump hands over. It refreshes the IWDG it configures (Table 113).

CubeMX note: `USB_DEVICE/App/usb_device.c` starts no DFU class on OTG_FS, which the `.ioc` does not describe yet. Regenerating code from the current `.ioc` brings the DFU stub back on the DFU port; update the `.ioc` first.
