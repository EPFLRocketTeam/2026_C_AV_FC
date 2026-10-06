
#pragma once
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_USB_LOG

void app_printf_use_usb(bool use_usb);
bool app_printf_is_usb_enabled ();

bool app_printf_is_enabled ();

#ifdef __cplusplus
}
#endif

#define app_printf(...) do { \
    if (app_printf_is_enabled()) { \
        printf(__VA_ARGS__); \
    } \
} while (0)
