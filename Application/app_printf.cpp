
#include "Application/app_printf.h"

#ifdef ENABLE_CAN_LOG
static bool g_app_printf_can_enabled = true;
#else
static bool g_app_printf_can_enabled = false;
#endif

#ifdef ENABLE_USB_LOG
static bool g_app_printf_usb_enabled = true;
#else
static bool g_app_printf_usb_enabled = false;
#endif

void app_printf_use_usb(bool use_usb) {
    g_app_printf_usb_enabled = use_usb;
}

bool app_printf_is_usb_enabled () {
    return g_app_printf_usb_enabled;
}

bool app_printf_is_enabled () {
    return g_app_printf_usb_enabled;
}