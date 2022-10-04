#pragma once

#ifdef __cplusplus
extern "C" {
#endif
    void bt_console_init(void *ignore);
    void usb_console_init(void *ignore);
    void tast_bt_console(void *ignore);
    void tast_usb_console(void *ignore);
#ifdef __cplusplus
}
#endif