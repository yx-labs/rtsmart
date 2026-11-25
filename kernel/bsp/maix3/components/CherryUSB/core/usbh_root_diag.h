/* SPDX-License-Identifier: Apache-2.0 */
#ifndef USBH_ROOT_DIAG_H
#define USBH_ROOT_DIAG_H

#include <stdint.h>

/* The CanMV UAC1 board has one dedicated host controller and one root port.
 * Stage names are static literals, never descriptors supplied by the device. */
enum usbh_root_event {
    ROOT_INIT, ROOT_WORKER, ROOT_NOTIFY, ROOT_DISPATCH, ROOT_PORT_STATUS,
    ROOT_RESET, ROOT_ENUM_TRY, ROOT_PHASE, ROOT_ERROR, ROOT_CONFIGURED,
    ROOT_RELEASE, ROOT_UAC
};
enum usbh_startup_policy { ROOT_BOOT_BASELINE, ROOT_BOOT_SETTLE, ROOT_BOOT_LONG, ROOT_BOOT_EARLY, ROOT_BOOT_EARLY30 };
enum usbh_root_mark { ROOT_MARK_PRE, ROOT_MARK_ASSERT, ROOT_MARK_CLEAR, ROOT_MARK_POST, ROOT_MARK_DISCONNECT,
    ROOT_MARK_SETUP, ROOT_MARK_SETUP_DONE, ROOT_MARK_COUNT };
enum usbh_frame_mark { ROOT_FRAME_CLEAR, ROOT_FRAME_SETUP, ROOT_FRAME_IRQ, ROOT_FRAME_COUNT };
#ifdef CHERRY_USB_HOST_ENABLE_CLASS_UAC1_PCM
void usbh_root_note(enum usbh_root_event event, const char *stage, int value, uint32_t detail);
void usbh_root_hw(int mode, uint32_t hprt, uint32_t sw_flags, uint32_t power_flags);
void usbh_root_load_policy(void);
int usbh_root_begin_trial(void);
int usbh_root_active_policy(void);
void usbh_root_mark(enum usbh_root_mark mark, int mode, uint32_t hprt);
void usbh_root_end_trial(void);
int usbh_root_trace_wanted(enum usbh_root_mark mark);
void usbh_root_frames(enum usbh_frame_mark mark, int mode, uint32_t hfnum, uint32_t hfir, uint32_t hcfg);
void usbh_root_setup_result(int rc);
#ifdef CHERRY_USB_HC_DRV_DWC2
struct usbh_bus;
void dwc2_trace_first_setup(struct usbh_bus *bus, int done, int result);
#endif
#define USBH_ROOT_PORT_NOTE(p, ...) do { if ((p)->parent && (p)->parent->is_roothub) usbh_root_note(__VA_ARGS__); } while (0)
#else
#define usbh_root_note(...) ((void)0)
#define usbh_root_hw(...) ((void)0)
#define usbh_root_load_policy() ((void)0)
#define usbh_root_begin_trial() (-1)
#define usbh_root_active_policy() (-1)
#define usbh_root_mark(...) ((void)0)
#define usbh_root_end_trial() ((void)0)
#define USBH_ROOT_PORT_NOTE(...) ((void)0)
#endif
#endif
