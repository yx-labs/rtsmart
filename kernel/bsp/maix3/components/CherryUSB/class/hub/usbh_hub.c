/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbh_core.h"
#include "usbh_hub.h"
#include "usbh_root_diag.h"

#undef USB_DBG_TAG
#define USB_DBG_TAG "usbh_hub"
#include "usb_log.h"

#define DEV_FORMAT "/dev/hub%d"

#define HUB_DEBOUNCE_TIMEOUT   1500
#define HUB_DEBOUNCE_STEP      25
#define HUB_DEBOUNCE_STABLE    100
#define DELAY_TIME_AFTER_RESET 200
#define DELAY_TIME_BEFORE_ENUMERATE 20
#define DELAY_TIME_ENUM_NOTCONN_BACKOFF 100
#define HUB_ENUM_ATTEMPTS      3
#define ROOT_RECHECK_MS        1000
#define ROOT_RECHECK_ATTEMPTS  2

#define EXTHUB_FIRST_INDEX 2

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_hub_buf[CONFIG_USBHOST_MAX_BUS][USB_ALIGN_UP(32, CONFIG_USB_ALIGN_SIZE)];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_hub_intbuf[CONFIG_USBHOST_MAX_BUS][CONFIG_USBHOST_MAX_EXTHUBS + 1][USB_ALIGN_UP(1, CONFIG_USB_ALIGN_SIZE)];

extern int usbh_free_devaddr(struct usbh_hubport *hport);
extern int usbh_enumerate(struct usbh_hubport *hport);

static const char *speed_table[] = { "error-speed", "low-speed", "full-speed", "high-speed", "wireless-speed", "super-speed", "superplus-speed" };

#ifdef CONFIG_USBHOST_XHCI
struct usbh_hubport *usbh_get_roothub_port(unsigned int port)
{
    return &roothub.child[port - 1];
}
#endif

#if CONFIG_USBHOST_MAX_EXTHUBS > 0
static struct usbh_hub g_hub_class[CONFIG_USBHOST_MAX_EXTHUBS];
static uint32_t g_devinuse = 0;

static struct usbh_hub *usbh_hub_class_alloc(void)
{
    int devno;

    for (devno = 0; devno < CONFIG_USBHOST_MAX_EXTHUBS; devno++) {
        if ((g_devinuse & (1 << devno)) == 0) {
            g_devinuse |= (1 << devno);
            usb_memset(&g_hub_class[devno], 0, sizeof(struct usbh_hub));
            g_hub_class[devno].index = EXTHUB_FIRST_INDEX + devno;
            return &g_hub_class[devno];
        }
    }
    return NULL;
}

static void usbh_hub_class_free(struct usbh_hub *hub_class)
{
    int devno = hub_class->index - EXTHUB_FIRST_INDEX;

    if (devno >= 0 && devno < 32) {
        g_devinuse &= ~(1 << devno);
    }
    usb_memset(hub_class, 0, sizeof(struct usbh_hub));
}
#endif

#if CONFIG_USBHOST_MAX_EXTHUBS > 0
static int _usbh_hub_get_hub_descriptor(struct usbh_hub *hub, uint8_t *buffer)
{
    struct usb_setup_packet *setup;
    int ret;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_DEVICE;
    setup->bRequest = USB_REQUEST_GET_DESCRIPTOR;

    /* TODO: hub descriptor has some difference between USB 2.0 and USB 3.x,
       and we havn't handle the difference here */
    if ((hub->parent->speed == USB_SPEED_SUPER) ||
        (hub->parent->speed == USB_SPEED_SUPER_PLUS)) {
        setup->wValue = HUB_DESCRIPTOR_TYPE_HUB3 << 8;
    } else {
        setup->wValue = HUB_DESCRIPTOR_TYPE_HUB << 8;
    }

    setup->wIndex = 0;
    setup->wLength = USB_SIZEOF_HUB_DESC;

    ret = usbh_control_transfer(hub->parent, setup, g_hub_buf[hub->bus->busid]);
    if (ret < 0) {
        return ret;
    }
    usb_memcpy(buffer, g_hub_buf, USB_SIZEOF_HUB_DESC);
    return ret;
}
#if 0
static int _usbh_hub_get_status(struct usbh_hub *hub, uint8_t *buffer)
{
    struct usb_setup_packet *setup;
    int ret;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_DEVICE;
    setup->bRequest = HUB_REQUEST_GET_STATUS;
    setup->wValue = 0;
    setup->wIndex = 0;
    setup->wLength = 2;

    ret = usbh_control_transfer(hub->parent, setup, g_hub_buf);
    if (ret < 0) {
        return ret;
    }
    usb_memcpy(buffer, g_hub_buf, 2);
    return ret;
}
#endif
#endif

static int _usbh_hub_get_portstatus(struct usbh_hub *hub, uint8_t port, struct hub_port_status *port_status)
{
    struct usb_setup_packet *setup;
    int ret;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
    setup->bRequest = HUB_REQUEST_GET_STATUS;
    setup->wValue = 0;
    setup->wIndex = port;
    setup->wLength = 4;

    ret = usbh_control_transfer(hub->parent, setup, g_hub_buf[hub->bus->busid]);
    if (ret < 0) {
        return ret;
    }
    usb_memcpy(port_status, g_hub_buf[hub->bus->busid], 4);
    return ret;
}

static int _usbh_hub_set_feature(struct usbh_hub *hub, uint8_t port, uint8_t feature)
{
    struct usb_setup_packet *setup;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
    setup->bRequest = HUB_REQUEST_SET_FEATURE;
    setup->wValue = feature;
    setup->wIndex = port;
    setup->wLength = 0;

    return usbh_control_transfer(hub->parent, setup, NULL);
}

static int _usbh_hub_clear_feature(struct usbh_hub *hub, uint8_t port, uint8_t feature)
{
    struct usb_setup_packet *setup;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
    setup->bRequest = HUB_REQUEST_CLEAR_FEATURE;
    setup->wValue = feature;
    setup->wIndex = port;
    setup->wLength = 0;

    return usbh_control_transfer(hub->parent, setup, NULL);
}

static int _usbh_hub_set_depth(struct usbh_hub *hub, uint16_t depth)
{
    struct usb_setup_packet *setup;

    setup = hub->parent->setup;

    setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_DEVICE;
    setup->bRequest = HUB_REQUEST_SET_HUB_DEPTH;
    setup->wValue = depth;
    setup->wIndex = 0;
    setup->wLength = 0;

    return usbh_control_transfer(hub->parent, setup, NULL);
}

#if CONFIG_USBHOST_MAX_EXTHUBS > 0
static int parse_hub_descriptor(struct usb_hub_descriptor *desc, uint16_t length)
{
    if (desc->bLength != USB_SIZEOF_HUB_DESC) {
        USB_LOG_ERR("invalid device bLength 0x%02x\r\n", desc->bLength);
        return -1;
    } else if (desc->bDescriptorType != HUB_DESCRIPTOR_TYPE_HUB) {
        USB_LOG_ERR("unexpected descriptor 0x%02x\r\n", desc->bDescriptorType);
        return -2;
    } else {
        USB_LOG_RAW("Hub Descriptor:\r\n");
        USB_LOG_RAW("bLength: 0x%02x             \r\n", desc->bLength);
        USB_LOG_RAW("bDescriptorType: 0x%02x     \r\n", desc->bDescriptorType);
        USB_LOG_RAW("bNbrPorts: 0x%02x           \r\n", desc->bNbrPorts);
        USB_LOG_RAW("wHubCharacteristics: 0x%04x \r\n", desc->wHubCharacteristics);
        USB_LOG_RAW("bPwrOn2PwrGood: 0x%02x      \r\n", desc->bPwrOn2PwrGood);
        USB_LOG_RAW("bHubContrCurrent: 0x%02x    \r\n", desc->bHubContrCurrent);
        USB_LOG_RAW("DeviceRemovable: 0x%02x     \r\n", desc->DeviceRemovable);
        USB_LOG_RAW("PortPwrCtrlMask: 0x%02x     \r\n", desc->PortPwrCtrlMask);
    }
    return 0;
}
#endif

static int usbh_hub_get_portstatus(struct usbh_hub *hub, uint8_t port, struct hub_port_status *port_status)
{
    struct usb_setup_packet roothub_setup;
    struct usb_setup_packet *setup;

    if (hub->is_roothub) {
        setup = &roothub_setup;
        setup->bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
        setup->bRequest = HUB_REQUEST_GET_STATUS;
        setup->wValue = 0;
        setup->wIndex = port;
        setup->wLength = 4;
        return usbh_roothub_control(hub->bus, &roothub_setup, (uint8_t *)port_status);
    } else {
        return _usbh_hub_get_portstatus(hub, port, port_status);
    }
}

int usbh_hub_set_feature(struct usbh_hub *hub, uint8_t port, uint8_t feature)
{
    struct usb_setup_packet roothub_setup;
    struct usb_setup_packet *setup;

    if (hub->is_roothub) {
        setup = &roothub_setup;
        setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
        setup->bRequest = HUB_REQUEST_SET_FEATURE;
        setup->wValue = feature;
        setup->wIndex = port;
        setup->wLength = 0;
        return usbh_roothub_control(hub->bus, setup, NULL);
    } else {
        return _usbh_hub_set_feature(hub, port, feature);
    }
}

int usbh_hub_clear_feature(struct usbh_hub *hub, uint8_t port, uint8_t feature)
{
    struct usb_setup_packet roothub_setup;
    struct usb_setup_packet *setup;

    if (hub->is_roothub) {
        setup = &roothub_setup;
        setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_OTHER;
        setup->bRequest = HUB_REQUEST_CLEAR_FEATURE;
        setup->wValue = feature;
        setup->wIndex = port;
        setup->wLength = 0;
        return usbh_roothub_control(hub->bus, setup, NULL);
    } else {
        return _usbh_hub_clear_feature(hub, port, feature);
    }
}

static int usbh_hub_set_depth(struct usbh_hub *hub, uint16_t depth)
{
    struct usb_setup_packet roothub_setup;
    struct usb_setup_packet *setup;

    if (hub->is_roothub) {
        setup = &roothub_setup;
        setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_DEVICE;
        setup->bRequest = HUB_REQUEST_SET_HUB_DEPTH;
        setup->wValue = depth;
        setup->wIndex = 0;
        setup->wLength = 0;
        return usbh_roothub_control(hub->bus, setup, NULL);
    } else {
        return _usbh_hub_set_depth(hub, depth);
    }
}

static void usbh_hubport_release(struct usbh_hubport *child)
{
    if (child->connected) {
        USBH_ROOT_PORT_NOTE(child, ROOT_RELEASE, NULL, 0, 0);
        child->connected = false;
        usbh_free_devaddr(child);
        for (uint8_t i = 0; i < CONFIG_USBHOST_MAX_INTERFACES; i++) {
            if (!usbh_intf_slot_valid(child, i)) {
                continue;
            }

            if (child->config.intf[i].class_driver && child->config.intf[i].class_driver->disconnect) {
                CLASS_DISCONNECT(child, i);
            }
        }
        child->config.config_desc.bNumInterfaces = 0;
        usbh_kill_urb(&child->ep0_urb);

#ifdef CHERRY_USB_HC_DRV_DWC2
extern int dwc2_hcd_endpoint_disable(struct usbh_hcd *hcd, struct usb_host_endpoint *ep, int retry);

        if (child->bus != NULL) {
            struct usbh_hcd *hcd = &child->bus->hcd;
            for (int i = 0; i < 16; i++) {
                dwc2_hcd_endpoint_disable(hcd, &child->hep_in[i], 250);
                dwc2_hcd_endpoint_disable(hcd, &child->hep_out[i], 250);
            }
        }
#endif
        usb_osal_mutex_delete(child->mutex);
    }
}

#if CONFIG_USBHOST_MAX_EXTHUBS > 0
static void hub_int_complete_callback(void *arg, int nbytes)
{
    struct usbh_hub *hub = (struct usbh_hub *)arg;

    if (nbytes > 0) {
        USB_LOG_DBG("wake hub\n");
        usbh_hub_thread_wakeup(hub);
    }
#ifndef CHERRY_USB_HC_DRV_DWC2
    else if (nbytes == -USB_ERR_NAK) {
        /* Restart timer to submit urb again */
        USB_LOG_DBG("Restart timer\r\n");
        usb_osal_timer_start(hub->int_timer);
    } else if (nbytes == -USB_ERR_DT) {
        usbh_submit_urb(&hub->intin_urb);
    }
#else
    else if (nbytes == -ECONNRESET){
        USB_LOG_DBG("hub disconnect\r\n");
    } else {
        USB_LOG_ERR("%s: %d", __func__, nbytes);
    }
    usbh_submit_urb(&hub->intin_urb);
#endif
}

static void hub_int_timeout(void *arg)
{
    struct usbh_hub *hub = (struct usbh_hub *)arg;

    usbh_int_urb_fill(&hub->intin_urb, hub->parent, hub->intin, hub->int_buffer, 1, 0, hub_int_complete_callback, hub);
    usbh_submit_urb(&hub->intin_urb);
}

#ifdef CHERRY_USB_HC_DRV_DWC2
extern void hub_tt_work(struct rt_work* work, void* work_data);
#endif

static int usbh_hub_connect(struct usbh_hubport *hport, uint8_t intf)
{
    struct usb_endpoint_descriptor *ep_desc;
    struct hub_port_status port_status;
    int ret;

    struct usbh_hub *hub = usbh_hub_class_alloc();
    if (hub == NULL) {
        USB_LOG_ERR("Fail to alloc hub_class\r\n");
        return -USB_ERR_NOMEM;
    }

    hub->hub_addr = hport->dev_addr;
    hub->parent = hport;
    hub->bus = hport->bus;

#ifdef CHERRY_USB_HC_DRV_DWC2
    uint16_t wHubCharacteristics;

    rt_spin_lock_init(&hub->tt.lock);
    usb_dlist_init(&hub->tt.clear_list);
    rt_work_init(&hub->tt.clear_work, hub_tt_work, hub);

    switch (hport->device_desc.bDeviceProtocol) {
    case USB_HUB_PR_FS:
        USB_LOG_ERR("Currently Don't Support Full Speed Hubs\n");
        return -1;
        break;
    case USB_HUB_PR_HS_SINGLE_TT:
        USB_LOG_INFO("Single TT\n");
        hub->tt.hub = hport;
        break;
    case USB_HUB_PR_HS_MULTI_TT:
        ret = usbh_set_interface(hport, 0, 1);
        if (ret > 0) {
            USB_LOG_INFO("TT per port\n");
            hub->tt.multi = 1;
        } else
            USB_LOG_ERR("Using single TT (err %d)\n", ret);
        hub->tt.hub = hport;
        break;
    case USB_HUB_PR_SS:
        /* USB 3.0 hubs don't have a TT */
        break;
    default:
        USB_LOG_INFO("Unrecognized hub protocol %d\n",
                     hport->device_desc.bDeviceProtocol);
        break;
    }

    wHubCharacteristics = hub->hub_desc.wHubCharacteristics;

    /* Note 8 FS bit times == (8 bits / 12000000 bps) ~= 666ns */
    switch (wHubCharacteristics & HUB_CHAR_TTTT_MASK) {
    case HUB_CHAR_TTTT_8_BITS:
        if (hport->device_desc.bDeviceProtocol != 0) {
            hub->tt.think_time = 666;
            USB_LOG_DBG("TT requires at most %d "
                        "FS bit times (%d ns)\n",
                        8, hub->tt.think_time);
        }
        break;
    case HUB_CHAR_TTTT_16_BITS:
        hub->tt.think_time = 666 * 2;
        USB_LOG_DBG("TT requires at most %d "
                    "FS bit times (%d ns)\n",
                    16, hub->tt.think_time);
        break;
    case HUB_CHAR_TTTT_24_BITS:
        hub->tt.think_time = 666 * 3;
        USB_LOG_DBG("TT requires at most %d "
                    "FS bit times (%d ns)\n",
                    24, hub->tt.think_time);
        break;
    case HUB_CHAR_TTTT_32_BITS:
        hub->tt.think_time = 666 * 4;
        USB_LOG_DBG("TT requires at most %d "
                    "FS bit times (%d ns)\n",
                    32, hub->tt.think_time);
        break;
    }
#endif

    hport->config.intf[intf].priv = hub;

    ret = _usbh_hub_get_hub_descriptor(hub, (uint8_t *)&hub->hub_desc);
    if (ret < 0) {
        return ret;
    }

    parse_hub_descriptor(&hub->hub_desc, USB_SIZEOF_HUB_DESC);

    for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
        hub->child[port].port = port + 1;
        hub->child[port].parent = hub;
        hub->child[port].bus = hport->bus;
    }

    ep_desc = &hport->config.intf[intf].altsetting[0].ep[0].ep_desc;
    if (ep_desc->bEndpointAddress & 0x80) {
        USBH_EP_INIT(hub->intin, ep_desc);
    } else {
        return -1;
    }

    if (hport->speed == USB_SPEED_SUPER) {
        uint16_t depth = 0;
        struct usbh_hubport *parent = hport->parent->parent;
        while (parent) {
            depth++;
            parent = parent->parent->parent;
        }

        ret = usbh_hub_set_depth(hub, depth);
        if (ret < 0) {
            return ret;
        }
    }

    for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
        ret = usbh_hub_set_feature(hub, port + 1, HUB_PORT_FEATURE_POWER);
        if (ret < 0) {
            return ret;
        }
    }

    for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
        ret = usbh_hub_get_portstatus(hub, port + 1, &port_status);
        USB_LOG_INFO("port %u, status:0x%02x, change:0x%02x\r\n", port + 1, port_status.wPortStatus, port_status.wPortChange);
        if (ret < 0) {
            return ret;
        }
    }

    hub->connected = true;
    snprintf(hport->config.intf[intf].devname, CONFIG_USBHOST_DEV_NAMELEN, DEV_FORMAT, hub->index);

    usb_slist_add_tail(&hub->bus->hub_list, &hub->list);

    USB_LOG_INFO("Register HUB Class:%s\r\n", hport->config.intf[intf].devname);

    hub->int_buffer = g_hub_intbuf[hub->bus->busid][hub->index - 1];

    hub->int_timer = usb_osal_timer_create("hubint_tim", USBH_GET_URB_INTERVAL(hub->intin->bInterval, hport->speed), hub_int_timeout, hub, 0);
    usb_osal_timer_start(hub->int_timer);
    return 0;
}

static int usbh_hub_disconnect(struct usbh_hubport *hport, uint8_t intf)
{
    struct usbh_hubport *child;
    int ret = 0;

    struct usbh_hub *hub = (struct usbh_hub *)hport->config.intf[intf].priv;

    if (hub) {
        if (hub->intin) {
            usbh_kill_urb(&hub->intin_urb);
        }

        usb_osal_timer_delete(hub->int_timer);

        for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
            child = &hub->child[port];
            usbh_hubport_release(child);
            child->parent = NULL;
        }

        if (hport->config.intf[intf].devname[0] != '\0') {
            USB_LOG_INFO("Unregister HUB Class:%s\r\n", hport->config.intf[intf].devname);
            usb_slist_remove(&hub->bus->hub_list, &hub->list);
        }

        usbh_hub_class_free(hub);
    }
    return ret;
}
#endif

static int usbh_hub_check_attachment(struct usbh_hub *hub, uint8_t port, struct hub_port_status *status)
{
    int ret = usbh_hub_get_portstatus(hub, port, status);
    if (ret < 0)
        return ret;
    if (!(status->wPortStatus & HUB_PORT_STATUS_CONNECTION) ||
        (status->wPortChange & HUB_PORT_STATUS_C_CONNECTION))
        return -USB_ERR_NOTCONN;
    if (status->wPortStatus & HUB_PORT_STATUS_OVERCURRENT)
        return -USB_ERR_IO;
#ifdef CHERRY_USB_HOST_ENABLE_CLASS_UAC1_PCM
    if (hub->is_roothub && (!(status->wPortStatus & HUB_PORT_STATUS_POWER) ||
        (status->wPortStatus & HUB_PORT_STATUS_SUSPEND)))
        return -USB_ERR_IO;
#endif
    return 0;
}

/* A reset returning success says nothing about the preceding enumeration.
 * Give each attempt a fresh USB address, endpoint state and class ownership. */
static int usbh_hub_enumerate_port(struct usbh_hub *hub, uint8_t port, unsigned attempts)
{
    struct usbh_hubport *child = &hub->child[port - 1];
    struct hub_port_status status;
    int ret;

    usbh_hubport_release(child);
    for (unsigned attempt = 1; attempt <= attempts; attempt++) {
        /* The previous EP0 failure can also mean a real unplug. */
        if (hub->is_roothub) usbh_root_note(ROOT_PHASE, "pre-reset-status", 0, 0);
        ret = usbh_hub_check_attachment(hub, port, &status);
        if (ret < 0)
            return ret;

        int policy = hub->is_roothub ? usbh_root_begin_trial() : -1;
        if (policy == ROOT_BOOT_SETTLE || policy == ROOT_BOOT_LONG) {
            /* Only the first stable attach early in this host's lifetime.
             * Do not reset a replacement device or a port that went away. */
            usbh_root_note(ROOT_PHASE, "startup-settle", 0, 0);
            usb_osal_msleep(1000);
            ret = usbh_hub_check_attachment(hub, port, &status);
            if (ret < 0) {
                usbh_root_end_trial();
                return ret;
            }
        }
        if (hub->is_roothub) usbh_root_note(ROOT_RESET, NULL, 0, 0);
        ret = usbh_hub_set_feature(hub, port, HUB_PORT_FEATURE_RESET);
        if (ret < 0) {
            if (policy >= 0) usbh_root_end_trial();
            return ret;
        }
        bool early = policy == ROOT_BOOT_EARLY || policy == ROOT_BOOT_EARLY30;
        uint32_t released = early ? usb_osal_timestamp() : 0;
        usb_osal_msleep(early ? (policy == ROOT_BOOT_EARLY30 ? 30 : 15) : DELAY_TIME_AFTER_RESET);
        if (hub->is_roothub) usbh_root_note(ROOT_PHASE, "post-reset-status", 0, 0);
        for (;;) {
            ret = usbh_hub_check_attachment(hub, port, &status);
            if (ret < 0 || !early || (!(status.wPortStatus & HUB_PORT_STATUS_RESET) &&
                (status.wPortStatus & HUB_PORT_STATUS_ENABLE))) break;
            uint32_t elapsed = usb_osal_timestamp() - released;
            if (elapsed >= DELAY_TIME_AFTER_RESET) break;
            uint32_t remaining = DELAY_TIME_AFTER_RESET - elapsed;
            usb_osal_msleep(remaining < 5 ? remaining : 5);
        }
        if (policy >= 0) usbh_root_end_trial();
        if (ret < 0)
            return ret;
        if ((status.wPortStatus & HUB_PORT_STATUS_RESET) ||
            !(status.wPortStatus & HUB_PORT_STATUS_ENABLE))
            return -USB_ERR_IO;
        if (status.wPortChange & HUB_PORT_STATUS_C_RESET) {
            if (hub->is_roothub) usbh_root_note(ROOT_PHASE, "clear-reset", 0, 0);
            ret = usbh_hub_clear_feature(hub, port, HUB_PORT_FEATURE_C_RESET);
            if (ret < 0)
                return ret;
        }

        uint8_t speed = USB_SPEED_FULL;
        if (status.wPortStatus & HUB_PORT_STATUS_HIGH_SPEED)
            speed = USB_SPEED_HIGH;
        else if (status.wPortStatus & HUB_PORT_STATUS_LOW_SPEED)
            speed = USB_SPEED_LOW;
#ifdef CONFIG_USBHOST_XHCI
        else {
            extern uint8_t usbh_get_port_speed(struct usbh_hub *hub, const uint8_t port);
            uint8_t super_speed = usbh_get_port_speed(hub, port);
            if (super_speed > USB_SPEED_HIGH)
                speed = super_speed;
        }
#endif
        usb_memset(child, 0, sizeof(*child));
#ifdef CHERRY_USB_HC_DRV_DWC2
        if (!hub->parent) {
            child->tt = &hub->tt;
            child->ttport = 1;
        } else if (speed != USB_SPEED_HIGH && hub->parent->speed == USB_SPEED_HIGH) {
            if (!hub->tt.hub)
                return -USB_ERR_IO;
            child->tt = &hub->tt;
            child->ttport = port;
        }
#endif
        child->mutex = usb_osal_mutex_create();
        if (!child->mutex)
            return -USB_ERR_NOMEM;
        child->parent = hub;
        child->port = port;
        child->speed = speed;
        child->bus = hub->bus;
        child->connected = true;

        /* A validated early sample already includes the reset-recovery time. */
        if (!early) usb_osal_msleep(DELAY_TIME_BEFORE_ENUMERATE);
        if (hub->is_roothub) usbh_root_note(ROOT_ENUM_TRY, NULL, 0, 0);
        ret = usbh_enumerate(child);
        if (ret >= 0) {
            USB_LOG_INFO("Enumerated %s device on bus %u hub %u port %u\r\n",
                         speed_table[speed], hub->bus->busid, hub->index, port);
            return 0;
        }
        USBH_ROOT_PORT_NOTE(child, ROOT_ERROR, NULL, ret, 0);
        usbh_hubport_release(child);
        USB_LOG_WRN("Hub %u port %u enumeration attempt %u failed (%d)\r\n",
                    hub->index, port, attempt, ret);
        if (attempt == attempts)
            return ret;
        if (ret == -USB_ERR_NOTCONN)
            usb_osal_msleep(DELAY_TIME_ENUM_NOTCONN_BACKOFF);
    }
    return -USB_ERR_INVAL;
}

static void usbh_hub_events(struct usbh_hub *hub, bool rescan, uint8_t *root_retries)
{
    struct usbh_hubport *child;
    struct hub_port_status port_status;
    uint8_t portchange_index;
    uint16_t portstatus;
    uint16_t portchange;
    uint16_t mask;
    uint16_t feat;
    int ret;

    if (!hub->connected) {
        return;
    }

    if (hub->is_roothub) usbh_root_note(ROOT_DISPATCH, NULL, rescan, 0);
    /* A timeout must not consume a genuine interrupt queued on another CPU. */
    portchange_index = 0;
    if (!rescan) {
        size_t flags = usb_osal_enter_critical_section();
        portchange_index = hub->int_buffer[0];
        hub->int_buffer[0] &= ~portchange_index;
        usb_osal_leave_critical_section(flags);
    }

    for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
        USB_LOG_DBG("Port change:0x%02x\r\n", portchange_index);

        child = &hub->child[port];
        if (rescan ? child->connected : !(portchange_index & (1 << (port + 1)))) {
            continue;
        }
        portchange_index &= ~(1 << (port + 1));
        USB_LOG_DBG("Port %d change\r\n", port + 1);

        /* Read hub port status */
        ret = usbh_hub_get_portstatus(hub, port + 1, &port_status);
        if (ret < 0) {
            if (hub->is_roothub) usbh_root_note(ROOT_ERROR, "read-status", ret, 0);
            USB_LOG_ERR("Failed to read port %u status, errorcode: %d\r\n", port + 1, ret);
            continue;
        }

        portstatus = port_status.wPortStatus;
        portchange = port_status.wPortChange;

        USB_LOG_DBG("port %u, status:0x%02x, change:0x%02x\r\n", port + 1, portstatus, portchange);

        if (hub->is_roothub) usbh_root_note(ROOT_PORT_STATUS, NULL, portstatus, portchange);

        /* First, clear all change bits */
        mask = 1;
        feat = HUB_PORT_FEATURE_C_CONNECTION;
        while (portchange) {
            if (portchange & mask) {
                ret = usbh_hub_clear_feature(hub, port + 1, feat);
                if (ret < 0) {
                    if (hub->is_roothub) usbh_root_note(ROOT_ERROR, "clear-change", ret, 0);
                    USB_LOG_ERR("Failed to clear port %u, change mask:%04x, errorcode:%d\r\n", port + 1, mask, ret);
                    break;
                }
                portchange &= (~mask);
            }
            mask <<= 1;
            feat++;
        }

        if (portchange)
            continue; /* A failed CLEAR_FEATURE will be observed on the next check. */
        portchange = port_status.wPortChange;
        bool changed = (portchange & HUB_PORT_STATUS_C_CONNECTION) != 0;
        bool retry = false;
        if (root_retries) {
            if (changed || !(portstatus & HUB_PORT_STATUS_CONNECTION))
                root_retries[port] = 0;
            else if (rescan && !child->connected && (portstatus & HUB_PORT_STATUS_POWER) &&
                     !(portstatus & (HUB_PORT_STATUS_OVERCURRENT | HUB_PORT_STATUS_SUSPEND)) &&
                     root_retries[port] < ROOT_RECHECK_ATTEMPTS) {
                root_retries[port]++;
                retry = true;
            }
        }

        /* Debounce both physical change notifications and recovery attempts. */
        if (changed || retry) {
            uint16_t connection = 0;
            uint16_t debouncestable = 0;
            for (uint32_t debouncetime = 0; debouncetime < HUB_DEBOUNCE_TIMEOUT; debouncetime += HUB_DEBOUNCE_STEP) {
                /* Read hub port status */
                ret = usbh_hub_get_portstatus(hub, port + 1, &port_status);
                if (ret < 0) {
                    USB_LOG_ERR("Failed to read port %u status, errorcode: %d\r\n", port + 1, ret);
                    continue;
                }

                portstatus = port_status.wPortStatus;
                portchange = port_status.wPortChange;

                USB_LOG_DBG("Port %u, status:0x%02x, change:0x%02x\r\n", port + 1, portstatus, portchange);

                if (!(portchange & HUB_PORT_STATUS_C_CONNECTION) &&
                    ((portstatus & HUB_PORT_STATUS_CONNECTION) == connection)) {
                    debouncestable += HUB_DEBOUNCE_STEP;
                    if (debouncestable >= HUB_DEBOUNCE_STABLE) {
                        break;
                    }
                } else {
                    debouncestable = 0;
                    connection = portstatus & HUB_PORT_STATUS_CONNECTION;
                }

                if (portchange & HUB_PORT_STATUS_C_CONNECTION) {
                    usbh_hub_clear_feature(hub, port + 1, HUB_PORT_FEATURE_C_CONNECTION);
                }

                usb_osal_msleep(HUB_DEBOUNCE_STEP);
            }

            /** check if debounce ok */
            if (debouncestable < HUB_DEBOUNCE_STABLE) {
                if (hub->is_roothub) usbh_root_note(ROOT_ERROR, "debounce", -USB_ERR_TIMEOUT, 0);
                USB_LOG_ERR("Failed to debounce port %u\r\n", port + 1);
                continue;
            }

            /* Last, check connect status */
            if (portstatus & HUB_PORT_STATUS_CONNECTION) {
                ret = usbh_hub_enumerate_port(hub, port + 1, retry ? 1 : HUB_ENUM_ATTEMPTS);
                if (ret < 0) {
                    if (hub->is_roothub) usbh_root_note(ROOT_ERROR, NULL, ret, 0);
                    USB_LOG_ERR("Hub %u port %u enumerate failed: %d\r\n", hub->index, port + 1, ret);
                }
            } else {
                usbh_hubport_release(child);
                USB_LOG_INFO("Device on Bus %u, Hub %u, Port %u disconnected\r\n", hub->bus->busid, hub->index, port + 1);
            }
        }
    }

#ifndef CHERRY_USB_HC_DRV_DWC2
    /* Start next hub int transfer */
    if (!hub->is_roothub && hub->connected) {
        usb_osal_timer_start(hub->int_timer);
    }
#endif
}

static void usbh_hub_thread(void *argument)
{
    struct usbh_hub *hub;
    int ret = 0;

    struct usbh_bus *bus = (struct usbh_bus *)argument;

    usbh_root_note(ROOT_WORKER, NULL, 0, 0);
    uint8_t *root_retries = NULL;
#ifdef CHERRY_USB_HOST_ENABLE_CLASS_UAC1_PCM
    uint8_t retries[CONFIG_USBHOST_MAX_RHPORTS] = { 0 };
    root_retries = retries;
#endif
    while (1) {
        uint32_t wait = USB_OSAL_WAITING_FOREVER;
        struct usbh_hub *root = &bus->hcd.roothub;
        if (root_retries) {
            for (uint8_t port = 0; port < root->hub_desc.bNbrPorts; port++) {
                if (!root->child[port].connected) {
                    wait = ROOT_RECHECK_MS;
                    break;
                }
            }
        }
        ret = usb_osal_mq_recv(bus->hub_mq, (uintptr_t *)&hub, wait);
        if (ret == -USB_ERR_TIMEOUT && wait != USB_OSAL_WAITING_FOREVER) {
            usbh_hub_events(root, true, root_retries);
            continue;
        }
        if (ret < 0) {
            continue;
        }
        usbh_hub_events(hub, false, hub == root ? root_retries : NULL);
    }
}

void usbh_hub_thread_wakeup(struct usbh_hub *hub)
{
    int rc = usb_osal_mq_send(hub->bus->hub_mq, (uintptr_t)hub);
    if (hub->is_roothub) usbh_root_note(ROOT_NOTIFY, NULL, rc, 0);
    (void)rc;
}

int usbh_hub_initialize(struct usbh_bus *bus)
{
    char thread_name[32] = { 0 };
    int ret;

    bus->hub_mq = usb_osal_mq_create(7);
    if (bus->hub_mq == NULL) {
        usbh_root_note(ROOT_INIT, "queue", -USB_ERR_NOMEM, 0);
        USB_LOG_ERR("Failed to create hub mq\r\n");
        return -1;
    }

    ret = usb_hc_init(bus);
    if (ret < 0) {
        USB_LOG_ERR("Failed to init host controller, ret=%d\r\n", ret);
        usb_osal_mq_delete(bus->hub_mq);
        bus->hub_mq = NULL;
        return ret;
    }

    usbh_root_note(ROOT_INIT, "worker-create", 0, 0);
    snprintf(thread_name, 32, "usbh_hub%u", bus->busid);
    bus->hub_thread = usb_osal_thread_create(thread_name, CONFIG_USBHOST_PSC_STACKSIZE, CONFIG_USBHOST_PSC_PRIO, usbh_hub_thread, bus);
    if (bus->hub_thread == NULL) {
        usbh_root_note(ROOT_INIT, "worker-create", -USB_ERR_NOMEM, 0);
        USB_LOG_ERR("Failed to create hub thread\r\n");
        usb_hc_deinit(bus);
        usb_osal_mq_delete(bus->hub_mq);
        bus->hub_mq = NULL;
        return -1;
    }
    usbh_root_note(ROOT_INIT, "ready", 0, 0);
    return 0;
}

int usbh_hub_deinitialize(struct usbh_bus *bus)
{
    usb_slist_t *hub_list;
    struct usbh_hubport *hport;
    size_t flags;

    flags = usb_osal_enter_critical_section();

    usb_slist_for_each(hub_list, &bus->hub_list)
    {
        struct usbh_hub *hub = usb_slist_entry(hub_list, struct usbh_hub, list);

        for (uint8_t port = 0; port < hub->hub_desc.bNbrPorts; port++) {
            hport = &hub->child[port];

            usbh_hubport_release(hport);
        }
    }

    usb_hc_deinit(bus);

    usb_osal_leave_critical_section(flags);

    usb_osal_mq_delete(bus->hub_mq);
    usb_osal_thread_delete(bus->hub_thread);

    return 0;
}

#if CONFIG_USBHOST_MAX_EXTHUBS > 0
const struct usbh_class_driver hub_class_driver = {
    .driver_name = "hub",
    .connect = usbh_hub_connect,
    .disconnect = usbh_hub_disconnect
};

CLASS_INFO_DEFINE const struct usbh_class_info hub_class_info = {
    .match_flags = USB_CLASS_MATCH_INTF_CLASS,
    .class = USB_DEVICE_CLASS_HUB,
    .subclass = 0,
    .protocol = 0,
    .vid = 0x00,
    .pid = 0x00,
    .class_driver = &hub_class_driver
};
#endif
