/* SPDX-License-Identifier: MIT
 * Full-speed UAC1 S16_LE PCM for CanMV on the K230 DWC2 DDMA host.
 * One persistent character device per direction. Only one direction owns
 * hardware at a time. USB giveback only wakes a native pump; no Python or
 * userspace buffers are touched by the USB completion path.
 */
#include <errno.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include <lwp.h>
#include <lwp_user_mm.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "usbh_core.h"
#include "uac1_pcm.h"
#include "uac1_profile.h"

#define PACKETS 8  /* DWC2 DDMA indexes ISO packet descriptors modulo a power of two */
#define SLOTS 4
#define FIFO_SIZE 65536
_Static_assert((PACKETS & (PACKETS - 1)) == 0 && PACKETS <= 32,
               "DWC2 DDMA expects power-of-two ISO packet counts up to 32");
enum direction { IDLE, CAPTURE, PLAYBACK };

struct transfer {
    struct usbh_urb urb;
    struct usbh_iso_frame_packet packets[PACKETS];
    volatile rt_bool_t done;
    size_t payload;
};
_Static_assert(offsetof(struct transfer, packets) ==
               offsetof(struct transfer, urb) + offsetof(struct usbh_urb, iso_packet),
               "ISO packet storage must follow URB header");
static struct transfer slots[SLOTS];
/* No packet shares a DMA cache line with another URB or kernel metadata. */
static uint8_t dma[SLOTS][UAC1_MAX_PACKET * PACKETS] USB_MEM_ALIGNX;
static uint8_t control_data[CONFIG_USB_ALIGN_SIZE] USB_MEM_ALIGNX;
static struct usb_setup_packet setup USB_MEM_ALIGNX;

static struct {
    struct rt_device dev[3];
    struct rt_mutex lock;
    struct rt_semaphore wake;
    rt_bool_t initialized, active, sent_audio, padded;
    struct usbh_hubport *port;
    struct uac1_profile profile;
    struct dfs_fd *owner;
    enum direction dir;
    struct uac1_alt selected;
    uint32_t generation, errors, xruns, inflight;
    unsigned head, queued;
    int fault;
    uint8_t fifo[FIFO_SIZE];
    size_t first, used;
} pcm;

/* All state except transfer.done belongs to the lifecycle mutex. done is
 * published by the DWC2 giveback thread and the semaphore provides wakeup. */
static size_t ring_read(uint8_t *dst, size_t n)
{
    if (n > pcm.used) n = pcm.used;
    size_t part = FIFO_SIZE - pcm.first;
    if (part > n) part = n;
    rt_memcpy(dst, pcm.fifo + pcm.first, part);
    rt_memcpy(dst + part, pcm.fifo, n - part);
    pcm.first = (pcm.first + n) % FIFO_SIZE;
    pcm.used -= n;
    return n;
}

static size_t ring_write(const uint8_t *src, size_t n)
{
    if (n > FIFO_SIZE - pcm.used) n = FIFO_SIZE - pcm.used;
    size_t last = (pcm.first + pcm.used) % FIFO_SIZE;
    size_t part = FIFO_SIZE - last;
    if (part > n) part = n;
    rt_memcpy(pcm.fifo + last, src, part);
    rt_memcpy(pcm.fifo, src + part, n - part);
    pcm.used += n;
    return n;
}

static void giveback(void *arg, int count)
{
    struct transfer *slot = arg;
    (void)count;
    slot->done = RT_TRUE;
    rt_sem_release(&pcm.wake);
}

static int set_alt(const struct uac1_alt *alt, uint8_t setting)
{
    setup.bmRequestType = USB_REQUEST_RECIPIENT_INTERFACE;
    setup.bRequest = USB_REQUEST_SET_INTERFACE;
    setup.wValue = setting;
    setup.wIndex = alt->interface;
    setup.wLength = 0;
    return usbh_control_transfer(pcm.port, &setup, NULL);
}

static int set_rate(const struct uac1_alt *alt)
{
    if (!alt->set_rate) return 0;
    control_data[0] = UAC1_RATE & 255;
    control_data[1] = (UAC1_RATE >> 8) & 255;
    control_data[2] = (UAC1_RATE >> 16) & 255;
    setup.bmRequestType = USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_ENDPOINT;
    setup.bRequest = 1;       /* UAC1 SET_CUR */
    setup.wValue = 1 << 8;    /* SAMPLING_FREQ_CONTROL */
    setup.wIndex = alt->endpoint;
    setup.wLength = 3;
    return usbh_control_transfer(pcm.port, &setup, control_data);
}

static int stop_stream(void)
{
    int result = 0;
    if (!pcm.active) return 0;
    pcm.active = RT_FALSE;
    /* usbh_kill_urb waits for giveback/use_count, including completions
     * already queued in the DWC2 bottom half. Only then reuse static slots. */
    for (unsigned i = 0; i < SLOTS; ++i) {
        if (slots[i].urb.ep && slots[i].urb.hport)
            usbh_kill_urb(&slots[i].urb);
    }
    if (pcm.port && pcm.port->connected && set_alt(&pcm.selected, 0) < 0)
        result = -EIO;
    pcm.first = pcm.used = pcm.inflight = pcm.queued = pcm.head = 0;
    return result;
}

static int submit_one(unsigned index)
{
    struct transfer *s = &slots[index];
    size_t block = pcm.dir == CAPTURE ? pcm.selected.max_packet :
                   (UAC1_RATE / 1000) * 2 * pcm.selected.channels;
    /* DWC2 DMA addresses must be word aligned even for mono mic endpoints
     * whose maximum packet size can be 98 bytes on a 48 kHz adaptive clock. */
    size_t stride = (block + 3) & ~(size_t)3;
    s->payload = 0;
    s->done = RT_FALSE;
    for (unsigned j = 0; j < PACKETS; ++j) {
        struct usbh_iso_frame_packet *iso = &s->urb.iso_packet[j];
        iso->offset = j * stride;
        iso->length = block;
        iso->actual_length = 0;
        iso->errorcode = 0;
        if (pcm.dir == PLAYBACK) {
            uint8_t *packet = dma[index] + iso->offset;
            size_t n = ring_read(packet, block);
            if (n) {
                if (pcm.padded) ++pcm.xruns; /* a gap followed by actual data */
                pcm.padded = RT_FALSE;
                pcm.sent_audio = RT_TRUE;
            }
            if (n < block) {
                rt_memset(packet + n, 0, block - n);
                if (pcm.sent_audio) pcm.padded = RT_TRUE;
            }
            s->payload += n;
        }
    }
    s->urb.transfer_buffer_length = stride * PACKETS;
    int result = usbh_submit_urb(&s->urb);
    if (result < 0) return -EIO;
    pcm.inflight += s->payload;
    ++pcm.queued;
    return 0;
}

static int complete_one(struct transfer *s)
{
    if (s->urb.errorcode < 0) return -EIO;
    for (unsigned j = 0; j < PACKETS; ++j) {
        const struct usbh_iso_frame_packet *iso = &s->urb.iso_packet[j];
        size_t n = iso->actual_length;
        if (iso->errorcode < 0 || n > iso->length ||
            n % (2u * pcm.selected.channels))
            return -EIO;
        if (pcm.dir == CAPTURE) {
            if (n > FIFO_SIZE - pcm.used) {
                ++pcm.xruns;
                continue; /* drop this complete USB packet */
            }
            ring_write(s->urb.transfer_buffer + iso->offset, n);
        } else if (n != iso->length) {
            return -EIO; /* never acknowledge truncated output as delivered */
        }
    }
    pcm.inflight -= s->payload;
    return 0;
}

static void pump(void *unused)
{
    (void)unused;
    for (;;) {
        rt_sem_take(&pcm.wake, RT_WAITING_FOREVER);
        rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
        if (pcm.active && !pcm.fault) {
            while (pcm.queued && slots[pcm.head].done) {
                int rc = complete_one(&slots[pcm.head]);
                pcm.head = (pcm.head + 1) % SLOTS;
                --pcm.queued;
                if (rc < 0) { pcm.fault = rc; break; }
            }
            if (pcm.dir == CAPTURE || pcm.used || pcm.sent_audio) {
                while (!pcm.fault && pcm.queued < SLOTS)
                    pcm.fault = submit_one((pcm.head + pcm.queued) % SLOTS);
            }
            if (pcm.fault) {
                ++pcm.errors;
                stop_stream();
            }
        }
        rt_mutex_release(&pcm.lock);
    }
}

static int pcm_open(struct dfs_fd *fd)
{
    struct rt_device *dev = fd->fnode->data;
    if (dev == &pcm.dev[IDLE]) {
        fd->data = NULL;
        return 0; /* capability query */
    }
    enum direction dir = dev == &pcm.dev[CAPTURE] ? CAPTURE : PLAYBACK;
    if ((fd->flags & O_ACCMODE) != (dir == CAPTURE ? O_RDONLY : O_WRONLY))
        return -EACCES;
    int rc = 0;
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    if (!pcm.port || !pcm.port->connected) { rc = -ENODEV; goto out; }
    if (pcm.owner) { rc = -EBUSY; goto out; }
    struct uac1_alt alt = dir == CAPTURE ? pcm.profile.capture : pcm.profile.playback;
    if (!alt.channels) { rc = -ENODEV; goto out; }
    /* Bind the endpoint discovered in this generation, never a stale pointer
     * cached across removal. Only one ISO data endpoint is accepted per alt. */
    if (alt.interface >= CONFIG_USBHOST_MAX_INTERFACES ||
        alt.setting >= CONFIG_USBHOST_MAX_INTF_ALTSETTINGS ||
        alt.setting >= pcm.port->config.intf[alt.interface].altsetting_num) {
        rc = -EINVAL; goto out;
    }
    struct usbh_interface_altsetting *slot =
        &pcm.port->config.intf[alt.interface].altsetting[alt.setting];
    struct usb_endpoint_descriptor *ep = &slot->ep[0].ep_desc;
    if (slot->intf_desc.bNumEndpoints != 1 || slot->intf_desc.bAlternateSetting != alt.setting ||
        ep->bEndpointAddress != alt.endpoint || ep->wMaxPacketSize != alt.max_packet ||
        (ep->bmAttributes & 0x33) != 1 || ep->bInterval != 1) {
        rc = -EINVAL; goto out;
    }
    if (set_alt(&alt, alt.setting) < 0 || set_rate(&alt) < 0) {
        if (pcm.port->connected) set_alt(&alt, 0);
        rc = -EIO; goto out;
    }
    pcm.dir = dir;
    pcm.selected = alt;
    pcm.fault = pcm.errors = pcm.xruns = pcm.inflight = 0;
    pcm.first = pcm.used = pcm.head = pcm.queued = 0;
    pcm.sent_audio = pcm.padded = RT_FALSE;
    rt_memset(slots, 0, sizeof(slots));
    for (unsigned i = 0; i < SLOTS; ++i) {
        struct usbh_urb *urb = &slots[i].urb;
        urb->transfer_buffer = dma[i];
        urb->ep = ep;
        urb->hport = pcm.port;
        urb->complete = giveback;
        urb->arg = &slots[i];
        urb->interval = 1;
        urb->num_of_iso_packets = PACKETS;
    }
    pcm.owner = fd;
    pcm.active = RT_TRUE;
    fd->data = dev;
    if (dir == CAPTURE) rt_sem_release(&pcm.wake);
out:
    rt_mutex_release(&pcm.lock);
    return rc;
}

static int pcm_close(struct dfs_fd *fd)
{
    int rc = 0;
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    if (fd == pcm.owner) {
        rc = stop_stream();
        pcm.owner = NULL;
        pcm.dir = IDLE;
    }
    fd->data = NULL;
    rt_mutex_release(&pcm.lock);
    return rc;
}

static int transfer_ready(struct dfs_fd *fd, enum direction dir)
{
    if (pcm.owner != fd || !pcm.port || !pcm.port->connected)
        return -ENODEV;
    if (pcm.dir != dir) return -EBADF;
    if (pcm.fault) return pcm.fault;
    if (!pcm.active) return -EPIPE;
    return 0;
}

static int pcm_read(struct dfs_fd *fd, void *data, size_t size)
{
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    int rc = transfer_ready(fd, CAPTURE);
    if (!rc && size) {
        size -= size % (2 * pcm.selected.channels);
        rc = size ? (int)ring_read(data, size) : -EINVAL;
        if (!rc) rc = -EAGAIN;
    }
    rt_mutex_release(&pcm.lock);
    return rc;
}

static int pcm_write(struct dfs_fd *fd, const void *data, size_t size)
{
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    int rc = transfer_ready(fd, PLAYBACK);
    if (!rc && size) {
        size -= size % (2 * pcm.selected.channels);
        rc = size ? (int)ring_write(data, size) : -EINVAL;
        if (!rc) rc = -EAGAIN;
        if (rc > 0 && !pcm.queued) rt_sem_release(&pcm.wake);
    }
    rt_mutex_release(&pcm.lock);
    return rc;
}

static int copy_result(void *dst, void *src, size_t bytes)
{
    if (!dst) return -EFAULT;
    if (lwp_self())
        return lwp_user_accessable(dst, bytes) && lwp_put_to_user(dst, src, bytes) == bytes ? 0 : -EFAULT;
    rt_memcpy(dst, src, bytes);
    return 0;
}

static int pcm_ioctl(struct dfs_fd *fd, int cmd, void *args)
{
    int rc = 0;
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    if (fd->data && fd != pcm.owner) { rc = -ENODEV; goto out; }
    int online = pcm.port && pcm.port->connected;
    uint32_t vid = online ? pcm.port->device_desc.idVendor : 0;
    uint32_t pid = online ? pcm.port->device_desc.idProduct : 0;
    switch (cmd) {
    case UAC1_PCM_INFO: {
        struct uac1_pcm_info info = {UAC1_PCM_ABI, online, UAC1_RATE,
            pcm.owner == fd ? pcm.selected.channels : 0, 16, pcm.used,
            pcm.inflight, pcm.xruns, vid, pid, pcm.generation, pcm.errors};
        rc = copy_result(args, &info, sizeof(info));
        break;
    }
    case UAC1_PCM_CAPS: {
        struct uac1_pcm_caps info = {UAC1_PCM_ABI, online, vid, pid, pcm.generation,
                                    pcm.profile.capture.channels, pcm.profile.playback.channels,
                                    online ? UAC1_RATE : 0};
        rc = copy_result(args, &info, sizeof(info));
        break;
    }
    case UAC1_PCM_DROP:
        rc = pcm.owner == fd ? stop_stream() : -EBADF;
        break;
    default: rc = -ENOTTY;
    }
out:
    rt_mutex_release(&pcm.lock);
    return rc;
}

static const struct dfs_file_ops pcm_fops = {
    .open = pcm_open, .close = pcm_close, .ioctl = pcm_ioctl,
    .read = pcm_read, .write = pcm_write
};

static int pcm_init(void)
{
    const char *names[] = {"uac0i", "uac0c", "uac0p"};
    rt_mutex_init(&pcm.lock, "uaclock", RT_IPC_FLAG_PRIO);
    rt_sem_init(&pcm.wake, "uacgive", 0, RT_IPC_FLAG_FIFO);
    unsigned i;
    for (i = 0; i < 3; ++i) {
        pcm.dev[i].type = RT_Device_Class_Char;
        if (rt_device_register(&pcm.dev[i], names[i], RT_DEVICE_FLAG_RDWR) != RT_EOK)
            goto failed;
        pcm.dev[i].fops = &pcm_fops;
    }
    rt_thread_t thread = rt_thread_create("uac-pcm", pump, NULL, 4096, 12, 5);
    if (!thread) goto failed;
    if (rt_thread_startup(thread) != RT_EOK) {
        rt_thread_delete(thread);
        goto failed;
    }
    pcm.initialized = RT_TRUE;
    return 0;
failed:
    while (i) rt_device_unregister(&pcm.dev[--i]);
    rt_mutex_detach(&pcm.lock);
    rt_sem_detach(&pcm.wake);
    return -ENOMEM;
}
INIT_DEVICE_EXPORT(pcm_init);

static int audio_connect(struct usbh_hubport *port, uint8_t ctrl)
{
    struct uac1_profile profile;
    if (!pcm.initialized || port->speed != USB_SPEED_FULL || !port->parent ||
        !port->parent->is_roothub ||
        uac1_parse(port->raw_config_desc, port->config.config_desc.wTotalLength, ctrl, &profile)) {
        /* An unmatched AudioControl interface must not fail enumeration of
         * independent interfaces (e.g. the HID buttons on a composite card). */
        port->config.intf[ctrl].class_driver = NULL;
        return 0;
    }
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    if (pcm.port) {
        port->config.intf[ctrl].class_driver = NULL;
        rt_mutex_release(&pcm.lock);
        return 0;
    }
    pcm.port = port;
    pcm.profile = profile;
    ++pcm.generation;
    port->config.intf[ctrl].priv = &pcm;
    rt_snprintf(port->config.intf[ctrl].devname, CONFIG_USBHOST_DEV_NAMELEN, "/dev/uac0i");
    USB_LOG_INFO("UAC1 %04x:%04x: 48000 Hz, capture %u ch, playback %u ch\n",
                 port->device_desc.idVendor, port->device_desc.idProduct,
                 profile.capture.channels, profile.playback.channels);
    rt_mutex_release(&pcm.lock);
    return 0;
}

static int audio_disconnect(struct usbh_hubport *port, uint8_t ctrl)
{
    if (port->config.intf[ctrl].priv != &pcm) return 0;
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    stop_stream();
    pcm.owner = NULL; /* old fds never gain access to a replacement device */
    pcm.port = NULL;
    pcm.dir = IDLE;
    ++pcm.generation;
    rt_memset(&pcm.profile, 0, sizeof(pcm.profile));
    port->config.intf[ctrl].priv = NULL;
    rt_mutex_release(&pcm.lock);
    return 0;
}

static const struct usbh_class_driver audio_ac = {
    .driver_name = "uac1_pcm", .connect = audio_connect, .disconnect = audio_disconnect
};
CLASS_INFO_DEFINE const struct usbh_class_info canmv_uac1_class_info = {
    .match_flags = USB_CLASS_MATCH_INTF_CLASS | USB_CLASS_MATCH_INTF_SUBCLASS | USB_CLASS_MATCH_INTF_PROTOCOL,
    .class = 1, .subclass = 1, .protocol = 0, .class_driver = &audio_ac
};

/* Only mark the specific AS interfaces negotiated by our AC function as
 * claimed. AudioControl owns the stream buffers and the disconnect path. */
static int streaming_connect(struct usbh_hubport *port, uint8_t intf)
{
    if (!pcm.initialized)
        goto unclaimed;
    rt_mutex_take(&pcm.lock, RT_WAITING_FOREVER);
    int owned = pcm.port == port &&
        ((pcm.profile.capture.channels && pcm.profile.capture.interface == intf) ||
         (pcm.profile.playback.channels && pcm.profile.playback.interface == intf));
    rt_mutex_release(&pcm.lock);
    if (owned) return 0;
unclaimed:
    port->config.intf[intf].class_driver = NULL;
    return 0;
}

static const struct usbh_class_driver audio_as = {
    .driver_name = "uac1_stream", .connect = streaming_connect
};
CLASS_INFO_DEFINE const struct usbh_class_info canmv_uac1_stream_info = {
    .match_flags = USB_CLASS_MATCH_INTF_CLASS | USB_CLASS_MATCH_INTF_SUBCLASS | USB_CLASS_MATCH_INTF_PROTOCOL,
    .class = 1, .subclass = 2, .protocol = 0, .class_driver = &audio_as
};
