/* SPDX-License-Identifier: MIT */
#include "shim.h"
#include "../../usbh_uac1_pcm.c"

static struct usbh_urb *pending[SLOTS];
static unsigned submits, kills, altsetting, requested_hz;

int usbh_control_transfer(struct usbh_hubport *p, struct usb_setup_packet *r, uint8_t *data)
{
    assert(p->connected);
    if (r->bRequest == USB_REQUEST_SET_INTERFACE)
        altsetting = r->wValue;
    else {
        assert(r->bRequest == 1 && r->wLength == 3 && data);
        requested_hz = data[0] | (data[1] << 8) | (data[2] << 16);
    }
    return 0;
}

int usbh_submit_urb(struct usbh_urb *urb)
{
    size_t slot = (struct transfer *)urb->arg - slots;
    assert(slot < SLOTS && !pending[slot] && urb->hport->connected);
    assert(urb->num_of_iso_packets == PACKETS && urb->ep && urb->transfer_buffer);
    for (unsigned j = 0; j < PACKETS; ++j)
        assert(urb->iso_packet[j].offset + urb->iso_packet[j].length <= urb->transfer_buffer_length);
    pending[slot] = urb;
    ++submits;
    return 0;
}

int usbh_kill_urb(struct usbh_urb *urb)
{
    size_t slot = (struct transfer *)urb->arg - slots;
    if (pending[slot]) {
        pending[slot] = NULL;
        urb->complete(urb->arg, -1);
        ++kills;
    }
    return 0;
}

static void service(void)
{
    if (!setjmp(idle)) pump(NULL);
    assert(!pcm.lock.locked);
}

static void deliver(size_t slot, unsigned frame_bytes, int16_t value)
{
    struct usbh_urb *u = pending[slot];
    assert(u);
    pending[slot] = NULL;
    u->errorcode = 0;
    for (unsigned j = 0; j < PACKETS; ++j) {
        struct usbh_iso_frame_packet *p = &u->iso_packet[j];
        assert(frame_bytes <= p->length);
        p->actual_length = frame_bytes;
        if (u->ep->bEndpointAddress & 0x80) {
            for (unsigned k = 0; k + 1 < frame_bytes; k += 2) {
                u->transfer_buffer[p->offset + k] = value & 255;
                u->transfer_buffer[p->offset + k + 1] = (uint16_t)value >> 8;
            }
        }
    }
    u->complete(u->arg, (int)(frame_bytes * PACKETS));
}

static void cache_descriptors(struct usbh_hubport *p, uint8_t *raw, size_t size)
{
    /* Populate the same public descriptor fields that host enumeration passes
     * to the class callbacks. The profile selection still uses the real parser. */
    p->raw_config_desc = raw;
    p->config.config_desc.wTotalLength = (uint16_t)size;
    int interface = -1, alt = -1;
    for (size_t i = 0; i + 2 <= size && raw[i]; i += raw[i]) {
        uint8_t *d = raw + i;
        assert(i + d[0] <= size);
        if (d[1] == 4) {
            interface = d[2];
            alt = d[3];
            assert(interface < CONFIG_USBHOST_MAX_INTERFACES && alt < CONFIG_USBHOST_MAX_INTF_ALTSETTINGS);
            struct usbh_interface *f = &p->config.intf[interface];
            f->altsetting_num = alt + 1;
            f->altsetting[alt].intf_desc.bAlternateSetting = alt;
            f->altsetting[alt].intf_desc.bNumEndpoints = d[4];
        } else if (d[1] == 5) {
            assert(interface >= 0 && alt >= 0);
            struct usb_endpoint_descriptor *ep = &p->config.intf[interface].altsetting[alt].ep[0].ep_desc;
            ep->bEndpointAddress = d[2]; ep->bmAttributes = d[3];
            ep->wMaxPacketSize = d[4] | (d[5] << 8); ep->bInterval = d[6];
        }
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    uint8_t raw[256], received[2048], payload[FIFO_SIZE];
    FILE *source = fopen(argv[1], "rb");
    assert(source);
    size_t size = fread(raw, 1, sizeof(raw), source);
    fclose(source);
    assert(size == 253);
    struct usbh_hub root = {0};
    root.is_roothub = true;
    struct usbh_hubport board = {0};
    board.parent = &root;
    board.speed = USB_SPEED_FULL;
    board.connected = true;
    board.device_desc.idVendor = 0x08bb;
    board.device_desc.idProduct = 0x2902;
    cache_descriptors(&board, raw, size);
    assert(pcm_init() == 0);
    struct dfs_file i = { &pcm.dev[IDLE] }, c = { &pcm.dev[CAPTURE] }, p = { &pcm.dev[PLAYBACK] };
    struct dfs_fd info = { &i, O_RDONLY, NULL }, mic = { &c, O_RDONLY, NULL };
    struct dfs_fd output = { &p, O_WRONLY, NULL }, new_output = output;
    struct uac1_pcm_caps caps;
    struct uac1_pcm_info progress;
    assert(pcm_fops.open(&info) == 0 && pcm_fops.ioctl(&info, UAC1_PCM_CAPS, &caps) == 0);
    assert(!caps.connected && pcm_fops.open(&mic) == -ENODEV);
    assert(audio_connect(&board, 0) == 0);
    assert(pcm_fops.ioctl(&info, UAC1_PCM_CAPS, &caps) == 0 && caps.connected);
    assert(caps.vid == 0x08bb && caps.pid == 0x2902 && caps.capture_channels == 1 && caps.playback_channels == 2);
    uint32_t generation = caps.generation;
    assert(pcm_fops.open(&mic) == 0 && altsetting == 1 && requested_hz == 48000);
    assert(pcm_fops.open(&output) == -EBUSY && pcm_fops.read(&mic, received, 12) == -EAGAIN);
    service();
    assert(submits == SLOTS);
    deliver(1, 94, 0x1122); service();
    assert(pcm_fops.read(&mic, received, sizeof(received)) == -EAGAIN);
    deliver(0, 98, 0x3344); service();
    assert(pcm_fops.read(&mic, received, sizeof(received)) == (98 + 94) * PACKETS);
    assert(received[0] == 0x44 && received[98 * PACKETS] == 0x22);
    /* Reject a non-whole capture sample and force all in-flight URBs down. */
    deliver(pcm.head, 95, 0); service();
    assert(pcm_fops.read(&mic, received, 8) == -EIO);
    assert(pcm_fops.ioctl(&mic, UAC1_PCM_INFO, &progress) == 0 && progress.io_errors);
    assert(!pcm.active && kills && pcm_fops.close(&mic) == 0);
    service();
    assert(pcm_fops.open(&output) == 0);
    assert(pcm_fops.open(&mic) == -EBUSY);
    memset(payload, 0x5c, sizeof(payload));
    assert(pcm_fops.write(&output, payload, 1536) == 1536);
    service();
    assert(pcm_fops.ioctl(&output, UAC1_PCM_INFO, &progress) == 0);
    assert(progress.buffered_bytes == 0 && progress.inflight_bytes == 1536);
    for (unsigned j = 0; j < 1536; ++j) assert(dma[0][j] == 0x5c);
    deliver(0, 192, 0); service();
    assert(pcm_fops.ioctl(&output, UAC1_PCM_INFO, &progress) == 0 && !progress.inflight_bytes);
    assert(pcm_fops.write(&output, payload, FIFO_SIZE - 8) == FIFO_SIZE - 8);
    assert(pcm_fops.write(&output, payload, 28) == 8);
    assert(pcm_fops.write(&output, payload, 4) == -EAGAIN);
    assert(pcm_fops.ioctl(&info, UAC1_PCM_DROP, NULL) == -EBADF);
    assert(pcm_fops.ioctl(&output, UAC1_PCM_DROP, NULL) == 0 && altsetting == 0);
    assert(pcm_fops.write(&output, payload, 4) == -EPIPE);
    assert(pcm_fops.close(&output) == 0);
    assert(pcm_fops.open(&output) == 0);
    assert(pcm_fops.write(&output, payload, 1024) == 1024); service();
    board.connected = false;
    assert(audio_disconnect(&board, 0) == 0);
    for (unsigned k = 0; k < SLOTS; ++k) assert(!pending[k]);
    assert(pcm_fops.ioctl(&info, UAC1_PCM_CAPS, &caps) == 0 && !caps.connected);
    assert(caps.generation != generation);
    board.connected = true;
    assert(audio_connect(&board, 0) == 0 && pcm_fops.open(&new_output) == 0);
    assert(pcm_fops.write(&output, payload, 8) == -ENODEV);
    assert(pcm_fops.close(&output) == 0 && pcm_fops.write(&new_output, payload, 8) == 8);
    assert(pcm_fops.close(&new_output) == 0 && pcm_fops.close(&info) == 0);
    puts("UAC1: real profile + PCM fops/URB pump: capture, payload drain, ring backpressure, DROP, ISO error, reconnect OK");
    return 0;
}
