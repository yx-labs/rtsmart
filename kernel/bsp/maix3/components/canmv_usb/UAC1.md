# CanMV UAC1 PCM on 01Studio K230

Select the kernel configuration `k230_canmv_01studio_uac1_defconfig` in the
matching CanMV v1.8 RT-Smart SDK. The stock configuration leaves the driver
disabled. The new configuration uses USB host 1 and the K230 DWC2 descriptor
DMA driver.

The first version supports a directly attached Full Speed USB Audio Class 1
function with a Type-I signed 16-bit PCM alternate setting at 48 kHz. Capture
and playback can each have one or two channels, with a single isochronous data
endpoint at 1 ms intervals. Playback supports adaptive or synchronous clocks.
Cards with a separate feedback endpoint, UAC2 clock controls, hub split
transactions or alternative sample rates require further driver work. Device
selection uses interface descriptors and the AudioControl header's stream
collection, so a missing IAD or an unrelated HID interface is supported.

## Userspace interface

The user/kernel ABI is declared in `uac1_pcm.h`. Opening `/dev/uac0i`
and requesting `UAC1_PCM_CAPS` is nonintrusive, including when unplugged.
Open `/dev/uac0c` read-only or `/dev/uac0p` write-only. Exactly one open
PCM file owns both the USB function's current mode and its native queue.
A second open returns `EBUSY`. Use the channel count in `UAC1_PCM_INFO`
and convert from/to the application's rate outside this driver.

Reads and writes are frame-aligned and nonblocking, and can be short.
Retry `EAGAIN` after servicing the application. A transport failure reports
`EIO` and increments `io_errors`; physical removal reports `ENODEV`.
The generation increases across disconnect and reconnect. A stale descriptor
cannot read, write, stop or close the next owner's stream.

`buffered_bytes` counts queued application PCM. For playback,
`inflight_bytes` counts real application bytes submitted in ISO transfers,
excluding native silence padding. Require both to reach zero in a connected,
error-free generation before declaring USB output delivered. This measures
delivery to the device, not analog speaker completion. `UAC1_PCM_DROP` and
close cancel/join all USB requests and clear the queue. The info node creates
no stream. The ring capacity is 64 KiB; the kernel pump maintains four
8-packet DMA requests and propagates per-packet transfer errors.

## Verification

Run `python3 tests/test_profile.py` here to compile and exercise the C
descriptor parser against a captured 253-byte USB configuration, alternate
rate ordering, malformed configurations and unsupported endpoints.
On a candidate board, compare `lsusb -v`, `lsusb -t` and the capability ioctl.
Expected interface drivers for selected streams are `uac1_pcm` (control) and
`uac1_stream` (streaming). Record actual PCM capture, independent audible
playback, detach mid-stream, reconnect and application capture/play transitions.
Zero I/O errors, pending buffers and PCM timestamps are measurable transport
signals; acoustic verification needs an independent microphone or listener.
