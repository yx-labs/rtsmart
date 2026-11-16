/* SPDX-License-Identifier: MIT */
#ifndef CANMV_UAC1_PCM_H
#define CANMV_UAC1_PCM_H
#include <stdint.h>

/* Fixed-width userspace ABI. The information node does not claim any stream.
 * Open /dev/uac0c for reads, or /dev/uac0p for writes, one stream at a time.
 * Data is little endian signed 16-bit interleaved at the reported rate. */
#define UAC1_PCM_ABI 1
#define UAC1_PCM_INFO 0x5501
#define UAC1_PCM_DROP 0x5502
#define UAC1_PCM_CAPS 0x5503

struct uac1_pcm_info {
    uint32_t abi, connected, rate, channels, bits, buffered_bytes;
    uint32_t inflight_bytes, xruns, vid, pid, generation, io_errors;
};

struct uac1_pcm_caps {
    uint32_t abi, connected, vid, pid, generation;
    uint32_t capture_channels, playback_channels, rate;
};
#endif
