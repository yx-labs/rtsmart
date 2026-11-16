/* SPDX-License-Identifier: MIT */
#ifndef CANMV_UAC1_PROFILE_H
#define CANMV_UAC1_PROFILE_H
#include <stdint.h>
#include <stddef.h>

#define UAC1_RATE 48000u
#define UAC1_MAX_PACKET 256u

struct uac1_alt {
    uint8_t interface, setting, channels, endpoint, set_rate;
    uint16_t max_packet;
};
struct uac1_profile {
    struct uac1_alt capture, playback;
};

/* Parse the physical configuration, without depending on descriptor order,
 * an IAD, contiguous interface numbers or a fixed VID/PID. This first driver
 * supports 1 ms Full Speed Type-I PCM streams; slower devices and profiles
 * requiring separate feedback or clock entities are rejected. */
int uac1_parse(const uint8_t *raw, size_t len, uint8_t control,
               struct uac1_profile *out);
#endif
