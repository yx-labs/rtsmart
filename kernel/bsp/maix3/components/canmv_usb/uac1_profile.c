/* SPDX-License-Identifier: MIT */
#include "uac1_profile.h"
#include <string.h>

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le24(const uint8_t *p)
{
    return le16(p) | ((uint32_t)p[2] << 16);
}

struct candidate {
    struct uac1_alt alt;
    int streaming, pcm, format, ep_count, invalid;
};

/* UAC1 Type-I PCM formats can advertise discrete rates or a closed range. */
static int supports_rate(const uint8_t *d)
{
    if (d[0] < 8 || d[3] != 1 || (d[4] != 1 && d[4] != 2) ||
        d[5] != 2 || d[6] != 16)
        return 0;
    if (!d[7])
        return d[0] >= 14 && le24(d + 8) <= UAC1_RATE && le24(d + 11) >= UAC1_RATE;
    if (8u + 3u * d[7] > d[0])
        return 0;
    for (unsigned i = 0; i < d[7]; ++i)
        if (le24(d + 8 + 3 * i) == UAC1_RATE)
            return 1;
    return 0;
}

static void pick(struct candidate *c, const uint8_t *collection,
                 unsigned count, struct uac1_profile *out)
{
    if (!c->streaming || !c->pcm || !c->format || c->invalid || c->ep_count != 1 ||
        c->alt.max_packet < (UAC1_RATE / 1000u) * 2u * c->alt.channels)
        return;
    for (unsigned i = 0; i < count; ++i) {
        if (collection[i] != c->alt.interface)
            continue;
        int input = (c->alt.endpoint & 0x80) != 0;
        struct uac1_alt *dst = input ? &out->capture : &out->playback;
        if (!dst->channels || (input ? c->alt.channels < dst->channels :
                                      c->alt.channels > dst->channels))
            *dst = c->alt;
        return;
    }
}

int uac1_parse(const uint8_t *raw, size_t len, uint8_t control,
               struct uac1_profile *out)
{
    const uint8_t *collection = NULL;
    unsigned count = 0;
    int in_control = 0;
    struct candidate c = {0};
    memset(out, 0, sizeof(*out));
    if (!raw || len < 9 || raw[0] != 9 || raw[1] != 2 || le16(raw + 2) != len)
        return -1;
    /* Validate all lengths first and identify the AS interfaces belonging to
     * this AudioControl header (including UAC1 devices without any IAD). */
    for (size_t off = 0; off < len;) {
        const uint8_t *d = raw + off;
        if (len - off < 2 || d[0] < 2 || d[0] > len - off)
            return -1;
        if (d[1] == 4) {
            if (d[0] < 9)
                return -1;
            in_control = d[2] == control && d[3] == 0 && d[5] == 1 && d[6] == 1 && d[7] == 0;
        } else if (in_control && d[1] == 0x24 && d[0] >= 3 && d[2] == 1) {
            if (collection || d[0] < 8 || le16(d + 3) != 0x0100 || 8u + d[7] > d[0])
                return -1;
            count = d[7];
            collection = d + 8;
        }
        off += d[0];
    }
    if (!collection || !count)
        return -1;
    for (size_t off = 0; off < len; off += raw[off]) {
        const uint8_t *d = raw + off;
        if (d[1] == 4) {
            pick(&c, collection, count, out);
            memset(&c, 0, sizeof(c));
            c.streaming = d[5] == 1 && d[6] == 2 && d[7] == 0 && d[3] && d[4] == 1;
            c.alt.interface = d[2];
            c.alt.setting = d[3];
        } else if (c.streaming && d[1] == 0x24 && d[0] >= 3) {
            if (d[2] == 1)
                c.pcm = d[0] >= 7 && le16(d + 5) == 1;
            if (d[2] == 2 && supports_rate(d)) {
                c.format = 1;
                c.alt.channels = d[4];
            }
        } else if (c.streaming && d[1] == 5) {
            ++c.ep_count;
            if (d[0] < 7 || (d[3] & 0x33) != 1 || (d[2] & 0x0f) == 0 ||
                (d[2] & 0x70) || d[6] != 1 || (d[0] >= 9 && d[8]) ||
                le16(d + 4) > UAC1_MAX_PACKET || (!(d[2] & 0x80) && (d[3] & 12) < 8)) {
                c.invalid = 1;
                continue;
            }
            c.alt.endpoint = d[2];
            c.alt.max_packet = le16(d + 4);
        } else if (c.streaming && c.ep_count == 1 && d[1] == 0x25 && d[0] >= 4 && d[2] == 1) {
            c.alt.set_rate = d[3] & 1;
        }
    }
    pick(&c, collection, count, out);
    return out->capture.channels || out->playback.channels ? 0 : -1;
}
