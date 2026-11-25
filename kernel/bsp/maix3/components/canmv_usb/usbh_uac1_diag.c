/* SPDX-License-Identifier: MIT */
/* One bounded snapshot per read-only open. Reading never touches MMIO,
 * opens PCM, submits a USB transfer or alters a recovery counter. */
#include <errno.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include <rtdevice.h>
#include <rthw.h>
#include "usbh_root_diag.h"

static struct rt_device diag_dev;
#ifdef RT_USING_SMP
static struct rt_spinlock diag_lock;
#endif
struct root_snapshot {
    const char *init, *phase, *failure, *uac;
    int init_rc, error, mode, uac_rc;
    uint32_t workers, notices, missed, events, polls, status_reads;
    uint32_t status, change, resets, attempts, configs, releases;
    uint32_t addr, vidpid, bound, hprt, flags, gating;
    rt_tick_t hw_tick, error_tick;
    rt_tick_t worker_tick;
    unsigned policy, claimed, capture, trial_reset;
    int setup_rc;
    struct { unsigned present; rt_tick_t tick; int mode; uint32_t hprt; } first[ROOT_MARK_COUNT];
    struct { unsigned present; rt_tick_t tick; int mode; uint32_t num, interval, cfg; } frame[ROOT_FRAME_COUNT];
};
static struct root_snapshot last = { .init = "none", .phase = "none",
    .failure = "none", .uac = "unseen", .mode = -1, .policy = ROOT_BOOT_EARLY };

void usbh_root_load_policy(void)
{
    char data[16];
    unsigned policy = ROOT_BOOT_EARLY;
    int fd = open("/sdcard/uac_boot_policy.txt", O_RDONLY);
    if (fd >= 0) {
        int n = read(fd, data, sizeof(data));
        close(fd);
        if (n > 0 && n < (int)sizeof(data)) {
            while (n > 0 && (data[n - 1] == '\n' || data[n - 1] == '\r')) n--;
            if (n == 8 && !rt_memcmp(data, "baseline", 8)) policy = ROOT_BOOT_BASELINE;
            else if (n == 6 && !rt_memcmp(data, "settle", 6)) policy = ROOT_BOOT_SETTLE;
            else if (n == 4 && !rt_memcmp(data, "long", 4)) policy = ROOT_BOOT_LONG;
            else if (n == 7 && !rt_memcmp(data, "early30", 7)) policy = ROOT_BOOT_EARLY30;
        }
    }
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    last.policy = policy;
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

static int mark_wanted(enum usbh_root_mark mark)
{
    if (last.first[mark].present) return 0;
    if (last.capture) return 1;
    if (last.resets != last.trial_reset || !last.first[ROOT_MARK_CLEAR].present) return 0;
    /* An outstanding control request may complete after the observation window. */
    if (mark == ROOT_MARK_SETUP_DONE) return last.first[ROOT_MARK_SETUP].present;
    return (mark == ROOT_MARK_SETUP || mark == ROOT_MARK_DISCONNECT) &&
        rt_tick_get() - last.first[ROOT_MARK_CLEAR].tick < rt_tick_from_millisecond(300);
}

int usbh_root_trace_wanted(enum usbh_root_mark mark)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    int result = mark_wanted(mark);
    rt_spin_unlock_irqrestore(&diag_lock, irq);
    return result;
}

static void record_mark(enum usbh_root_mark mark, rt_tick_t tick, int mode, uint32_t hprt)
{
    if (!mark_wanted(mark)) return;
    last.first[mark].present = 1;
    last.first[mark].tick = tick;
    last.first[mark].mode = mode;
    last.first[mark].hprt = hprt;
}

int usbh_root_begin_trial(void)
{
    int policy = -1;
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    if (last.workers && !last.claimed && !last.configs &&
        rt_tick_get() - last.worker_tick < rt_tick_from_millisecond(5000)) {
        last.claimed = last.capture = 1;
        last.trial_reset = last.resets + 1;
        policy = (int)last.policy;
        record_mark(ROOT_MARK_PRE, last.hw_tick, last.mode, last.hprt);
    }
    rt_spin_unlock_irqrestore(&diag_lock, irq);
    return policy;
}

int usbh_root_active_policy(void)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    int policy = last.capture ? (int)last.policy : -1;
    rt_spin_unlock_irqrestore(&diag_lock, irq);
    return policy;
}

void usbh_root_mark(enum usbh_root_mark mark, int mode, uint32_t hprt)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    record_mark(mark, rt_tick_get(), mode, hprt);
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

void usbh_root_end_trial(void)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    record_mark(ROOT_MARK_POST, last.hw_tick, last.mode, last.hprt);
    last.capture = 0;
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

void usbh_root_frames(enum usbh_frame_mark mark, int mode, uint32_t hfnum, uint32_t hfir, uint32_t hcfg)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    if (!last.frame[mark].present) {
        last.frame[mark].present = 1;
        last.frame[mark].tick = rt_tick_get();
        last.frame[mark].mode = mode;
        last.frame[mark].num = hfnum;
        last.frame[mark].interval = hfir;
        last.frame[mark].cfg = hcfg;
    }
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

void usbh_root_setup_result(int rc)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    last.setup_rc = rc;
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

void usbh_root_note(enum usbh_root_event event, const char *stage, int value, uint32_t detail)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    switch (event) {
        case ROOT_INIT: last.init = stage; last.init_rc = value; break;
        case ROOT_WORKER: last.worker_tick = rt_tick_get(); last.workers++; break;
        case ROOT_NOTIFY: last.notices++; if (value < 0) last.missed++; break;
        case ROOT_DISPATCH: if (value) last.polls++; else last.events++; break;
        case ROOT_PORT_STATUS: last.status_reads++; last.status = (uint32_t)value; last.change = detail; break;
        case ROOT_RESET: last.phase = "port-reset"; last.resets++; break;
        case ROOT_ENUM_TRY:
            last.phase = "enumerate"; last.attempts++;
            last.uac = "unseen"; last.uac_rc = 0; last.bound = 0;
            break;
        case ROOT_PHASE: last.phase = stage; break;
        case ROOT_ERROR:
            last.failure = stage ? stage : last.phase;
            last.error = value; last.error_tick = rt_tick_get(); break;
        case ROOT_CONFIGURED:
            last.configs++; last.addr = (uint32_t)value; last.vidpid = detail; break;
        case ROOT_RELEASE: last.releases++; last.addr = 0; last.vidpid = 0; break;
        case ROOT_UAC: last.uac = stage; last.uac_rc = value; last.bound = detail; break;
    }
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

void usbh_root_hw(int mode, uint32_t hprt, uint32_t sw_flags, uint32_t power_flags)
{
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    last.mode = mode;
    last.hprt = hprt;
    last.flags = sw_flags;
    last.gating = power_flags;
    last.hw_tick = rt_tick_get();
    rt_spin_unlock_irqrestore(&diag_lock, irq);
}

struct diag_file { int length; char text[1536]; };

static int diag_open(struct dfs_fd *fd)
{
    if ((fd->flags & O_ACCMODE) != O_RDONLY) return -EACCES;
    struct diag_file *file = rt_malloc(sizeof(*file));
    if (!file) return -ENOMEM;
    rt_base_t irq = rt_spin_lock_irqsave(&diag_lock);
    struct root_snapshot s = last;
    rt_spin_unlock_irqrestore(&diag_lock, irq);
    int n = rt_snprintf(file->text, sizeof(file->text),
        "uac0diag_v1 tick=%u hz=%u init=%s init_rc=%d worker=%u notify=%u mq_fail=%u event=%u poll=%u\n"
        "hw_tick=%u mode=%d hprt_valid=%u hprt=%08x sw_flags=%08x gate=%x status_reads=%u status=%04x change=%04x\n"
        "resets=%u attempts=%u phase=%s fail=%s rc=%d fail_tick=%u configs=%u releases=%u addr=%u vidpid=%08x\n"
        "uac=%s uac_rc=%d bound=%u\n"
        "first_reset_policy=%s claimed=%u\n",
        (unsigned)rt_tick_get(), (unsigned)RT_TICK_PER_SECOND, s.init, s.init_rc, s.workers,
        s.notices, s.missed, s.events, s.polls, (unsigned)s.hw_tick, s.mode, (unsigned)(s.mode == 1),
        s.hprt, s.flags, s.gating, s.status_reads, s.status, s.change, s.resets, s.attempts,
        s.phase, s.failure, s.error, (unsigned)s.error_tick, s.configs, s.releases, s.addr,
        s.vidpid, s.uac, s.uac_rc, s.bound,
        s.policy == ROOT_BOOT_BASELINE ? "baseline" : s.policy == ROOT_BOOT_SETTLE ? "settle" :
        s.policy == ROOT_BOOT_LONG ? "long" : s.policy == ROOT_BOOT_EARLY30 ? "early30" : "early", s.claimed);
    if (n < 0 || (size_t)n >= sizeof(file->text)) { rt_free(file); return -EIO; }
    /* A dash records an unobserved phase, including a disconnect outside
     * the first reset window. Otherwise: tick/mode/raw HPRT. */
    static const char *const names[ROOT_MARK_COUNT] = {
        "first_pre", "first_assert", "first_clear", "first_post", "first_irq", "first_setup", "first_setup_done"
    };
    for (unsigned i = 0; i < ROOT_MARK_COUNT; i++) {
        size_t room = sizeof(file->text) - n;
        int written = s.first[i].present ?
            rt_snprintf(file->text + n, room, "%s=%u/%d/%08x\n",
                        names[i], (unsigned)s.first[i].tick, s.first[i].mode, s.first[i].hprt) :
            rt_snprintf(file->text + n, room, "%s=-\n", names[i]);
        if (written < 0 || (size_t)written >= room) { rt_free(file); return -EIO; }
        n += written;
    }
    static const char *const frame_names[ROOT_FRAME_COUNT] = { "frame_clear", "frame_setup", "frame_irq" };
    for (unsigned i = 0; i < ROOT_FRAME_COUNT; i++) {
        size_t room = sizeof(file->text) - n;
        int written = s.frame[i].present ?
            rt_snprintf(file->text + n, room, "%s=%u/%d/%08x/%08x/%08x\n", frame_names[i],
                        (unsigned)s.frame[i].tick, s.frame[i].mode, s.frame[i].num,
                        s.frame[i].interval, s.frame[i].cfg) :
            rt_snprintf(file->text + n, room, "%s=-\n", frame_names[i]);
        if (written < 0 || (size_t)written >= room) { rt_free(file); return -EIO; }
        n += written;
    }
    size_t room = sizeof(file->text) - n;
    int written = s.first[ROOT_MARK_SETUP_DONE].present ?
        rt_snprintf(file->text + n, room, "setup_rc=%d\n", s.setup_rc) :
        rt_snprintf(file->text + n, room, "setup_rc=-\n");
    if (written < 0 || (size_t)written >= room) { rt_free(file); return -EIO; }
    n += written;
    file->length = n;
    fd->pos = 0;
    fd->data = file;
    return 0;
}

static int diag_read(struct dfs_fd *fd, void *dst, size_t count)
{
    struct diag_file *file = fd->data;
    if (fd->pos < 0 || !file) return -EINVAL;
    if (fd->pos >= file->length || !count) return 0;
    size_t remaining = (size_t)(file->length - fd->pos);
    if (count > remaining) count = remaining;
    const char *src = file->text + fd->pos;
    /* sys_read already supplies the DFS callback a kernel buffer. */
    rt_memcpy(dst, src, count);
    fd->pos += count;
    return (int)count;
}

static int diag_close(struct dfs_fd *fd)
{
    rt_free(fd->data);
    fd->data = NULL;
    return 0;
}

static const struct dfs_file_ops diag_fops = {
    .open = diag_open, .read = diag_read, .close = diag_close
};

static int usbh_diag_init(void)
{
    rt_spin_lock_init(&diag_lock);
    diag_dev.type = RT_Device_Class_Char;
    int rc = rt_device_register(&diag_dev, "uac0diag", RT_DEVICE_FLAG_RDONLY);
    if (rc == RT_EOK) diag_dev.fops = &diag_fops;
    return rc;
}
INIT_DEVICE_EXPORT(usbh_diag_init);
