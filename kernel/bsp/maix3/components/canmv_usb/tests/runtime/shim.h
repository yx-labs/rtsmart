/* Deterministic scheduler and minimal RT/DFS facades for the host test. The
 * actual CanMV C driver, CherryUSB structures and descriptor parser are used. */
#ifndef UAC1_TEST_SHIM_H
#define UAC1_TEST_SHIM_H
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifndef O_ACCMODE
#define O_ACCMODE 3
#endif
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_EOK 0
#define RT_WAITING_FOREVER (-1)
#define RT_IPC_FLAG_PRIO 1
#define RT_IPC_FLAG_FIFO 0
#define RT_DEVICE_FLAG_RDWR 3
#define RT_Device_Class_Char 1
#define INIT_DEVICE_EXPORT(fn)
typedef int rt_bool_t;
typedef int rt_spinlock_t;
struct rt_work { int unused; };
#define rt_memcpy memcpy
#define rt_memset memset
#define rt_kprintf printf
#define rt_snprintf snprintf

struct rt_mutex { int locked; };
struct rt_semaphore { unsigned count; };
struct dfs_file { void *data; };
struct dfs_fd { struct dfs_file *fnode; int flags; void *data; };
struct dfs_file_ops {
    int (*open)(struct dfs_fd *);
    int (*close)(struct dfs_fd *);
    int (*ioctl)(struct dfs_fd *, int, void *);
    int (*read)(struct dfs_fd *, void *, size_t);
    int (*write)(struct dfs_fd *, const void *, size_t);
};
struct rt_device { int type; const struct dfs_file_ops *fops; };
typedef void *rt_thread_t;
#define rt_mutex_init(m,...) ((m)->locked = 0)
#define rt_mutex_take(m,forever) do { assert(!(m)->locked); (m)->locked = 1; } while (0)
#define rt_mutex_release(m) do { assert((m)->locked); (m)->locked = 0; } while (0)
#define rt_mutex_detach(m) ((void)(m))
#define rt_sem_init(s,...) ((s)->count = 0)
#define rt_sem_detach(s) ((void)(s))
#define rt_sem_release(s) (++(s)->count)
static jmp_buf idle;
static void rt_sem_take(struct rt_semaphore *s, int timeout) {
    assert(timeout == RT_WAITING_FOREVER);
    if (!s->count) longjmp(idle, 1);
    --s->count;
}
#define rt_device_register(dev,name,flags) (assert(dev && name && (flags) == RT_DEVICE_FLAG_RDWR), RT_EOK)
#define rt_device_unregister(dev) ((void)(dev))
#define rt_thread_create(name,entry,arg,depth,prio,slice) ((void *)1)
#define rt_thread_startup(thread) ((void)(thread), RT_EOK)
#define rt_thread_delete(thread) ((void)(thread))
#define lwp_self() NULL
#define lwp_user_accessable(addr,n) (1)
#define lwp_put_to_user(dst,src,n) (memcpy(dst,src,n),(n))
#endif
