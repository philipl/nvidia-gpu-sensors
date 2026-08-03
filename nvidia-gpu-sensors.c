/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2026 Philip Langdale
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * nvidia-gpu-sensors - Read nvidia GPU temperature and voltage data
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/* ---- RM ioctl constants (open-gpu-kernel-modules) ------------------------ */
#define NV_IOCTL_MAGIC        'F'
#define NV_ESC_REGISTER_FD    201
#define NV_ESC_RM_MAP_MEMORY  0x4E
#define NV_ESC_RM_FREE        0x29
#define NV_ESC_RM_CONTROL     0x2A
#define NV_ESC_RM_ALLOC       0x2B

#define NV01_ROOT_CLIENT      0x00000041
#define NV01_DEVICE_0         0x00000080
#define NV20_SUBDEVICE_0      0x00002080
#define RM_USER_SHARED_DATA   0x000000de

#define NV0000_CTRL_CMD_GPU_GET_PROBED_IDS  0x00000214
#define NV0000_CTRL_CMD_GPU_ATTACH_IDS      0x00000215
#define NV0000_CTRL_CMD_GPU_GET_ID_INFO_V2  0x00000205
#define NV00DE_CTRL_CMD_REQUEST_DATA_POLL   0x00de0001

#define NV2080_CTRL_CMD_VOLT_VOLT_RAILS_GET_INFO    0x2080b201
#define NV2080_CTRL_CMD_VOLT_VOLT_RAILS_GET_STATUS  0x2080b202

#define NV2080_CTRL_CMD_GPU_EXEC_REG_OPS  0x20800122
#define NV2080_REG_OP_READ_32             0x00
#define NV2080_REG_OP_TYPE_GLOBAL         0x00

/* RUSD poll mask + layout (cl00de.h, verified) */
#define POLL_ALL           0x7f
#define RUSD_SIZE          3152
#define RUSD_TEMPS_OFF     2312
#define RUSD_TEMP_STRIDE   16
#define RUSD_TEMP_VALUE    8
#define RUSD_TEMP_COUNT    5
#define RUSD_TS_INVALID           0ull
#define RUSD_TS_WRITE_IN_PROGRESS 0xffffffffffffffffull
#define SENSOR_GPU    0
#define SENSOR_MEMORY 1

/* VOLT status layout (empirical) */
#define VOLT_RAIL_BASE   0x20
#define VOLT_RAIL_STRIDE 100
#define VOLT_CURR_OFF    0x08

/*
 * NV_THERM Hot Spot sensors, read from BAR0 through the RM EXEC_REG_OPS control
 * (NV2080_CTRL_CMD_GPU_EXEC_REG_OPS). RM performs the BAR0 register read on our
 * behalf over the /dev/nvidiactl fd we already hold, so — unlike an mmap of the
 * sysfs resource0 BAR — no kernel-lockdown (LOCKDOWN_PCI_ACCESS, usually enabled
 * by Secure Boot) or CONFIG_IO_STRICT_DEVMEM gate can make it unavailable.
 * Root-only in practice: RM holds non-root callers to a register allowlist that
 * excludes NV_THERM. We read only the small NV_THERM scan window, and the chip-id
 * gate below keeps the Blackwell-only offsets off any other architecture.
 */
#define NV_PMC_BOOT_0    0x0u          /* chip id = (boot0 >> 20); safe read on any GPU */

/* Column widths for the readout table. */
#define W_TEMP 10
#define W_VOLT 13

/*
 * Hot Spot = max over the on-die sensor array. There is a set of memor pages
 * where we've observed various entries to contain valid temperature readings,
 * while others are zero or contain a 0xBADF5040 marker. It is unclear how the
 * distribution of valid readings is established. It might be specific to
 * particular Blackwell GPU models, or even individual devices, but it doesn't
 * appear to be universally constant.
 *
 * So, we have an educated guess as to the start and end of the region, and will
 * look for valid temperatures within it. Note that there seem to be multiple
 * regions with semantically different temperature readings. I don't know for
 * sure what the others are, and have been excluding them from the tool. It's
 * possible that the scan will pull in some, and perhaps distort the final
 * hotspot calculation, but we'll learn something at least.
 *
 * Unpopulated slots read as zero or as the poison marker.
 */
#define NV_THERM_SCAN_FIRST   0xAD0A50u
#define NV_THERM_SCAN_LAST    0xAD0AFCu
#define NV_THERM_MAX_SENSORS  (((NV_THERM_SCAN_LAST - NV_THERM_SCAN_FIRST) / 4) + 1)

/* Plausibility bound on a decoded reading. */
#define NV_THERM_TEMP_MAX     130.0

#define NV_MAX_DEVICES        32
#define NV_PROC_NAME_MAX_LENGTH 100
#define NV_GPU_INVALID_ID     0xFFFFFFFFu

typedef uint32_t NvU32;
typedef int32_t  NvS32;
typedef uint32_t NvHandle;

typedef struct {
    NvHandle hRoot, hObjectParent, hObjectNew;
    NvU32    hClass;
    uint64_t pAllocParms;
    NvU32    paramsSize, status;
} NVOS21;

typedef struct {
    NvHandle hClient, hObject;
    NvU32    cmd, flags;
    uint64_t params;
    NvU32    paramsSize, status;
} NVOS54;

typedef struct {
    NvHandle hRoot, hObjectParent, hObjectOld;
    NvU32    status;
} NVOS00;

typedef struct {
    NvHandle hClient, hDevice, hMemory;
    uint64_t offset, length, pLinearAddress;
    NvU32    status, flags;
    int      fd;
    int      pad;
} NVOS33_FD;

typedef struct {
    NvHandle hClient;
    NvU32    processID;
    char     processName[NV_PROC_NAME_MAX_LENGTH];
    uint64_t pOsPidInfo;
} NV0000_ALLOC;

typedef struct {
    NvU32    deviceId;
    NvHandle hClientShare, hTargetClient, hTargetDevice;
    NvU32    flags;
    uint64_t vaSpaceSize, vaStartInternal, vaLimitInternal;
    NvU32    vaMode;
} NV0080_ALLOC;

typedef struct {
    NvU32 subDeviceId;
} NV2080_ALLOC;

/* EXEC_REG_OPS structs (ctrl2080gpu.h, intact — no NVML trace needed). */
typedef struct {
    uint8_t  regOp, regType, regStatus, regQuad;
    NvU32    regGroupMask, regSubGroupMask, regOffset;
    NvU32    regValueHi, regValueLo, regAndNMaskHi, regAndNMaskLo;
} NV2080_REG_OP;                          /* 32 B */

typedef struct {
    NvHandle hClientTarget, hChannelTarget;
    NvU32    bNonTransactional, reserved00[2], regOpCount;
    uint64_t regOps;                      /* NvP64 @ +24 */
    struct { NvU32 flags; uint64_t route; } grRouteInfo;   /* @ +32, 16 B */
} NV2080_EXEC_REG_OPS_PARAMS;             /* 48 B */

_Static_assert(sizeof(NV2080_REG_OP) == 32, "NV2080_REG_OP layout");
_Static_assert(sizeof(NV2080_EXEC_REG_OPS_PARAMS) == 48, "EXEC_REG_OPS params layout");

typedef struct {
    uint64_t polledDataMask;
} NV00DE_ALLOC;

typedef struct {
    NvU32 gpuIds[NV_MAX_DEVICES];
    NvU32 excludedGpuIds[NV_MAX_DEVICES];
    NvU32 gpuFlags[NV_MAX_DEVICES];
} PROBED_IDS;

typedef struct {
    NvU32 gpuIds[NV_MAX_DEVICES];
    NvU32 failedId;
} ATTACH_IDS;

typedef struct {
    NvU32 gpuId, gpuFlags, deviceInstance, subDeviceInstance;
    NvU32 sliStatus, boardId, gpuInstance;
    NvS32 numaId;
} IDINFO_V2;

static int g_fd = -1;

static int nv_ioctl(unsigned nr, void *arg, unsigned size)
{
    return ioctl(g_fd, _IOC(_IOC_READ | _IOC_WRITE, NV_IOCTL_MAGIC, nr, size), arg);
}

static int rm_alloc(NvHandle hRoot, NvHandle parent, NvHandle *pNew, NvU32 cls, void *params, NvU32 psize)
{
    NVOS21 p = {0};
    p.hRoot = hRoot;
    p.hObjectParent = parent;
    p.hObjectNew = *pNew;
    p.hClass = cls;
    p.pAllocParms = (uint64_t)(uintptr_t)params;
    p.paramsSize = psize;
    if (nv_ioctl(NV_ESC_RM_ALLOC, &p, sizeof p) != 0 || p.status != 0)
        return -1;
    *pNew = p.hObjectNew;
    return 0;
}

static int rm_control(NvHandle hClient, NvHandle hObject, NvU32 cmd, void *params, NvU32 psize)
{
    NVOS54 p = {0};
    p.hClient = hClient;
    p.hObject = hObject;
    p.cmd = cmd;
    p.params = (uint64_t)(uintptr_t)params;
    p.paramsSize = psize;
    if (nv_ioctl(NV_ESC_RM_CONTROL, &p, sizeof p) != 0 || p.status != 0)
        return -1;
    return 0;
}

/* Batch-read a set of memory offsets via the nvidia driver's EXEC_REG_OPS ioctl.
 * Although the addresses are interpreted as global addresses, don't expect this to
 * work for locations not claimed by the driver already.
 */
static int regops_read(NvHandle hClient, NvHandle hSubdev,
                       const NvU32 *offs, int n, NvU32 *vals)
{
    NV2080_REG_OP ops[NV_THERM_MAX_SENSORS + 1] = {0};
    if (n < 1 || n > (int)(sizeof ops / sizeof ops[0]))
        return -1;
    for (int i = 0; i < n; i++) {
        ops[i].regOp = NV2080_REG_OP_READ_32;
        ops[i].regType = NV2080_REG_OP_TYPE_GLOBAL;
        ops[i].regOffset = offs[i];
    }
    NV2080_EXEC_REG_OPS_PARAMS p = {0};
    p.bNonTransactional = 1;   /* a bad entry doesn't void the whole batch */
    p.regOpCount = n;
    p.regOps = (uint64_t)(uintptr_t)ops;
    if (rm_control(hClient, hSubdev, NV2080_CTRL_CMD_GPU_EXEC_REG_OPS, &p, sizeof p))
        return -1;
    /* We don't bother checking if the ioctl considered the read a success or not,
     * as we'll check the validity of the returned value in the caller.
     */
    for (int i = 0; i < n; i++)
        vals[i] = ops[i].regValueLo;
    return 0;
}

/* Read one RUSD temperature entry with the timestamp seqlock guard.
 * Returns 1 and sets *out (degC) on success, 0 if the sensor is unavailable. */
static int read_temp(volatile uint8_t *rusd, int sensor, double *out)
{
    const volatile uint8_t *e = rusd + RUSD_TEMPS_OFF + sensor * RUSD_TEMP_STRIDE;
    uint64_t ts = 0;
    int32_t raw = 0;
    for (int tries = 0; tries < 200; tries++) {
        ts = *(volatile uint64_t *)e;
        if (ts == RUSD_TS_WRITE_IN_PROGRESS) {
            usleep(1000);
            continue;
        }
        raw = *(volatile int32_t *)(e + RUSD_TEMP_VALUE);
        if (*(volatile uint64_t *)e == ts)
            break;
    }
    if (ts == RUSD_TS_INVALID)
        return 0;
    *out = raw / 256.0;
    return 1;
}

/*
 * Why the Hot Spot path is (un)available on a GPU. Read via RM EXEC_REG_OPS,
 * so the old mmap/lockdown failure modes are gone; what remains is root, the
 * ioctl itself, the architecture gate, and whether the scan window had readings.
 */
typedef enum {
    THERM_OK = 0,
    THERM_NOT_ROOT,       /* non-root: RM allowlist excludes NV_THERM offsets */
    THERM_REGOPS_FAILED,  /* NV2080_CTRL_CMD_GPU_EXEC_REG_OPS failed */
    THERM_NOT_BLACKWELL,  /* genuinely another architecture */
    THERM_NO_SENSORS,     /* Blackwell + read OK, but the scan window was empty */
} ThermStatus;

/* Per-GPU handles + mapped RUSD page, set up once and reused across samples. */
typedef struct {
    unsigned index;
    NvHandle hClient, hDevice, hSubdev, hRusd;
    volatile uint8_t *rusd;
    NvU32 railMask;
    unsigned sensors[NV_THERM_MAX_SENSORS];   /* populated slots, found by scan */
    int nSensors;
    NvU32 therm[NV_THERM_MAX_SENSORS];         /* last EXEC_REG_OPS read of the window */
    ThermStatus thermStatus;
    uint32_t boot0;           /* NV_PMC_BOOT_0 as read via regops */
} GPU;

static int is_blackwell(uint32_t boot0)
{
    unsigned chip = boot0 >> 20;
    return chip >= 0x1B0 && chip <= 0x1BF;   /* GB202/3/5/6/7 */
}

/* Index of a scan-window offset within g->therm[]. */
#define THERM_IDX(off)  (((off) - NV_THERM_SCAN_FIRST) / 4)

/* Decode an NV_THERM temperature word: (w & 0xFFFF)/256, tagged 0x4000.
 * Returns 1 on a valid reading. */
static int therm_temp(uint32_t w, double *out)
{
    if ((w >> 16) != 0x4000)
        return 0;
    unsigned b1 = (w >> 8) & 0xFF;
    if (b1 == 0 || b1 == 0xFF)
        return 0;
    double t = (double)(w & 0xFFFF) / 256.0;
    if (t > NV_THERM_TEMP_MAX)
        return 0;
    *out = t;
    return 1;
}

/* Re-read the whole NV_THERM scan window into g->therm via one regops batch. */
static int therm_refresh(GPU *g)
{
    NvU32 offs[NV_THERM_MAX_SENSORS];
    for (unsigned i = 0; i < NV_THERM_MAX_SENSORS; i++)
        offs[i] = NV_THERM_SCAN_FIRST + i * 4;
    return regops_read(g->hClient, g->hSubdev, offs, NV_THERM_MAX_SENSORS, g->therm);
}

/* Find the populated slots of the sensor array on this chip, from the current
 * g->therm snapshot: every slot holding a valid reading joins the max. */
static int therm_scan_sensors(GPU *g)
{
    int n = 0;
    for (unsigned i = 0; i < NV_THERM_MAX_SENSORS; i++) {
        double t;
        if (therm_temp(g->therm[i], &t))
            g->sensors[n++] = NV_THERM_SCAN_FIRST + i * 4;
    }
    return n;
}

/* Bring up the Hot Spot path for a GPU via EXEC_REG_OPS: check root, read the
 * chip id, gate on Blackwell, then scan the NV_THERM window for live sensors. */
static void therm_setup(GPU *g)
{
    g->nSensors = 0;
    g->boot0 = 0;

    if (geteuid() != 0) {
        g->thermStatus = THERM_NOT_ROOT;
        return;
    }

    /* NV_PMC_BOOT_0 @ +0x0 is a safe, allowlisted read on any GPU; gate on its
     * chip id so the Blackwell-only hotspot offsets are never read elsewhere. */
    NvU32 off0 = NV_PMC_BOOT_0, boot0 = 0;
    if (regops_read(g->hClient, g->hSubdev, &off0, 1, &boot0)) {
        g->thermStatus = THERM_REGOPS_FAILED;
        return;
    }
    g->boot0 = boot0;
    unsigned chip = boot0 >> 20;
    if (chip == 0x000 || chip == 0xFFF) {
        g->thermStatus = THERM_REGOPS_FAILED;
        return;
    }
    if (!is_blackwell(boot0)) {
        g->thermStatus = THERM_NOT_BLACKWELL;
        return;
    }
    if (therm_refresh(g)) {
        g->thermStatus = THERM_REGOPS_FAILED;
        return;
    }
    g->nSensors = therm_scan_sensors(g);
    g->thermStatus = g->nSensors ? THERM_OK : THERM_NO_SENSORS;
}

/* Render the Hot Spot availability reason. */
static void therm_explain(const GPU *g, char *buf, size_t n)
{
    switch (g->thermStatus) {
    case THERM_OK:
        snprintf(buf, n, "available via EXEC_REG_OPS (chip 0x%03X, %d sensor(s))",
                 g->boot0 >> 20, g->nSensors);
        break;
    case THERM_NOT_ROOT:
        snprintf(buf, n, "needs root");
        break;
    case THERM_REGOPS_FAILED:
        snprintf(buf, n, "EXEC_REG_OPS (NV2080_CTRL_CMD_GPU_EXEC_REG_OPS) failed on "
                         "this GPU (NV_PMC_BOOT_0 read back 0x%08X)", g->boot0);
        break;
    case THERM_NOT_BLACKWELL:
        snprintf(buf, n, "chip id 0x%03X (NV_PMC_BOOT_0 = 0x%08X) is not Blackwell (GB20x)",
                 g->boot0 >> 20, g->boot0);
        break;
    case THERM_NO_SENSORS:
        snprintf(buf, n, "chip 0x%03X read OK, but no valid readings in the NV_THERM "
                         "scan window 0x%06X..0x%06X — the sensor array may sit elsewhere "
                         "on this chip; please report --sensors output",
                 g->boot0 >> 20, NV_THERM_SCAN_FIRST, NV_THERM_SCAN_LAST);
        break;
    }
}

static void print_env_diagnostics(void)
{
    printf("environment:\n");
    printf("  euid            = %u%s\n", (unsigned)geteuid(),
           geteuid() == 0 ? "" : "   (Hot Spot needs root)");
    printf("\n");
}

/* Hot Spot = max over the sensors the scan found. Returns 1 if any read valid.
 * Re-reads the window via regops each call and re-validates every slot per
 * sample, so one going quiet just drops out. */
static int therm_hotspot(GPU *g, double *out)
{
    if (g->thermStatus != THERM_OK || therm_refresh(g))
        return 0;
    double mx = -1e9;
    for (int i = 0; i < g->nSensors; i++) {
        double t;
        if (therm_temp(g->therm[THERM_IDX(g->sensors[i])], &t) && t > mx)
            mx = t;
    }
    if (mx < -1e8)
        return 0;
    *out = mx;
    return 1;
}

/* colour helpers (temperature bands) */
static int g_color = 1;
static const char *temp_color(double c)
{
    if (!g_color)
        return "";
    if (c >= 85.0)
        return "\033[31m";      /* red    */
    if (c >= 70.0)
        return "\033[33m";      /* yellow */
    return "\033[32m";          /* green  */
}
static const char *C_RESET(void)
{
    return g_color ? "\033[0m" : "";
}
static const char *C_DIM(void)
{
    return g_color ? "\033[2m" : "";
}
static const char *C_BOLD(void)
{
    return g_color ? "\033[1m" : "";
}

/* Right-aligned, coloured temperature / voltage table cells (fixed visible width). */
static void cell_temp(int have, double t)
{
    if (have) {
        char b[16];
        snprintf(b, sizeof b, "%.1f C", t);
        printf("%s%*s%s", temp_color(t), W_TEMP, b, C_RESET());
    } else
        printf("%s%*s%s", C_DIM(), W_TEMP, "n/a", C_RESET());
}
static void cell_volt(int have, double v)
{
    if (have) {
        char b[16];
        snprintf(b, sizeof b, "%.3f V", v);
        printf("%*s", W_VOLT, b);
    } else
        printf("%s%*s%s", C_DIM(), W_VOLT, "n/a", C_RESET());
}

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s)
{
    (void)s;
    g_stop = 1;
}

/* Bring a single GPU up to a mapped RUSD page + VOLT rail mask + Hot Spot path. */
static int gpu_setup(GPU *g, NvHandle hClient, NvU32 gpuId, unsigned index)
{
    g->index = index;
    g->hClient = hClient;
    g->rusd = NULL;
    g->railMask = 0;
    g->nSensors = 0;
    g->thermStatus = THERM_REGOPS_FAILED;
    g->boot0 = 0;

    IDINFO_V2 idi = {0};
    idi.gpuId = gpuId;
    if (rm_control(hClient, hClient, NV0000_CTRL_CMD_GPU_GET_ID_INFO_V2, &idi, sizeof idi))
        return -1;

    char devnode[64];
    snprintf(devnode, sizeof devnode, "/dev/nvidia%u", idi.deviceInstance);
    int dev_fd = open(devnode, O_RDWR | O_CLOEXEC);
    if (dev_fd >= 0) {
        struct { int ctl_fd; } reg = { g_fd };
        ioctl(dev_fd, _IOC(_IOC_READ|_IOC_WRITE, NV_IOCTL_MAGIC, NV_ESC_REGISTER_FD, sizeof reg), &reg);
    }

    g->hDevice = 0xCB000002 + index * 0x10;
    NV0080_ALLOC da = {0};
    da.deviceId = idi.deviceInstance;
    if (rm_alloc(hClient, hClient, &g->hDevice, NV01_DEVICE_0, &da, sizeof da))
        return -1;
    g->hSubdev = 0xCB000003 + index * 0x10;
    NV2080_ALLOC sa = {0};
    sa.subDeviceId = idi.subDeviceInstance;
    if (rm_alloc(hClient, g->hDevice, &g->hSubdev, NV20_SUBDEVICE_0, &sa, sizeof sa))
        return -1;

    /* RUSD page for temperatures */
    g->hRusd = 0xCB000004 + index * 0x10;
    NV00DE_ALLOC ra = { .polledDataMask = POLL_ALL };
    if (rm_alloc(hClient, g->hSubdev, &g->hRusd, RM_USER_SHARED_DATA, &ra, sizeof ra) == 0) {
        int mapfd = open("/dev/nvidiactl", O_RDWR | O_CLOEXEC);
        NVOS33_FD m = {0};
        m.hClient = hClient;
        m.hDevice = g->hDevice;
        m.hMemory = g->hRusd;
        m.offset = 0;
        m.length = RUSD_SIZE;
        m.fd = mapfd;
        if (nv_ioctl(NV_ESC_RM_MAP_MEMORY, &m, sizeof m) == 0 && m.status == 0) {
            void *p = mmap(NULL, RUSD_SIZE, PROT_READ, MAP_SHARED, mapfd, 0);
            if (p != MAP_FAILED)
                g->rusd = p;
        }
        NvU32 pollMask[2] = { POLL_ALL, 0 };
        rm_control(hClient, g->hRusd, NV00DE_CTRL_CMD_REQUEST_DATA_POLL, pollMask, sizeof pollMask);
    }

    /* VOLT rail enumeration (static): learn the populated-rail mask once */
    static uint8_t info[2444];
    if (rm_control(hClient, g->hSubdev, NV2080_CTRL_CMD_VOLT_VOLT_RAILS_GET_INFO, info, sizeof info) == 0)
        g->railMask = ((NvU32 *)info)[1];

    /* Hot Spot path (EXEC_REG_OPS): chip id + one-time NV_THERM sensor scan */
    therm_setup(g);

    return 0;
}

/* Sample one GPU and print its row. */
static void gpu_print(GPU *g)
{
    double tgpu = 0, tmem = 0, thot = 0;
    int have_gpu = g->rusd && read_temp(g->rusd, SENSOR_GPU, &tgpu);
    int have_mem = g->rusd && read_temp(g->rusd, SENSOR_MEMORY, &tmem);
    int have_hot = therm_hotspot(g, &thot);

    /* rail voltages */
    NvU32 v0 = 0, v1 = 0;
    int have_v0 = 0, have_v1 = 0;
    if (g->railMask) {
        static uint8_t status[3232];
        ((NvU32 *)status)[1] = g->railMask;
        if (rm_control(g->hClient, g->hSubdev, NV2080_CTRL_CMD_VOLT_VOLT_RAILS_GET_STATUS,
                       status, sizeof status) == 0) {
            if (g->railMask & (1u << 0)) {
                v0 = *(NvU32 *)(status + VOLT_RAIL_BASE + 0 * VOLT_RAIL_STRIDE + VOLT_CURR_OFF);
                have_v0 = 1;
            }
            if (g->railMask & (1u << 1)) {
                v1 = *(NvU32 *)(status + VOLT_RAIL_BASE + 1 * VOLT_RAIL_STRIDE + VOLT_CURR_OFF);
                have_v1 = 1;
            }
        }
    }

    printf("  %s%-4u%s", C_BOLD(), g->index, C_RESET());
    printf("  ");
    cell_temp(have_gpu, tgpu);
    printf("  ");
    cell_temp(have_mem, tmem);
    printf("  ");
    cell_temp(have_hot, thot);
    printf("  ");
    cell_volt(have_v0, v0 / 1e6);
    printf("  ");
    cell_volt(have_v1, v1 / 1e6);
    printf("\n");
}

int main(int argc, char **argv)
{
    int watch = 0, list_sensors = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--watch") || !strcmp(argv[i], "-w"))
            watch = 1;
        else if (!strcmp(argv[i], "--no-color"))
            g_color = 0;
        else if (!strcmp(argv[i], "--sensors"))
            list_sensors = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("usage: %s [--watch|-w] [--no-color] [--sensors]\n", argv[0]);
            printf("  shows GPU/memory/hot-spot temperature and both rail voltages per GPU.\n");
            printf("  Hot Spot needs root, degrades to n/a otherwise.\n");
            printf("  --sensors dumps the raw NV_THERM sensor array and which slots the scan\n");
            printf("            accepted, plus why Hot Spot is unavailable when it is.\n");
            printf("            Attach its output to a bug report.\n");
            return 0;
        }
    }
    if (g_color && !isatty(1))
        g_color = 0;

    g_fd = open("/dev/nvidiactl", O_RDWR | O_CLOEXEC);
    if (g_fd < 0) {
        fprintf(stderr, "open /dev/nvidiactl: %s\n", strerror(errno));
        return 1;
    }

    NvHandle hClient = 0xCB000001;
    NV0000_ALLOC ca = {0};
    ca.hClient = hClient;
    ca.processID = getpid();
    snprintf(ca.processName, sizeof ca.processName, "nvidia-gpu-sensors");
    if (rm_alloc(0, 0, &hClient, NV01_ROOT_CLIENT, &ca, sizeof ca)) {
        fprintf(stderr, "failed to create RM client\n");
        return 1;
    }

    static PROBED_IDS probed;
    if (rm_control(hClient, hClient, NV0000_CTRL_CMD_GPU_GET_PROBED_IDS, &probed, sizeof probed)) {
        fprintf(stderr, "failed to probe GPUs\n");
        return 1;
    }

    /* attach every probed GPU */
    static ATTACH_IDS att;
    for (int i = 0; i < NV_MAX_DEVICES; i++)
        att.gpuIds[i] = NV_GPU_INVALID_ID;
    int nAttach = 0;
    for (int i = 0; i < NV_MAX_DEVICES; i++)
        if (probed.gpuIds[i] != NV_GPU_INVALID_ID)
            att.gpuIds[nAttach++] = probed.gpuIds[i];
    if (nAttach)
        rm_control(hClient, hClient, NV0000_CTRL_CMD_GPU_ATTACH_IDS, &att, sizeof att);

    static GPU gpus[NV_MAX_DEVICES];
    int nGpu = 0;
    for (int i = 0; i < NV_MAX_DEVICES; i++) {
        if (probed.gpuIds[i] == NV_GPU_INVALID_ID)
            continue;
        if (gpu_setup(&gpus[nGpu], hClient, probed.gpuIds[i], (unsigned)nGpu) == 0)
            nGpu++;
    }
    if (nGpu == 0) {
        fprintf(stderr, "no GPUs detected\n");
        return 1;
    }

    /* One precise note per GPU whose Hot Spot column will read n/a. --sensors
     * reports the same thing inline, so don't say it twice there. */
    for (int i = 0; !list_sensors && i < nGpu; i++) {
        char why[512];
        if (gpus[i].thermStatus == THERM_OK)
            continue;
        therm_explain(&gpus[i], why, sizeof why);
        fprintf(stderr, "note: GPU %u Hot Spot unavailable: %s\n", gpus[i].index, why);
    }

    if (list_sensors) {
        print_env_diagnostics();
        for (int i = 0; i < nGpu; i++) {
            char why[512];
            therm_explain(&gpus[i], why, sizeof why);
            printf("GPU %u: NV_THERM scan 0x%06X..0x%06X, %d sensor(s)\n",
                   gpus[i].index,
                   NV_THERM_SCAN_FIRST, NV_THERM_SCAN_LAST, gpus[i].nSensors);
            printf("  Hot Spot: %s\n", why);

            /* Dump the raw window only when we legitimately read it (Blackwell,
             * root): NOT_BLACKWELL never touches these offsets by design. */
            if (gpus[i].thermStatus != THERM_OK && gpus[i].thermStatus != THERM_NO_SENSORS)
                continue;
            printf("  NV_PMC_BOOT_0 = 0x%08X (chip 0x%03X)\n",
                   gpus[i].boot0, gpus[i].boot0 >> 20);
            if (therm_refresh(&gpus[i])) {
                printf("  (re-read of NV_THERM window via EXEC_REG_OPS failed)\n");
                continue;
            }
            for (unsigned k = 0; k < NV_THERM_MAX_SENSORS; k++) {
                unsigned off = NV_THERM_SCAN_FIRST + k * 4;
                uint32_t w = gpus[i].therm[k];
                double t;
                int used = 0;
                for (int s = 0; s < gpus[i].nSensors; s++)
                    if (gpus[i].sensors[s] == off)
                        used = 1;
                printf("  0x%06X  %08X  %s", off, w, used ? "->" : "  ");
                if (therm_temp(w, &t))
                    printf("  %.2f C\n", t);
                else
                    printf("  --\n");
            }
        }
        return 0;
    }

    signal(SIGINT, on_sigint);

    do {
        if (watch)
            printf("\033[H\033[2J");   /* home + clear */
        printf("%s  %-4s  %*s  %*s  %*s  %*s  %*s%s\n", C_BOLD(), "GPU",
               W_TEMP, "Core Temp", W_TEMP, "Mem Temp", W_TEMP, "Hot Spot",
               W_VOLT, "NVVDD", W_VOLT, "MSVDD", C_RESET());
        printf("%s  %-4s  %*s  %*s  %*s  %*s  %*s%s\n", C_DIM(), "----",
               W_TEMP, "----------", W_TEMP, "----------", W_TEMP, "----------",
               W_VOLT, "-------------", W_VOLT, "-------------", C_RESET());
        for (int i = 0; i < nGpu; i++)
            gpu_print(&gpus[i]);
        fflush(stdout);
        if (watch && !g_stop)
            usleep(1000000);
    } while (watch && !g_stop);

    for (int i = 0; i < nGpu; i++) {
        if (gpus[i].rusd)
            munmap((void *)gpus[i].rusd, RUSD_SIZE);
    }
    NVOS00 f = {0};
    f.hRoot = hClient;
    f.hObjectOld = hClient;
    nv_ioctl(NV_ESC_RM_FREE, &f, sizeof f);
    close(g_fd);
    return 0;
}
