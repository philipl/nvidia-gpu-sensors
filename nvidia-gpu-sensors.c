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
#define NV_THERM_SCAN_FIRST   0xAD0A80u
#define NV_THERM_SCAN_LAST    0xAD0AFCu
#define NV_THERM_MAX_SENSORS  (((NV_THERM_SCAN_LAST - NV_THERM_SCAN_FIRST) / 4) + 1)

/* Plausibility bound on a decoded reading. */
#define NV_THERM_TEMP_MAX     130.0

/*
 * GDDR7 per-module DRAM temperature (Blackwell), from the FBPA DQR status
 * registers. Each frame-buffer partition has its own copy of the block at
 *
 *     NV_PFB_FBPA_<n>_DQR_STATUS_DQ_IC<i>_SUBP<s> = 0x009024C0 + n*0x4000 + (i*2+s)*4
 *     NV_PFB_FBPA_<n>_DQR_STATUS_VLD              = 0x009024D0 + n*0x4000
 *
 * 0x00900000 is the FBPA unicast window; the 0x009A24C0 aliases are the
 * broadcast copies. This is what the FBFALCON firmware polls to get the
 * DRAM-reported temperature, and unlike the documented mem-temp register
 * 0x9A44B0 it is not PLM-locked, so an unprivileged BAR0 read would work too.
 *
 * Each DQ word carries a GDDR temperature mode-register code replicated across
 * all four bytes of that word (per DRAM channel; the bytes have never been
 * observed to disagree). Each of the four DQ slots of a partition — indexed
 * (IC, SUBP), i.e. IC0_SUBP0, IC0_SUBP1, IC1_SUBP0, IC1_SUBP1 — is an
 * independent temperature readback, live when its VLD bit (24..27, one per
 * slot) is set; an unpopulated partition reads back the 0xBADF.... poison
 * marker in every register of the block (VLD and DQ alike).
 *
 * How many of those four slots are *distinct* memory chips depends on the
 * board's DRAM organisation, so a module is one live DQ slot and the board's
 * chip count is what the slots turn out to be:
 *
 *   - RTX 5090 (GB202, 32 GB, 512-bit): 8 populated FBPAs, each driving two
 *     GDDR7 devices — one per subpartition — and the two ICs are duplicate
 *     readback paths of the same device: across 320 sampled partition groups
 *     the pairs always agreed, IC0_SUBP0 == IC1_SUBP0 and IC0_SUBP1 ==
 *     IC1_SUBP1, never any other way. Its 4 live slots per FBPA therefore
 *     collapse to 2 unique chips (16 total); the aliased IC slots appear as
 *     modules with identical readings.
 *
 *   - RTX Pro 6000 Blackwell (GB202, 96 GB, 512-bit): 8 populated FBPAs, each
 *     driving four GDDR7 devices — one per (IC, SUBP) slot. Measured on this
 *     card: the driver reports PARTITION_COUNT = 8 (all of them, no further
 *     FBPAs exist), and across 300 samples the IC0/IC1 pairs of every
 *     subpartition disagreed in 100% of samples with a wandering +1..+6 code
 *     offset, i.e. all 32 slots are distinct chips.
 *
 * The registers were found by the gddr6 project
 * (https://github.com/olealgoritme/gddr6); we read them through EXEC_REG_OPS
 * rather than an mmap of BAR0.
 */
#define FBPA_DQR_DQ           0x009024C0u
#define FBPA_DQR_VLD          0x009024D0u
#define FBPA_STRIDE           0x00004000u
#define FBPA_MAX_PARTITIONS   16
#define FBPA_SUBPARTITIONS    2       /* SUBP slots per IC */
#define FBPA_ICS              2       /* IC channels per partition */
#define FBPA_DQR_REGS         (FBPA_ICS * FBPA_SUBPARTITIONS)  /* DQ slots per FBPA = 4 */
#define FBPA_MODULES_PER_PART (FBPA_DQR_REGS)  /* one module per DQ slot */
#define FBPA_MAX_MODULES      (FBPA_MAX_PARTITIONS * FBPA_MODULES_PER_PART)

/* DQ slot index within a partition's DQR block (0 .. FBPA_DQR_REGS-1). */
#define DQR_SLOT(ic, subp)    ((ic) * FBPA_SUBPARTITIONS + (subp))
#define MOD_FBPA(m)           ((m) / FBPA_MODULES_PER_PART)
#define MOD_SLOT(m)           ((m) % FBPA_MODULES_PER_PART)

/* GDDR temperature MR code: 2 C per step, code 20 == 0 C. */
#define GDDR_MRCODE_MIN       10      /* -20 C */
#define GDDR_MRCODE_MAX       80      /* 120 C */

#define REGOPS_MAX_OPS        (FBPA_MAX_PARTITIONS * (FBPA_DQR_REGS + 1))
_Static_assert(REGOPS_MAX_OPS >= NV_THERM_MAX_SENSORS, "regops batch fits the NV_THERM scan");

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

typedef struct {
    uint8_t  regOp, regType, regStatus, regQuad;
    NvU32    regGroupMask, regSubGroupMask, regOffset;
    NvU32    regValueHi, regValueLo, regAndNMaskHi, regAndNMaskLo;
} NV2080_REG_OP;

typedef struct {
    NvHandle hClientTarget, hChannelTarget;
    NvU32    bNonTransactional, reserved00[2], regOpCount;
    uint64_t regOps;
    struct { NvU32 flags; uint64_t route; } grRouteInfo;
} NV2080_EXEC_REG_OPS_PARAMS;

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

/* Batch-read global memory offsets via EXEC_REG_OPS ioctl. */
static int regops_read(NvHandle hClient, NvHandle hSubdev,
                       const NvU32 *offs, int n, NvU32 *vals)
{
    NV2080_REG_OP ops[REGOPS_MAX_OPS] = {0};
    if (n < 1 || n > (int)(sizeof ops / sizeof ops[0]))
        return -1;
    for (int i = 0; i < n; i++) {
        ops[i].regOp = NV2080_REG_OP_READ_32;
        ops[i].regType = NV2080_REG_OP_TYPE_GLOBAL;
        ops[i].regOffset = offs[i];
    }
    NV2080_EXEC_REG_OPS_PARAMS p = {0};
    p.bNonTransactional = 1;
    p.regOpCount = n;
    p.regOps = (uint64_t)(uintptr_t)ops;
    if (rm_control(hClient, hSubdev, NV2080_CTRL_CMD_GPU_EXEC_REG_OPS, &p, sizeof p))
        return -1;
    /* Caller validates returned values. */
    for (int i = 0; i < n; i++)
        vals[i] = ops[i].regValueLo;
    return 0;
}

/* Read one RUSD temperature entry with seqlock guard; returns 1 on valid reading. */
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

/* Raw-register path availability status. */
typedef enum {
    RAW_OK = 0,
    RAW_NOT_ROOT,       /* non-root: RM allowlist excludes these offsets */
    RAW_REGOPS_FAILED,  /* NV2080_CTRL_CMD_GPU_EXEC_REG_OPS failed */
    RAW_NOT_BLACKWELL,  /* genuinely another architecture */
    RAW_NO_SENSORS,     /* Blackwell + read OK, but the scanned window was empty */
} RawStatus;

typedef struct {
    unsigned index;
    NvHandle hClient, hDevice, hSubdev, hRusd;
    volatile uint8_t *rusd;
    NvU32 railMask;
    unsigned sensors[NV_THERM_MAX_SENSORS];   /* populated slots, found by scan */
    int nSensors;
    NvU32 therm[NV_THERM_MAX_SENSORS];         /* last EXEC_REG_OPS read of the window */
    RawStatus thermStatus;
    unsigned mods[FBPA_MAX_MODULES];           /* populated memory modules */
    int nMods;
    NvU32 dq[FBPA_MAX_PARTITIONS][FBPA_DQR_REGS];  /* last read of DQR_STATUS_DQ_* */
    NvU32 vld[FBPA_MAX_PARTITIONS];                /* last read of DQR_STATUS_VLD  */
    RawStatus gddrStatus;
    uint32_t boot0;           /* NV_PMC_BOOT_0 as read via regops */
} GPU;

static int is_blackwell(uint32_t boot0)
{
    unsigned chip = boot0 >> 20;
    return chip >= 0x1B0 && chip <= 0x1BF;   /* GB202/3/5/6/7 */
}

#define THERM_IDX(off)  (((off) - NV_THERM_SCAN_FIRST) / 4)

/* Decode an NV_THERM temperature word: (w & 0xFFFF)/256, tagged 0x4000. */
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

static int therm_refresh(GPU *g)
{
    NvU32 offs[NV_THERM_MAX_SENSORS];
    for (unsigned i = 0; i < NV_THERM_MAX_SENSORS; i++)
        offs[i] = NV_THERM_SCAN_FIRST + i * 4;
    return regops_read(g->hClient, g->hSubdev, offs, NV_THERM_MAX_SENSORS, g->therm);
}

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

static RawStatus raw_gate(GPU *g)
{
    g->boot0 = 0;

    if (geteuid() != 0)
        return RAW_NOT_ROOT;

    /* Safe, allowlisted read on any GPU; gates Blackwell-only offsets below. */
    NvU32 off0 = NV_PMC_BOOT_0, boot0 = 0;
    if (regops_read(g->hClient, g->hSubdev, &off0, 1, &boot0))
        return RAW_REGOPS_FAILED;
    g->boot0 = boot0;
    unsigned chip = boot0 >> 20;
    if (chip == 0x000 || chip == 0xFFF)
        return RAW_REGOPS_FAILED;
    if (!is_blackwell(boot0))
        return RAW_NOT_BLACKWELL;
    return RAW_OK;
}

static void therm_setup(GPU *g, RawStatus gate)
{
    g->nSensors = 0;
    g->thermStatus = gate;
    if (gate != RAW_OK)
        return;

    if (therm_refresh(g)) {
        g->thermStatus = RAW_REGOPS_FAILED;
        return;
    }
    g->nSensors = therm_scan_sensors(g);
    g->thermStatus = g->nSensors ? RAW_OK : RAW_NO_SENSORS;
}

static int gddr_code_temp(uint32_t dq, double *out)
{
    if ((dq & 0xFFFF0000u) == 0xBADF0000u)      /* unpopulated/unpowered partition */
        return 0;
    unsigned code = (dq >> 16) & 0xFF;
    if (code < GDDR_MRCODE_MIN || code > GDDR_MRCODE_MAX)
        return 0;
    *out = ((double)code - 20.0) * 2.0;
    return 1;
}

/*
 * Temperature of one memory module = one DQ slot (one IC x SUBP readback) of
 * one partition. The slot is live when its VLD bit (24+slot) is set; unpopulated
 * partitions read the 0xBADF.... poison back in the DQ word, which
 * gddr_code_temp rejects as well.
 */
static int gddr_module_temp(const GPU *g, unsigned m, double *out)
{
    unsigned f = MOD_FBPA(m), slot = MOD_SLOT(m);
    if (!((g->vld[f] >> (24 + slot)) & 1))      /* this slot not populated */
        return 0;
    return gddr_code_temp(g->dq[f][slot], out);
}

static int gddr_refresh(GPU *g)
{
    NvU32 offs[REGOPS_MAX_OPS], vals[REGOPS_MAX_OPS];
    int n = 0;
    for (unsigned f = 0; f < FBPA_MAX_PARTITIONS; f++) {
        for (unsigned k = 0; k < FBPA_DQR_REGS; k++)
            offs[n++] = FBPA_DQR_DQ + f * FBPA_STRIDE + k * 4;
        offs[n++] = FBPA_DQR_VLD + f * FBPA_STRIDE;
    }
    if (regops_read(g->hClient, g->hSubdev, offs, n, vals))
        return -1;
    n = 0;
    for (unsigned f = 0; f < FBPA_MAX_PARTITIONS; f++) {
        for (unsigned k = 0; k < FBPA_DQR_REGS; k++)
            g->dq[f][k] = vals[n++];
        g->vld[f] = vals[n++];
    }
    return 0;
}

static int gddr_scan_modules(GPU *g)
{
    int n = 0;
    for (unsigned m = 0; m < FBPA_MAX_MODULES; m++) {
        double t;
        if (gddr_module_temp(g, m, &t))
            g->mods[n++] = m;
    }
    return n;
}

static void gddr_setup(GPU *g, RawStatus gate)
{
    g->nMods = 0;
    g->gddrStatus = gate;
    if (gate != RAW_OK)
        return;

    if (gddr_refresh(g)) {
        g->gddrStatus = RAW_REGOPS_FAILED;
        return;
    }
    g->nMods = gddr_scan_modules(g);
    g->gddrStatus = g->nMods ? RAW_OK : RAW_NO_SENSORS;
}

static void raw_explain(const GPU *g, RawStatus st, const char *what,
                        unsigned first, unsigned last, int found,
                        char *buf, size_t n)
{
    switch (st) {
    case RAW_OK:
        snprintf(buf, n, "available via EXEC_REG_OPS (chip 0x%03X, %d %s)",
                 g->boot0 >> 20, found, what);
        break;
    case RAW_NOT_ROOT:
        snprintf(buf, n, "needs root");
        break;
    case RAW_REGOPS_FAILED:
        snprintf(buf, n, "EXEC_REG_OPS (NV2080_CTRL_CMD_GPU_EXEC_REG_OPS) failed on "
                         "this GPU (NV_PMC_BOOT_0 read back 0x%08X)", g->boot0);
        break;
    case RAW_NOT_BLACKWELL:
        snprintf(buf, n, "chip id 0x%03X (NV_PMC_BOOT_0 = 0x%08X) is not Blackwell (GB20x)",
                 g->boot0 >> 20, g->boot0);
        break;
    case RAW_NO_SENSORS:
        snprintf(buf, n, "chip 0x%03X read OK, but no valid readings in the scan window "
                         "0x%06X..0x%06X — the %s may sit elsewhere on this chip; "
                         "please report --sensors output",
                 g->boot0 >> 20, first, last, what);
        break;
    }
}

static void therm_explain(const GPU *g, char *buf, size_t n)
{
    raw_explain(g, g->thermStatus, "sensor(s)",
                NV_THERM_SCAN_FIRST, NV_THERM_SCAN_LAST, g->nSensors, buf, n);
}

static void gddr_explain(const GPU *g, char *buf, size_t n)
{
    raw_explain(g, g->gddrStatus, "memory module(s)", FBPA_DQR_DQ,
                FBPA_DQR_VLD + (FBPA_MAX_PARTITIONS - 1) * FBPA_STRIDE,
                g->nMods, buf, n);
}

static void therm_dump(GPU *g)
{
    if (g->thermStatus != RAW_OK && g->thermStatus != RAW_NO_SENSORS)
        return;
    printf("  NV_PMC_BOOT_0 = 0x%08X (chip 0x%03X)\n", g->boot0, g->boot0 >> 20);
    if (therm_refresh(g)) {
        printf("  (re-read of NV_THERM window via EXEC_REG_OPS failed)\n");
        return;
    }
    for (unsigned k = 0; k < NV_THERM_MAX_SENSORS; k++) {
        unsigned off = NV_THERM_SCAN_FIRST + k * 4;
        uint32_t w = g->therm[k];
        double t;
        int used = 0;
        for (int s = 0; s < g->nSensors; s++)
            if (g->sensors[s] == off)
                used = 1;
        printf("  0x%06X  %08X  %s", off, w, used ? "->" : "  ");
        if (therm_temp(w, &t))
            printf("  %.2f C\n", t);
        else
            printf("  --\n");
    }
}

/* True if the partition has at least one live DQ slot (same test as the module scan). */
static int gddr_partition_live(const GPU *g, unsigned f)
{
    for (unsigned slot = 0; slot < FBPA_DQR_REGS; slot++) {
        double t;
        if (gddr_module_temp(g, f * FBPA_MODULES_PER_PART + slot, &t))
            return 1;
    }
    return 0;
}

static void gddr_dump(GPU *g)
{
    if (g->gddrStatus != RAW_OK && g->gddrStatus != RAW_NO_SENSORS)
        return;
    if (gddr_refresh(g)) {
        printf("  (re-read of the FBPA DQR registers via EXEC_REG_OPS failed)\n");
        return;
    }
    printf("  %-8s %-8s %-8s %-8s %-8s %-8s %s\n", "DQ base", "IC0_S0",
           "IC0_S1", "IC1_S0", "IC1_S1", "VLD", "modules");
    static unsigned omitted[FBPA_MAX_PARTITIONS];
    unsigned nOmitted = 0;
    for (unsigned f = 0; f < FBPA_MAX_PARTITIONS; f++) {
        if (!gddr_partition_live(g, f)) {
            /* no live slots: unpopulated (or poison) partition, omit the row */
            omitted[nOmitted++] = f;
            continue;
        }
        printf("  %06X  ", FBPA_DQR_DQ + f * FBPA_STRIDE);
        for (unsigned k = 0; k < FBPA_DQR_REGS; k++)
            printf(" %08X", g->dq[f][k]);
        printf(" %08X ", g->vld[f]);
        for (unsigned slot = 0; slot < FBPA_DQR_REGS; slot++) {
            unsigned m = f * FBPA_MODULES_PER_PART + slot;
            double t;
            int used = 0;
            for (int i = 0; i < g->nMods; i++)
                if (g->mods[i] == m)
                    used = 1;
            if (gddr_module_temp(g, m, &t))
                printf("  %s m%-2u %.0f C", used ? "->" : "  ", m, t);
            else
                printf("     m%-2u --   ", m);
        }
        printf("\n");
    }
    /* one summary for the omitted partitions (indices only, no registers) */
    if (nOmitted) {
        unsigned i = 0;
        int first = 1;
        printf("  (no live DQR slots in partition%s", nOmitted > 1 ? "s" : "");
        while (i < nOmitted) {
            unsigned a = omitted[i], b = a;
            while (i + 1 < nOmitted && omitted[i + 1] == b + 1) {
                i++;
                b = omitted[i];
            }
            printf("%s %u%s%u", first ? "" : ",", a, b > a ? "-" : "", b);
            first = 0;
            i++;
        }
        printf(")\n");
    }
}

static void print_env_diagnostics(void)
{
    printf("environment:\n");
    printf("  euid            = %u%s\n", (unsigned)geteuid(),
           geteuid() == 0 ? "" : "   (Hot Spot needs root)");
    printf("\n");
}

static int therm_hotspot(GPU *g, double *out)
{
    if (g->thermStatus != RAW_OK || therm_refresh(g))
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

static int gpu_setup(GPU *g, NvHandle hClient, NvU32 gpuId, unsigned index)
{
    g->index = index;
    g->hClient = hClient;
    g->rusd = NULL;
    g->railMask = 0;
    g->nSensors = 0;
    g->nMods = 0;
    g->thermStatus = RAW_REGOPS_FAILED;
    g->gddrStatus = RAW_REGOPS_FAILED;
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

    /* EXEC_REG_OPS: chip id gate, then one-time scan. */
    RawStatus gate = raw_gate(g);
    therm_setup(g, gate);
    gddr_setup(g, gate);

    return 0;
}

/* One sampled set of sensor readings for a single GPU. */
typedef struct {
    double tgpu, tmem, thot;
    double v0, v1;
    int have_gpu, have_mem, have_hot, have_v0, have_v1;
} Sample;

/* Sample one GPU's sensors. */
static void gpu_sample(GPU *g, Sample *s)
{
    memset(s, 0, sizeof *s);
    s->have_gpu = g->rusd && read_temp(g->rusd, SENSOR_GPU, &s->tgpu);
    s->have_mem = g->rusd && read_temp(g->rusd, SENSOR_MEMORY, &s->tmem);
    s->have_hot = therm_hotspot(g, &s->thot);

    /* rail voltages */
    if (g->railMask) {
        static uint8_t status[3232];
        ((NvU32 *)status)[1] = g->railMask;
        if (rm_control(g->hClient, g->hSubdev, NV2080_CTRL_CMD_VOLT_VOLT_RAILS_GET_STATUS,
                       status, sizeof status) == 0) {
            if (g->railMask & (1u << 0)) {
                s->v0 = *(NvU32 *)(status + VOLT_RAIL_BASE + 0 * VOLT_RAIL_STRIDE + VOLT_CURR_OFF) / 1e6;
                s->have_v0 = 1;
            }
            if (g->railMask & (1u << 1)) {
                s->v1 = *(NvU32 *)(status + VOLT_RAIL_BASE + 1 * VOLT_RAIL_STRIDE + VOLT_CURR_OFF) / 1e6;
                s->have_v1 = 1;
            }
        }
    }
}

/* Query flags: which single metric to print, and whether to append units. */
static const char *g_query = NULL;
static int g_suffix = 0;

/* Print one metric as a bare value. Returns 0 if the metric is unavailable. */
static int query_print(const Sample *s)
{
    const char *val = NULL;
    int have = 0;
    char buf[16];

    if (!strcmp(g_query, "gpu")) {
        have = s->have_gpu;
        snprintf(buf, sizeof buf, "%.1f", s->tgpu);
        val = buf;
    } else if (!strcmp(g_query, "mem")) {
        have = s->have_mem;
        snprintf(buf, sizeof buf, "%.1f", s->tmem);
        val = buf;
    } else if (!strcmp(g_query, "hot")) {
        have = s->have_hot;
        snprintf(buf, sizeof buf, "%.1f", s->thot);
        val = buf;
    } else if (!strcmp(g_query, "nvvdd")) {
        have = s->have_v0;
        snprintf(buf, sizeof buf, "%.3f", s->v0);
        val = buf;
    } else if (!strcmp(g_query, "msvdd")) {
        have = s->have_v1;
        snprintf(buf, sizeof buf, "%.3f", s->v1);
        val = buf;
    }

    if (!have || !val)
        return 0;
    printf("%s", val);
    if (g_suffix)
        printf("%s", !strcmp(g_query, "nvvdd") || !strcmp(g_query, "msvdd") ? " V" : " C");
    printf("\n");
    return 1;
}

/* Sample one GPU and print its row. */
static void gpu_print(GPU *g)
{
    Sample s;
    gpu_sample(g, &s);

    printf("  %s%-4u%s", C_BOLD(), g->index, C_RESET());
    printf("  ");
    cell_temp(s.have_gpu, s.tgpu);
    printf("  ");
    cell_temp(s.have_hot, s.thot);
    printf("  ");
    cell_temp(s.have_mem, s.tmem);
    printf("  ");
    cell_volt(s.have_v0, s.v0);
    printf("  ");
    cell_volt(s.have_v1, s.v1);
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
        else if (!strcmp(argv[i], "--suffix"))
            g_suffix = 1;
        else if (!strcmp(argv[i], "--query") && i + 1 < argc)
            g_query = argv[++i];
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("usage: %s [--watch|-w] [--no-color] [--sensors] "
                   "[--query METRIC] [--suffix]\n", argv[0]);
            printf("  shows GPU/memory/hot-spot temperature and both rail voltages per GPU.\n");
            printf("  Hot Spot is the hottest on-die sensor. Mem Temp is already a\n");
            printf("  memory hot spot reading from the driver. Both need root and Blackwell,\n");
            printf("  and read n/a otherwise.\n");
            printf("  --query METRIC prints a single value (no table) instead:\n");
            printf("            gpu|mem|hot  temperature, nvvdd|msvdd  voltage.\n");
            printf("  --suffix    append units (C/V) to a --query value.\n");
            printf("  --sensors     dumps the raw NV_THERM sensor array and the FBPA DQR\n");
            printf("                memory registers, which slots the scans accepted, and\n");
            printf("                why a reading is unavailable when it is.\n");
            printf("                Attach its output to a bug report.\n");
            return 0;
        }
    }
    if (g_color && !isatty(1))
        g_color = 0;

    if (g_query &&
        strcmp(g_query, "gpu") && strcmp(g_query, "mem") && strcmp(g_query, "hot") &&
        strcmp(g_query, "nvvdd") && strcmp(g_query, "msvdd")) {
        fprintf(stderr, "unknown --query metric '%s' (gpu|mem|hot|nvvdd|msvdd)\n", g_query);
        return 1;
    }

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

    /* Warn about unavailable raw-register paths (--sensors duplicates this). */
    for (int i = 0; !list_sensors && !g_query && i < nGpu; i++) {
        char why[512];
        if (gpus[i].thermStatus != RAW_OK) {
            therm_explain(&gpus[i], why, sizeof why);
            fprintf(stderr, "note: GPU %u Hot Spot unavailable: %s\n", gpus[i].index, why);
        }
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
            therm_dump(&gpus[i]);

            gddr_explain(&gpus[i], why, sizeof why);
            printf("GPU %u: FBPA DQR scan, %d memory module(s)\n",
                   gpus[i].index, gpus[i].nMods);
            printf("  Memory modules: %s\n", why);
            gddr_dump(&gpus[i]);
        }
        return 0;
    }

    signal(SIGINT, on_sigint);

    /* Single-metric query mode: one bare value per line, one line per GPU. */
    if (g_query) {
        for (int i = 0; i < nGpu; i++) {
            Sample s;
            gpu_sample(&gpus[i], &s);
            if (!query_print(&s)) {
                if (watch)
                    continue;
                printf("n/a\n");
            }
        }
        return 0;
    }

    do {
        if (watch)
            printf("\033[H\033[2J");
        printf("%s  %-4s  %*s  %*s  %*s  %*s  %*s%s\n", C_BOLD(), "GPU",
               W_TEMP, "Core Temp", W_TEMP, "Hot Spot", W_TEMP, "Mem Temp",
               W_VOLT, "NVVDD", W_VOLT, "MSVDD", C_RESET());
        printf("%s  %-4s  %*s  %*s  %*s  %*s  %*s%s\n", C_DIM(), "----",
               W_TEMP, "----------", W_TEMP, "----------", W_TEMP, "----------",
               W_VOLT, "-------------", W_VOLT, "-------------",
               C_RESET());
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
