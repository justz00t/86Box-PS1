/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the VLSI VL82C330 "TOPCAT" system controller
 *          (386DX/486 set: VL82C330 + VL82C331 + VL82C332), as used on the
 *          IBM PS/1 2133/2155 486SX planar with 30-pin SIMMs (01-10156-01).
 *
 *          Written against the VLSI "TOPCAT" data manual (October 1990),
 *          VL82C330 section, and the IBM BIOS dated 08/27/92:
 *            - configuration index/data at ECh/EDh, enable/disable through
 *              dummy writes to FBh/F9h (enabled after reset; the IBM BIOS
 *              never writes FBh);
 *            - RAMMAP (Table 1 memory map codes), RAMMOV (Table 2 logical
 *              to physical bank switch) and the row/column address
 *              multiplexing of Tables 4-6 (non-interleaved columns). When
 *              RAMMAP selects a scheme for bigger DRAMs than are fitted,
 *              the SIMMs only see the MA lines they have and the bank
 *              aliases; the BIOS sizes memory by looking for that;
 *            - maps 1Eh/1Fh, which move the 384K above 640K to the top
 *              of memory;
 *            - AAXS..FAXS shadow RAM control (regs 0Dh-12h);
 *            - fast A20 (EEh), fast CPU reset (EFh), slow/fast (F4h/F5h).
 *
 *          Latched but not acted on: slot pointer, EMS (0Bh/0Ch, E8h-EBh),
 *          DRAM timing, refresh, sleep, bus clock and CTRL1. Interleaved
 *          multiplexing is not modelled (it only matters in mismatched
 *          maps with two equal banks, which the BIOS never programs).
 *
 * Authors: (contributed)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/machine.h>
#include <86box/mem.h>
#include <86box/port_92.h>
#include <86box/chipset.h>
#include <86box/plat_unused.h>

typedef struct vl82c330_t {
    uint8_t idx;
    uint8_t cfg_enabled;
    uint8_t regs[256];
    uint8_t ems_idx;
    uint8_t ems_set;

    /* Installed depth of each physical bank (four 30-pin SIMMs) as MB per bank: 0, 1, 4 or 16. */
    uint8_t  bank_mb[4];
    /* Offset of each physical bank's storage inside the host RAM block. */
    uint32_t bank_base[4];

    /* Current logical layout. */
    uint8_t  type_mb[4]; /* DRAM type RAMMAP assumes for logical bank 0-3, as MB per bank */
    uint8_t  phys[4];    /* physical bank behind logical bank 0-3 (RAMMOV) */
    uint32_t total_kb;
    int      aliased;
    int      remapped;

    mem_mapping_t low_mapping;
    mem_mapping_t high_mapping;
} vl82c330_t;

/* Table 1: DRAM type per logical bank for each memory map code (1 = 256K, 4 = 1M, 16 = 4M devices). */
static const uint8_t vl82c330_maps[32][4] = {
    {  1,  0,  0,  0 }, /* 00 */
    {  1,  1,  0,  0 }, /* 01 */
    {  1,  1,  1,  0 }, /* 02 */
    {  1,  1,  1,  1 }, /* 03 */
    {  4,  0,  0,  0 }, /* 04 */
    {  1,  4,  0,  0 }, /* 05 */
    {  1,  1,  4,  0 }, /* 06 */
    {  4,  4,  0,  0 }, /* 07 */
    {  4,  4,  1,  0 }, /* 08 */
    {  1,  1,  4,  4 }, /* 09 */
    {  4,  4,  4,  0 }, /* 0A */
    {  4,  4,  4,  4 }, /* 0B */
    { 16,  0,  0,  0 }, /* 0C */
    {  1, 16,  0,  0 }, /* 0D */
    {  1,  1, 16,  0 }, /* 0E */
    {  4, 16,  0,  0 }, /* 0F */
    {  4,  4, 16,  0 }, /* 10 */
    { 16, 16,  0,  0 }, /* 11 */
    { 16, 16,  1,  0 }, /* 12 */
    {  1,  1, 16, 16 }, /* 13 */
    { 16, 16,  4,  0 }, /* 14 */
    {  4,  4, 16, 16 }, /* 15 */
    { 16, 16, 16,  0 }, /* 16 */
    { 16, 16, 16, 16 }, /* 17 */
    {  1,  0,  0,  0 }, /* 18-1D: undefined, treated as power-up map */
    {  1,  0,  0,  0 },
    {  1,  0,  0,  0 },
    {  1,  0,  0,  0 },
    {  1,  0,  0,  0 },
    {  1,  0,  0,  0 },
    {  1,  1,  0,  0 }, /* 1E: as 01, 384K above 640K moved to the top */
    {  1,  0,  0,  0 }  /* 1F: as 00, 384K above 640K moved to the top */
};

/* Table 2: physical bank behind each logical bank for each RAMMOV code. */
static const uint8_t vl82c330_ramsw[16][4] = {
    { 0, 1, 2, 3 }, { 2, 0, 1, 3 }, { 0, 2, 1, 3 }, { 2, 1, 0, 3 },
    { 1, 2, 0, 3 }, { 1, 0, 3, 2 }, { 1, 2, 3, 0 }, { 2, 1, 3, 0 },
    { 0, 3, 1, 2 }, { 0, 3, 2, 1 }, { 2, 3, 1, 0 }, { 3, 0, 1, 2 },
    { 3, 0, 2, 1 }, { 3, 1, 2, 0 }, { 3, 2, 0, 1 }, { 3, 2, 1, 0 }
};

/*
 * Tables 4-6, no interleave: CPU address bit driven onto MA0..MA10 during the
 * column (CAS) and row (RAS) phase, for 256K, 1M and 4M DRAM maps.
 */
static const uint8_t vl82c330_col[3][11] = {
    { 4, 5, 6, 7, 8, 9, 10, 2, 3, 0xff, 0xff },
    { 4, 5, 6, 7, 8, 9, 10, 2, 3,   11, 0xff },
    { 4, 5, 6, 7, 8, 9, 10, 2, 3,   11,   12 }
};
static const uint8_t vl82c330_row[3][11] = {
    { 19, 18, 17, 16, 15, 14, 13, 12, 11, 0xff, 0xff },
    { 19, 18, 17, 16, 15, 14, 13, 12, 20,   21, 0xff },
    { 19, 18, 17, 16, 15, 14, 13, 21, 20,   22,   23 }
};

static int
vl82c330_shflags(uint8_t access)
{
    int ret;

    /* 00: read default, write slot; 01: read default, write board;
       10: read board, write slot;   11: read and write board. */
    switch (access) {
        default:
        case 0x00:
            ret = MEM_READ_EXTANY | MEM_WRITE_EXTANY;
            break;
        case 0x01:
            ret = MEM_READ_EXTANY | MEM_WRITE_INTERNAL;
            break;
        case 0x02:
            ret = MEM_READ_INTERNAL | MEM_WRITE_EXTANY;
            break;
        case 0x03:
            ret = MEM_READ_INTERNAL | MEM_WRITE_INTERNAL;
            break;
    }

    return ret;
}

static void
vl82c330_recalc_shadow(vl82c330_t *dev)
{
    uint32_t base;
    uint8_t  access;

    shadowbios       = 0;
    shadowbios_write = 0;

    /* AAXS (0Dh) .. FAXS (12h): one register per 64K segment, 2 bits per 16K page. */
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 8; j += 2) {
            base   = 0x000a0000 + (i << 16) + (j << 13);
            access = (dev->regs[0x0d + i] >> j) & 3;
            mem_set_mem_state(base, 0x4000, vl82c330_shflags(access));
            shadowbios |= ((base >= 0xe0000) && (access & 0x02));
            shadowbios_write |= ((base >= 0xe0000) && (access & 0x01));
        }
    }

    flushmmucache();
}

static int
vl82c330_type_index(uint8_t mb)
{
    return (mb >= 16) ? 2 : ((mb >= 4) ? 1 : 0);
}

/* Translate a CPU address to an offset into the host RAM block, or 0xffffffff if nothing answers. */
static uint32_t
vl82c330_translate(const vl82c330_t *dev, uint32_t addr)
{
    uint32_t size;
    uint32_t off;
    int      lb;
    int      t;
    int      inst;
    int      lines;
    uint8_t  pb;

    for (lb = 0; lb < 4; lb++) {
        size = ((uint32_t) dev->type_mb[lb]) << 20;
        if (size == 0)
            return 0xffffffff;
        if (addr < size)
            break;
        addr -= size;
    }
    if (lb == 4)
        return 0xffffffff;

    pb = dev->phys[lb];
    if (dev->bank_mb[pb] == 0)
        return 0xffffffff;

    t     = vl82c330_type_index(dev->type_mb[lb]);
    inst  = vl82c330_type_index(dev->bank_mb[pb]);
    lines = 9 + ((inst < t) ? inst : t); /* MA lines both the map and the SIMMs use */

    /* The cell selected is the one the SIMM sees on its own MA lines; express it as the
       address that reaches the same cell when the map matches the SIMMs, so the contents
       survive the BIOS switching from the sizing map to the final one. */
    off = addr & 3;
    for (int ma = 0; ma < lines; ma++) {
        off |= ((addr >> vl82c330_col[t][ma]) & 1) << vl82c330_col[inst][ma];
        off |= ((addr >> vl82c330_row[t][ma]) & 1) << vl82c330_row[inst][ma];
    }

    return dev->bank_base[pb] + off;
}

static uint8_t
vl82c330_ram_readb(uint32_t addr, void *priv)
{
    const vl82c330_t *dev = (vl82c330_t *) priv;
    uint32_t          off = vl82c330_translate(dev, addr);

    return (off == 0xffffffff) ? 0xff : ram[off];
}

static uint16_t
vl82c330_ram_readw(uint32_t addr, void *priv)
{
    return vl82c330_ram_readb(addr, priv) | (vl82c330_ram_readb(addr + 1, priv) << 8);
}

static uint32_t
vl82c330_ram_readl(uint32_t addr, void *priv)
{
    return vl82c330_ram_readw(addr, priv) | (((uint32_t) vl82c330_ram_readw(addr + 2, priv)) << 16);
}

static void
vl82c330_ram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    const vl82c330_t *dev = (vl82c330_t *) priv;
    uint32_t          off = vl82c330_translate(dev, addr);

    if (off != 0xffffffff)
        ram[off] = val;
}

static void
vl82c330_ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    vl82c330_ram_writeb(addr, val & 0xff, priv);
    vl82c330_ram_writeb(addr + 1, val >> 8, priv);
}

static void
vl82c330_ram_writel(uint32_t addr, uint32_t val, void *priv)
{
    vl82c330_ram_writew(addr, val & 0xffff, priv);
    vl82c330_ram_writew(addr + 2, val >> 16, priv);
}

static void
vl82c330_recalc_banks(vl82c330_t *dev)
{
    uint8_t  code    = dev->regs[0x03] & 0x1f;
    uint8_t  sw      = dev->regs[0x04] & 0x0f;
    int      matched = 1;
    int      remap   = (code >= 0x1e);
    uint32_t total   = 0; /* MB */

    for (uint8_t i = 0; i < 4; i++) {
        dev->type_mb[i] = vl82c330_maps[code][i];
        dev->phys[i]    = vl82c330_ramsw[sw][i];
        total += dev->type_mb[i];
        if (dev->type_mb[i] && (dev->bank_mb[dev->phys[i]] != dev->type_mb[i]))
            matched = 0;
    }
    dev->total_kb = total << 10;

    /* Undo a previous 384K relocation before touching the RAM mappings. */
    if (dev->remapped) {
        mem_remap_top_ex(0, dev->remapped);
        dev->remapped = 0;
    }

    if (matched) {
        /* The map matches the SIMMs: plain linear RAM, use the fast path. */
        dev->aliased = 0;

        mem_mapping_disable(&dev->low_mapping);
        mem_mapping_disable(&dev->high_mapping);

        mem_mapping_set_addr(&ram_low_mapping, 0x00000000, 0x000a0000);
        if (dev->total_kb > 1024)
            mem_mapping_set_addr(&ram_high_mapping, 0x00100000, (dev->total_kb - 1024) << 10);
        else
            mem_mapping_disable(&ram_high_mapping);

        if (remap) {
            mem_remap_top_ex(384, dev->total_kb);
            dev->remapped = dev->total_kb;
        }
    } else {
        /* Mismatch (this is what the BIOS provokes while sizing): go through the translator. */
        dev->aliased = 1;

        mem_mapping_disable(&ram_low_mapping);
        mem_mapping_disable(&ram_high_mapping);

        mem_mapping_set_addr(&dev->low_mapping, 0x00000000, 0x000a0000);
        if (dev->total_kb > 1024)
            mem_mapping_set_addr(&dev->high_mapping, 0x00100000, (dev->total_kb - 1024) << 10);
        else
            mem_mapping_disable(&dev->high_mapping);
    }

    flushmmucache();
}

static void
vl82c330_write(uint16_t addr, uint8_t val, void *priv)
{
    vl82c330_t *dev          = (vl82c330_t *) priv;
    int         special_func = !(dev->regs[0x14] & 0x80);

    switch (addr) {
        case 0xe8: /* EMS index */
            if (dev->cfg_enabled)
                dev->ems_idx = val;
            break;
        case 0xe9: /* EMS active set */
            if (dev->cfg_enabled)
                dev->ems_set = val;
            break;
        case 0xea: /* EMS page data: latched nowhere, EMS is not emulated */
        case 0xeb:
            break;

        case 0xec:
            if (dev->cfg_enabled)
                dev->idx = val;
            break;

        case 0xed:
            if (!dev->cfg_enabled)
                break;
            switch (dev->idx) {
                case 0x00: /* VER, read only */
                    break;
                case 0x01: /* SLOTHI: only A25-A24 exist, the rest reads as ones */
                    dev->regs[0x01] = val | 0xfc;
                    break;
                case 0x03: /* RAMMAP: bits 6-5 read as ones */
                    dev->regs[0x03] = val | 0x60;
                    vl82c330_recalc_banks(dev);
                    break;
                case 0x04: /* RAMMOV */
                    dev->regs[0x04] = val | 0xf0;
                    vl82c330_recalc_banks(dev);
                    break;
                case 0x0d ... 0x12: /* AAXS..FAXS */
                    dev->regs[dev->idx] = val;
                    vl82c330_recalc_shadow(dev);
                    break;
                case 0x15: /* TEST: factory use, never to be written */
                    break;
                default:
                    /* Everything else is latched. The IBM BIOS also writes index 81h early in
                       POST, which the data manual does not define. */
                    dev->regs[dev->idx] = val;
                    break;
            }
            break;

        case 0xee: /* Fast A20: any write disables it */
            if (special_func) {
                mem_a20_alt = 0x00;
                mem_a20_recalc();
            }
            break;

        case 0xf4: /* slow CPU clock */
        case 0xf5: /* full speed */
            break;

        case 0xf9: /* configuration disable */
            if (special_func)
                dev->cfg_enabled = 0;
            break;
        case 0xfb: /* configuration enable */
            if (special_func)
                dev->cfg_enabled = 1;
            break;

        default:
            break;
    }
}

static uint8_t
vl82c330_read(uint16_t addr, void *priv)
{
    vl82c330_t *dev          = (vl82c330_t *) priv;
    int         special_func = !(dev->regs[0x14] & 0x80);
    uint8_t     ret          = 0xff;

    switch (addr) {
        case 0xe8:
            if (dev->cfg_enabled)
                ret = dev->ems_idx;
            break;
        case 0xe9:
            if (dev->cfg_enabled)
                ret = dev->ems_set;
            break;

        case 0xec:
            if (dev->cfg_enabled)
                ret = dev->idx;
            break;
        case 0xed:
            if (dev->cfg_enabled)
                ret = dev->regs[dev->idx];
            break;

        case 0xee: /* Fast A20: a read enables it and returns FFh */
            if (special_func) {
                mem_a20_alt = 0x02;
                mem_a20_recalc();
            }
            break;
        case 0xef: /* Fast CPU reset */
            if (special_func)
                softresetx86();
            break;

        default:
            break;
    }

    return ret;
}

static void
vl82c330_reset(void *priv)
{
    vl82c330_t *dev = (vl82c330_t *) priv;

    memset(dev->regs, 0x00, sizeof(dev->regs));

    dev->regs[0x00] = 0xf2; /* VER: "330", revision 2 */
    dev->regs[0x01] = 0xff; /* SLOTHI */
    dev->regs[0x02] = 0xff; /* SLOTLO: slot pointer 03FFh, no slot memory */
    dev->regs[0x03] = 0xe0; /* RAMMAP: ROM at E0000h, one bank of 256K devices */
    dev->regs[0x04] = 0xf0; /* RAMMOV: logical bank n is physical bank n */
    dev->regs[0x05] = 0x7c; /* RAMSET */
    dev->regs[0x07] = 0xff; /* RASTMA: timing for 100 ns DRAM at 33 MHz */
    dev->regs[0x08] = 0xff; /* CASTMA */
    dev->regs[0x09] = 0xff; /* RASTMB */
    dev->regs[0x0a] = 0xff; /* CASTMB */
    dev->regs[0x13] = 0x01; /* SLEEP */
    dev->regs[0x14] = 0x06; /* MISCSET: BUSCLK = TCLK2 / 4 */

    dev->idx         = 0x00;
    dev->cfg_enabled = 1;
    dev->ems_idx     = 0x00;
    dev->ems_set     = 0x00;

    mem_a20_alt = 0x00;
    mem_a20_recalc();

    vl82c330_recalc_banks(dev);
    vl82c330_recalc_shadow(dev);
}

static void
vl82c330_close(void *priv)
{
    vl82c330_t *dev = (vl82c330_t *) priv;

    free(dev);
}

static void *
vl82c330_init(UNUSED(const device_t *info))
{
    vl82c330_t *dev = (vl82c330_t *) calloc(1, sizeof(vl82c330_t));
    /* Totals that two banks of four identical 256K/1M/4M SIMMs can make, largest first.
       Only the first two bank positions exist on the IBM planar (eight sockets). */
    static const uint8_t combos[9][3] = {
        { 32, 16, 16 },
        { 20, 16,  4 },
        { 17, 16,  1 },
        { 16, 16,  0 },
        {  8,  4,  4 },
        {  5,  4,  1 },
        {  4,  4,  0 },
        {  2,  1,  1 },
        {  1,  1,  0 }
    };
    uint32_t mb = mem_size >> 10;

    dev->bank_mb[0] = 1;
    dev->bank_mb[1] = 0;
    for (uint8_t i = 0; i < 9; i++) {
        if (mb >= combos[i][0]) {
            dev->bank_mb[0] = combos[i][1];
            dev->bank_mb[1] = combos[i][2];
            break;
        }
    }

    /* Keep each bank's cells where the final map will find them: Table 1 always lists the
       smaller bank first, so the BIOS swaps the banks with RAMMOV when bank 0 is the larger. */
    if (dev->bank_mb[1] && (dev->bank_mb[0] > dev->bank_mb[1])) {
        dev->bank_base[1] = 0x00000000;
        dev->bank_base[0] = ((uint32_t) dev->bank_mb[1]) << 20;
    } else {
        dev->bank_base[0] = 0x00000000;
        dev->bank_base[1] = ((uint32_t) dev->bank_mb[0]) << 20;
    }

    mem_mapping_add(&dev->low_mapping, 0x00000000, 0x000a0000,
                    vl82c330_ram_readb, vl82c330_ram_readw, vl82c330_ram_readl,
                    vl82c330_ram_writeb, vl82c330_ram_writew, vl82c330_ram_writel,
                    NULL, MEM_MAPPING_INTERNAL, dev);
    mem_mapping_disable(&dev->low_mapping);
    mem_mapping_add(&dev->high_mapping, 0x00100000, 0x00100000,
                    vl82c330_ram_readb, vl82c330_ram_readw, vl82c330_ram_readl,
                    vl82c330_ram_writeb, vl82c330_ram_writew, vl82c330_ram_writel,
                    NULL, MEM_MAPPING_INTERNAL, dev);
    mem_mapping_disable(&dev->high_mapping);

    /* E8h-EFh: EMS and configuration; F4h/F5h, F9h, FBh: special functions. F0h-F3h belong
       to the coprocessor interface and are left to the CPU emulation. */
    io_sethandler(0x00e8, 0x0008, vl82c330_read, NULL, NULL, vl82c330_write, NULL, NULL, dev);
    io_sethandler(0x00f4, 0x0002, vl82c330_read, NULL, NULL, vl82c330_write, NULL, NULL, dev);
    io_sethandler(0x00f9, 0x0001, vl82c330_read, NULL, NULL, vl82c330_write, NULL, NULL, dev);
    io_sethandler(0x00fb, 0x0001, vl82c330_read, NULL, NULL, vl82c330_write, NULL, NULL, dev);

    device_add(&port_92_device);

    vl82c330_reset(dev);

    return dev;
}

const device_t vl82c330_device = {
    .name          = "VLSI VL82c330 (TOPCAT)",
    .internal_name = "vl82c330",
    .flags         = 0,
    .local         = 0,
    .init          = vl82c330_init,
    .close         = vl82c330_close,
    .reset         = vl82c330_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
