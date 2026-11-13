/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the IBM PS/1 models 2011, 2121.
 *
 * Model 2011: The initial model, using a 10MHz 80286.
 *
 * Model 2121: This is similar to model 2011 but some of the functionality
 *             has moved to a chip at ports 0xe0 (index)/0xe1 (data). The
 *             only functions I have identified are enables for the first
 *             512K and next 128K of RAM, in bits 0 of registers 0 and 1
 *             respectively.
 *
 *             Port 0x105 bit 7 selects the ROM for reads of E0000-FFFFF
 *             and bit 6 sends writes to the shadow RAM, which the ROM's
 *             own power-on code uses to copy itself into the RAM behind
 *             it. The BIOS would otherwise miss that 128K on cold boots.
 *
 *             The reserved 384K is remapped to the top of extended memory.
 *             If this is not done then you get an error on startup.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *          Copyright 2008-2019 Sarah Walker.
 *          Copyright 2016-2019 Miran Grca.
 *          Copyright 2017-2019 Fred N. van Kempen.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/io.h>
#include <86box/dma.h>
#include <86box/pic.h>
#include <86box/pit.h>
#include <86box/mem.h>
#include <86box/nmi.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/chipset.h>
#include <86box/sio.h>
#include <86box/nvr.h>
#include <86box/gameport.h>
#include <86box/lpt.h>
#include <86box/serial.h>
#include <86box/keyboard.h>
#include <86box/hdc.h>
#include <86box/hdc_ide.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/port_6x.h>
#include <86box/video.h>
#include <86box/machine.h>
#include <86box/sound.h>
#include <86box/plat_unused.h>

typedef struct {
    int model;

    rom_t mid_rom, high_rom;

    uint8_t ps1_91,
        ps1_92,
        ps1_94,
        ps1_100,
        ps1_101,
        ps1_102,
        ps1_103,
        ps1_104,
        ps1_105;
    int     ps1_e0_addr;
    uint8_t ps1_e0_regs[256];

    serial_t *uart;
    lpt_t    *lpt;
} ps1_t;

static uint8_t
ps1_read(uint16_t port, void *priv)
{
    ps1_t  *ps  = (ps1_t *) priv;
    uint8_t ret = 0xff;

    switch (port) {
        case 0x0091:
            ret        = ps->ps1_91;
            ps->ps1_91 = 0;
            break;

        case 0x0092:
            ret = ps->ps1_92;
            break;

        case 0x0094:
            ret = ps->ps1_94;
            break;

        case 0x00e1:
            if (ps->model != 2011) {
                ret = ps->ps1_e0_regs[ps->ps1_e0_addr];
            }
            break;

        case 0x0100:
            ret = ps->ps1_100;
            break;

        case 0x0101:
            ret = ps->ps1_101;
            break;

        case 0x0102:
            if (ps->model == 2011)
                ret = ps->ps1_102 | 0x08;
            else
                ret = ps->ps1_102;
            break;

        case 0x0103:
            ret = ps->ps1_103;
            break;

        case 0x0104:
            ret = ps->ps1_104;
            break;

        case 0x0105:
            ret = ps->ps1_105;
            break;

        default:
            break;
    }

    return ret;
}

static void
ps1_m2121_recalc_memory(ps1_t *ps)
{
    /* Enable first 512K */
    mem_set_mem_state(0x00000, 0x80000,
                      (ps->ps1_e0_regs[0] & 0x01) ? (MEM_READ_INTERNAL | MEM_WRITE_INTERNAL) : (MEM_READ_EXTANY | MEM_WRITE_EXTANY));

    /* Enable 512-640K */
    mem_set_mem_state(0x80000, 0x20000,
                      (ps->ps1_e0_regs[1] & 0x01) ? (MEM_READ_INTERNAL | MEM_WRITE_INTERNAL) : (MEM_READ_EXTANY | MEM_WRITE_EXTANY));
}

/* 105h bit 7 selects the ROM for reads of E0000-FFFFF, bit 6 sends its
   writes to the shadow RAM. */
static void
ps1_m2121_shadow_update(ps1_t *ps)
{
    uint32_t read, write;

    read  = (ps->ps1_105 & 0x80) ? MEM_READ_EXTANY : MEM_READ_INTERNAL;
    write = (ps->ps1_105 & 0x40) ? MEM_WRITE_INTERNAL : MEM_WRITE_EXTANY;

    mem_set_mem_state_both(0x000e0000, 0x00020000, read | write);
    flushmmucache();
}

static void
ps1_write(uint16_t port, uint8_t val, void *priv)
{
    ps1_t *ps = (ps1_t *) priv;

    switch (port) {
        case 0x0092:
            if (ps->model != 2011) {
                if (val & 1) {
                    softresetx86();
                    cpu_set_edx();
                }
                ps->ps1_92 = val & ~1;
            } else {
                ps->ps1_92 = val;
            }
            mem_a20_alt = val & 2;
            mem_a20_recalc();
            break;

        case 0x0094:
            ps->ps1_94 = val;
            break;

        case 0x00e0:
            if (ps->model != 2011) {
                ps->ps1_e0_addr = val;
            }
            break;

        case 0x00e1:
            if (ps->model != 2011) {
                ps->ps1_e0_regs[ps->ps1_e0_addr] = val;
                ps1_m2121_recalc_memory(ps);
            }
            break;

        case 0x0102:
            if (!(ps->ps1_94 & 0x80)) {
                lpt_port_remove(ps->lpt);
                serial_remove(ps->uart);
                if (val & 0x04) {
                    if (val & 0x08)
                        serial_setup(ps->uart, COM1_ADDR, COM1_IRQ);
                    else
                        serial_setup(ps->uart, COM2_ADDR, COM2_IRQ);
                }
                if (val & 0x10) {
                    switch ((val >> 5) & 3) {
                        case 0:
                            lpt_port_setup(ps->lpt, LPT_MDA_ADDR);
                            break;
                        case 1:
                            lpt_port_setup(ps->lpt, LPT1_ADDR);
                            break;
                        case 2:
                            lpt_port_setup(ps->lpt, LPT2_ADDR);
                            break;

                        default:
                            break;
                    }
                }
                ps->ps1_102 = val;
            }
            break;

        case 0x0103:
            ps->ps1_103 = val;
            break;

        case 0x0104:
            ps->ps1_104 = val;
            break;

        case 0x0105:
            ps->ps1_105 = val;
            if (ps->model != 2011)
                ps1_m2121_shadow_update(ps);
            break;

        default:
            break;
    }
}

static void
ps1_setup(int model)
{
    ps1_t *ps;
    void  *priv;

    ps        = (ps1_t *) calloc(1, sizeof(ps1_t));
    ps->model = model;

    io_sethandler(0x0091, 1,
                  ps1_read, NULL, NULL, ps1_write, NULL, NULL, ps);
    io_sethandler(0x0092, 1,
                  ps1_read, NULL, NULL, ps1_write, NULL, NULL, ps);
    io_sethandler(0x0094, 1,
                  ps1_read, NULL, NULL, ps1_write, NULL, NULL, ps);
    io_sethandler(0x0100, 6,
                  ps1_read, NULL, NULL, ps1_write, NULL, NULL, ps);

    ps->uart = device_add_inst(&ns16450_device, 1);
    ps->lpt  = device_add_inst(&lpt_port_device, 1);
    lpt_port_remove(ps->lpt);
    lpt_port_setup(ps->lpt, LPT_MDA_ADDR);

    mem_remap_top(384);

    device_add(&fdc_ps2_device);

    if (model == 2011) {
        const device_t *d      = device_context_get_device();
        const char     *bios   = device_get_config_bios("bios_language");
        const char     *first  = device_get_bios_file(d, bios, 0);
        const char     *second = device_get_bios_file(d, bios, 1);

        /* No planar ID on the 2011; both POS registers read 0FFh. */
        ps->ps1_100 = 0xff;
        ps->ps1_101 = 0xff;

        if (!strcmp(bios, "english_us")) {
            /* US English */
            rom_init(&ps->high_rom, first,
                     0xfc0000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);
        } else if (second == NULL) {
            /* Combined ROM. */
            rom_init(&ps->high_rom, first,
                     0xf80000, 0x80000, 0x7ffff, 0, MEM_MAPPING_EXTERNAL);
        } else {
            /* Split ROM. */
            rom_init(&ps->mid_rom, first,
                     0xf80000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);

            rom_init(&ps->high_rom, second,
                     0xfc0000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);
        }

        lpt_set_next_inst(255);

        if (sound_card_current[0] == SOUND_INTERNAL)
            machine_snd = device_add(machine_get_snd_device(machine));

        /* Enable the builtin HDC. */
        if (hdc_current[0] == HDC_INTERNAL) {
            priv = device_add(&ps1_hdc_device);

            ps1_hdc_inform(priv, &ps->ps1_91);
        }

        /* Enable the PS/1 VGA controller. */
        device_add(&ps1vga_device);
    } else if (model == 2121) {
        const device_t *d      = device_context_get_device();
        const char     *bios   = device_get_config_bios("bios_language");
        const char     *first  = device_get_bios_file(d, bios, 0);
        const char     *second = device_get_bios_file(d, bios, 1);

        /* Read-only planar ID: the ROM compares it against 0FEh/0E8h. */
        ps->ps1_100 = 0xfe;
        ps->ps1_101 = 0xe8;

        /* Start with the ROM visible and the shadow RAM writable, which
           is the state the ROM's own power-on sequence expect to find. */
        ps->ps1_105 = 0xc0;
        ps1_m2121_shadow_update(ps);

        io_sethandler(0x00e0, 2,
                      ps1_read, NULL, NULL,
                      ps1_write, NULL, NULL,
                      ps);

        if (second == NULL) {
            rom_init(&ps->high_rom, first,
                     0xfc0000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);
        } else {
            rom_init(&ps->mid_rom, first,
                     0xf80000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);

            rom_init(&ps->high_rom, second,
                     0xfc0000, 0x40000, 0x3ffff, 0, MEM_MAPPING_EXTERNAL);
        }

        /* Initialize the video controller. */
        if (gfxcard[0] == VID_INTERNAL)
            device_add(&ibm_ps1_2121_device);

        device_add(&ide_isa_device);

        if (sound_card_current[0] == SOUND_INTERNAL)
            machine_snd = device_add(machine_get_snd_device(machine));
    }
}

uint8_t
machine_ps1_p1_handler(void)
{
    const uint8_t current_drive = fdc_get_current_drive();

    /* (B0 or F0) | (fdd_is_525(current_drive) on bit 6) */
    return 0xb0 | (fdd_is_525(&drives[current_drive]) ? 0x40 : 0x00);
}

static void
ps1_common_init(const machine_t *model)
{
    machine_common_init(model);

    refresh_at_enable = 1;
    pit_devs[0].set_out_func(pit_devs[0].data, 1, pit_refresh_timer_at);

    dma16_init();
    pic2_init();

    device_add_params(machine_get_kbc_device(machine), (void *) model->kbc_params);
    device_add(&port_6x_device);

    /* Audio uses ports 200h and 202-207h, so only initialize gameport on 201h. */
    standalone_gameport_type = &gameport_201_device;
}

static const device_config_t ps1_2011_config[] = {
    // clang-format off
    {
        .name           = "bios_language",
        .description    = "BIOS Language",
        .type           = CONFIG_BIOS,
        .default_string = "english_us",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios = {
            {
                .name          = "English (US)",
                .internal_name = "english_us",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/FC0000_US.BIN",
                                   "" }
            },
            {
                .name          = "English (UK)",
                .internal_name = "english_uk",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_UK.BIN",
                                   "roms/machines/ibmps1es/FC0000_UK.BIN",
                                   "" }
            },
            {
                .name          = "English (Canada)",
                .internal_name = "english_ca",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_CA.BIN",
                                   "roms/machines/ibmps1es/FC0000_CA.BIN",
                                   "" }
            },
            {
                .name          = "Portuguese",
                .internal_name = "portuguese",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_PT.BIN",
                                   "roms/machines/ibmps1es/FC0000_PT.BIN",
                                   "" }
            },
            {
                .name          = "Swedish",
                .internal_name = "swedish",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_SE.BIN",
                                   "roms/machines/ibmps1es/FC0000_SE.BIN",
                                   "" }
            },
            {
                .name          = "Dutch",
                .internal_name = "dutch",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_NL.BIN",
                                   "roms/machines/ibmps1es/FC0000_NL.BIN",
                                   "" }
            },
            {
                .name          = "French",
                .internal_name = "french",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_FR.BIN",
                                   "roms/machines/ibmps1es/FC0000_FR.BIN",
                                   "" }
            },
            {
                .name          = "German",
                .internal_name = "german",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_DE.BIN",
                                   "roms/machines/ibmps1es/FC0000_DE.BIN",
                                   "" }
            },
            {
                .name          = "Italian",
                .internal_name = "italian",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1es/F80000_IT.BIN",
                                   "roms/machines/ibmps1es/FC0000_IT.BIN",
                                   "" }
            },
            {
                .name          = "Spanish",
                .internal_name = "spanish",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 524288,
                .files         = { "roms/machines/ibmps1es/F80000_ES.BIN",
                                   "" }
            },
            { .files_no = 0 }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ps1_2011_device = {
    .name          = "IBM PS/1 model 2011",
    .internal_name = "ibmps1es",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ps1_2011_config
};

int
machine_ps1_m2011_init(const machine_t *model)
{
    int         ret;
    const char *fn;
    uint32_t    offset;

    if (!device_available(model->device)) {
        /* No ROMs available. */
        return 0;
    }

    device_context(model->device);
    if ((fn = device_get_bios_file(model->device, device_get_config_bios("bios_language"), 1)) == NULL) {
        /* Combined ROM or US English. */
        fn     = device_get_bios_file(model->device, device_get_config_bios("bios_language"), 0);
        offset = (!strcmp("english_us", device_get_config_bios("bios_language"))) ? 0x20000 : 0x60000;
    } else {
        /* Separated ROM. */
        offset = 0x20000;
    }

    if (!fn) {
        fn     = device_get_bios_file(model->device, "us_english", 0);
        offset = 0x20000;
    }

    ret = bios_load_linear(fn, 0x000e0000, 131072, offset);
    device_context_restore();

    if (bios_only || !ret) {
        return ret;
    }

    ps1_common_init(model);

    device_context(model->device);

    ps1_setup(2011);

    device_context_restore();

    return ret;
}

static const device_config_t ps1_2121_config[] = {
    // clang-format off
    {
        .name           = "bios_language",
        .description    = "BIOS Language",
        .type           = CONFIG_BIOS,
        .default_string = "no_rom_drive",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios = {
            {
                .name          = "No ROM drive",
                .internal_name = "no_rom_drive",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/FC0000.BIN",
                                   "" }
            },
            {
                .name          = "English (US)",
                .internal_name = "english_us",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/FC0000_US.BIN",
                                   "" }
            },
            {
                .name          = "English (UK)",
                .internal_name = "english_uk",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_UK.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_UK.BIN",
                                   "" }
            },
            {
                .name          = "Portuguese",
                .internal_name = "portuguese",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_PT.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_PT.BIN",
                                   "" }
            },
            {
                .name          = "French",
                .internal_name = "french",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_FR.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_FR.BIN",
                                   "" }
            },
            {
                .name          = "German",
                .internal_name = "german",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_DE.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_DE.BIN",
                                   "" }
            },
            {
                .name          = "Italian",
                .internal_name = "italian",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_IT.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_IT.BIN",
                                   "" }
            },
            {
                .name          = "Spanish",
                .internal_name = "spanish",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 2,
                .local         = 0,
                .size          = 262144,
                .files         = { "roms/machines/ibmps1_2121/F80000_ES.BIN",
                                   "roms/machines/ibmps1_2121/FC0000_ES.BIN",
                                   "" }
            },
            { .files_no = 0 }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ps1_2121_device = {
    .name          = "IBM PS/1 model 2121",
    .internal_name = "ibmps1_2121",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ps1_2121_config
};

int
machine_ps1_m2121_init(const machine_t *model)
{
    int         ret;
    const char *fn;

    if (!device_available(model->device)) {
        /* No ROMs available. */
        return 0;
    }

    device_context(model->device);
    if ((fn = device_get_bios_file(model->device, device_get_config_bios("bios_language"), 1)) == NULL) {
        fn = device_get_bios_file(model->device, device_get_config_bios("bios_language"), 0);
    }

    ret = bios_load_linear(fn, 0x000e0000, 131072, 0x20000);
    device_context_restore();

    if (bios_only || !ret) {
        return ret;
    }

    ps1_common_init(model);

    device_context(model->device);

    ps1_setup(2121);

    device_context_restore();

    return ret;
}

static const device_config_t ps1_2133_config[] = {
    // clang-format off
    {
        .name           = "bios",
        .description    = "BIOS Version",
        .type           = CONFIG_BIOS,
        .default_string = "ps1_2133_65g9499",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = {
            {
                .name          = "IBM PS/1 BIOS - Revision 65G9499 (09/01/93)",
                .internal_name = "ps1_2133_65g9499",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 131072,
                .files         = { "roms/machines/ibmps1_2133/65g9499.bin", "" }
            },
            {
                .name          = "IBM PS/1 BIOS - Revision 52G2973 (01/22/93)",
                .internal_name = "ps1_2133_52g2973",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 131072,
                .files         = { "roms/machines/ibmps1_2133/ps1_2133_52g2974_rom.bin", "" }
            },
            { .files_no = 0 }
        }
    },
    {
        .name           = "cache_size",
        .description    = "External cache",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = VL82C480_CACHE_128K,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "128 KB", .value = VL82C480_CACHE_128K },
            { .description = "256 KB", .value = VL82C480_CACHE_256K },
            { .description = ""                                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ps1_2133_device = {
    .name          = "IBM PS/1 model 2133",
    .internal_name = "ibmps1_2133",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ps1_2133_config
};

int
machine_ps1_m2133_init(const machine_t *model)
{
    int         ret = 0;
    const char *fn;
    int         cache_size;

    /* No ROMs available. */
    if (!device_available(model->device))
        return ret;

    device_context(model->device);
    fn         = device_get_bios_file(machine_get_device(machine), device_get_config_bios("bios"), 0);
    ret        = bios_load_linear(fn, 0x000e0000, 131072, 0);
    cache_size = device_get_config_int("cache_size");
    device_context_restore();

    if (bios_only || !ret)
        return ret;

    machine_at_common_init(model);

    device_add_params(&vl82c480_device, (void *) (uintptr_t) cache_size);
    /* The KBC and the RTC are on the VLSI VL82C114 Combination I/O. */
    device_add(&vl82c113_device);

    device_add(&ide_isa_device);
    /* The IBM drive takes DENSEL active low, so IDENT must be strapped low. */
    device_add_params(&pc873xx_device, (void *) (PCX73XX_IDE_PRI | PCX73XX_FDC_ON | PCX73XX_DENSEL_LOW | PCX730X_398));

    if (gfxcard[0] == VID_INTERNAL)
        device_add(machine_get_vid_device(machine));

    return ret;
}

/*
 * IBM PS/1 2133/2155, 486SX planar with 30-pin SIMMs (board 01-10156-01).
 *
 * VLSI TOPCAT (VL82C330/331/332), VL82C106 combination I/O, Tseng ET4000AX
 * on board with its video BIOS in the lower half of the 128K system ROM
 * (E000:0000), BIOS dated 08/27/92 on an even/odd EPROM pair (33G6876 /
 * 33G6875) - use the two halves interleaved into one 128K image.
 *
 * Per the IBM service manual (63G2028): soldered 486SX at 20 or 25 MHz with
 * an upgrade socket, two banks of four 30-pin SIMMs (256K, 1M or 4M, 32 MB
 * maximum), 512K of video RAM expandable to 1 MB, no external cache.
 * POST checkpoints go to port 190h (and to the parallel port at 3BCh).
 * Ports 50h/54h belong to a cache controller this planar does not have.
 */
/*
 * VLSI VL82C106 Combination I/O as wired on this planar (VTI data book, 1989, p. 6-89):
 * two 16C450 UARTs, the parallel port, the 8042-compatible keyboard/mouse controller,
 * the RTC and the floppy/IDE chip selects.
 *
 * Its control registers are indexes 69h-7Fh. The BIOS reaches them through 170h (index)
 * and 171h (data), and Control Register 0 is also readable and writable at port 102h:
 *   69h  Control 0: b0 system board enable, b1 FDC CS, b2 COMA, b3 COMA default (1 = 3F8h),
 *                   b4 LPT, b6-5 LPT default (00 = 3BCh, 01 = 378h, 10 = 278h), b7 extended LPT
 *   6Ah  Control 1: b0 COMB, b1 AT (1) / PS/2 (0) keyboard, b3 CS mode (1 = programmed
 *                   decodes below), b4 HDC CS, b5 IDE, b6 CS6, b7 CS7
 *   6Bh-7Fh  base low, base high, range for CS1 COMA, CS2 COMB, CS3 LPT, CS4 FDC,
 *            CS5 HDC, CS6 (3F2h) and CS7 (3F7h)
 * The keyboard controller and RTC are provided by the generic AT devices.
 */
typedef struct ps1_vl82c106_t {
    uint8_t   idx;
    uint8_t   regs[256];
    serial_t *uart[2];
    lpt_t    *lpt;
} ps1_vl82c106_t;

static void
ps1_vl82c106_recalc(ps1_vl82c106_t *dev)
{
    const uint8_t ctrl0   = dev->regs[0x69];
    const uint8_t ctrl1   = dev->regs[0x6a];
    const int     sys_en  = !!(ctrl0 & 0x01);
    const int     cs_mode = !!(ctrl1 & 0x08);
    uint16_t      addr;

    /* COMA (CS1). */
    serial_remove(dev->uart[0]);
    if (sys_en && (ctrl0 & 0x04)) {
        if (cs_mode)
            addr = (dev->regs[0x6c] << 8) | dev->regs[0x6b];
        else
            addr = (ctrl0 & 0x08) ? COM1_ADDR : COM2_ADDR;
        if (addr != 0x0000)
            serial_setup(dev->uart[0], addr, (addr & 0x0100) ? COM1_IRQ : COM2_IRQ);
    }

    /* COMB (CS2). */
    serial_remove(dev->uart[1]);
    if (ctrl1 & 0x01) {
        if (cs_mode)
            addr = (dev->regs[0x6f] << 8) | dev->regs[0x6e];
        else
            addr = (ctrl0 & 0x08) ? COM2_ADDR : COM1_ADDR;
        if (addr != 0x0000)
            serial_setup(dev->uart[1], addr, (addr & 0x0100) ? COM1_IRQ : COM2_IRQ);
    }

    /* Parallel port (CS3). */
    lpt_port_remove(dev->lpt);
    if (sys_en && (ctrl0 & 0x10)) {
        if (cs_mode)
            addr = (dev->regs[0x72] << 8) | dev->regs[0x71];
        else switch ((ctrl0 >> 5) & 0x03) {
            case 0x00:
                addr = LPT_MDA_ADDR;
                break;
            case 0x01:
                addr = LPT1_ADDR;
                break;
            case 0x02:
                addr = LPT2_ADDR;
                break;
            default:
                addr = 0x0000;
                break;
        }
        if (addr != 0x0000) {
            lpt_port_setup(dev->lpt, addr);
            lpt_port_irq(dev->lpt, LPT1_IRQ);
        }
    }
    lpt_set_ext(dev->lpt, !!(ctrl0 & 0x80));
}

static void
ps1_vl82c106_write(uint16_t port, uint8_t val, void *priv)
{
    ps1_vl82c106_t *dev = (ps1_vl82c106_t *) priv;

    switch (port) {
        case 0x0102: /* Control Register 0, PS/2-style alias */
            dev->regs[0x69] = val;
            ps1_vl82c106_recalc(dev);
            break;
        case 0x0170:
            dev->idx = val;
            break;
        case 0x0171:
            if ((dev->idx >= 0x69) && (dev->idx <= 0x7f)) {
                dev->regs[dev->idx] = val;
                if (dev->idx <= 0x72)
                    ps1_vl82c106_recalc(dev);
            }
            break;
        default:
            break;
    }
}

static uint8_t
ps1_vl82c106_read(uint16_t port, void *priv)
{
    const ps1_vl82c106_t *dev = (ps1_vl82c106_t *) priv;
    uint8_t               ret = 0xff;

    switch (port) {
        case 0x0102:
            ret = dev->regs[0x69];
            break;
        case 0x0170:
            ret = dev->idx;
            break;
        case 0x0171:
            if ((dev->idx >= 0x69) && (dev->idx <= 0x7f))
                ret = dev->regs[dev->idx];
            break;
        default:
            break;
    }

    return ret;
}

static void
ps1_vl82c106_reset(void *priv)
{
    ps1_vl82c106_t *dev = (ps1_vl82c106_t *) priv;

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->idx        = 0x00;
    dev->regs[0x69] = 0x37; /* board, FDC, COMA and LPT enabled; LPT default 378h */
    dev->regs[0x6a] = 0xf3; /* COMB, AT keyboard mode, HDC, IDE, CS6 and CS7; hard-wired decodes */

    ps1_vl82c106_recalc(dev);
}

static void
ps1_vl82c106_close(void *priv)
{
    free(priv);
}

static void *
ps1_vl82c106_init(UNUSED(const device_t *info))
{
    ps1_vl82c106_t *dev = (ps1_vl82c106_t *) calloc(1, sizeof(ps1_vl82c106_t));

    dev->uart[0] = device_add_inst(&ns16450_device, 1);
    dev->uart[1] = device_add_inst(&ns16450_device, 2);
    dev->lpt     = device_add_inst(&lpt_port_device, 1);

    io_sethandler(0x0102, 0x0001, ps1_vl82c106_read, NULL, NULL, ps1_vl82c106_write, NULL, NULL, dev);
    io_sethandler(0x0170, 0x0002, ps1_vl82c106_read, NULL, NULL, ps1_vl82c106_write, NULL, NULL, dev);

    ps1_vl82c106_reset(dev);

    return dev;
}

static const device_t ps1_vl82c106_device = {
    .name          = "VLSI VL82C106 Combination I/O (IBM PS/1 486SX 30-pin planar)",
    .internal_name = "ps1_vl82c106",
    .flags         = 0,
    .local         = 0,
    .init          = ps1_vl82c106_init,
    .close         = ps1_vl82c106_close,
    .reset         = ps1_vl82c106_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

static const device_config_t ps1_2133_486sx30_config[] = {
    // clang-format off
    {
        .name           = "bios",
        .description    = "BIOS Version",
        .type           = CONFIG_BIOS,
        .default_string = "ps1_2133_33g6875",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = {
            {
                .name          = "IBM PS/1 BIOS - 33G6876/33G6875 (08/27/92)",
                .internal_name = "ps1_2133_33g6875",
                .bios_type     = BIOS_NORMAL,
                .files_no      = 1,
                .local         = 0,
                .size          = 131072,
                .files         = { "roms/machines/ibmps1_2133_486sx30/33g6875_merged.bin", "" }
            },
            { .files_no = 0 }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ps1_2133_486sx30_device = {
    .name          = "IBM PS/1 model 2133 (486SX, 30-pin SIMM planar)",
    .internal_name = "ibmps1_2133_486sx30",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ps1_2133_486sx30_config
};

/*
 * 8042 input port. The BIOS insists on bits 0-5 reading as ones; bit 6 is a
 * planar strap that selects which DRAM timing set POST loads into the chipset.
 */
uint8_t
machine_ps1_m2133_486sx30_p1_handler(void)
{
    return 0xff;
}

int
machine_ps1_m2133_486sx30_init(const machine_t *model)
{
    int         ret = 0;
    const char *fn;

    /* No ROMs available. */
    if (!device_available(model->device))
        return ret;

    device_context(model->device);
    fn  = device_get_bios_file(machine_get_device(machine), device_get_config_bios("bios"), 0);
    ret = bios_load_linear(fn, 0x000e0000, 131072, 0);
    device_context_restore();

    if (bios_only || !ret)
        return ret;

    machine_at_init(model);

    device_add(&vl82c330_device);
    device_add(&ps1_vl82c106_device);

    device_add(&ide_isa_device);
    device_add(&fdc_at_device);

    if (gfxcard[0] == VID_INTERNAL)
        device_add(machine_get_vid_device(machine));

    return ret;
}
