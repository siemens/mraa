/*
 * Author: Le Jin <le.jin@siemens.com>
 * Copyright (c) Siemens AG, 2019
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "arm/siemens/platform_iot2050.h"

enum{
    GROUP_MAIN_DOMAIN = 0,
    GROUP_WAKUP_DOMAIN,
    MAX_GROUP_NUMBER
};

typedef struct{
    PinMuxInterface_t       super;
    int                     memFd;
    volatile uint32_t       *pinMuxRegBase[MAX_GROUP_NUMBER];
    volatile void           *mapAddress[MAX_GROUP_NUMBER];
}IOT2050PinMuxHandler_t;

static bool iot2050_pinmux_init(void);
static void iot2050_pinmux_deinit(void);
static void iot2050_pinmux_select_func(uint8_t group, uint16_t pinIndex, uint8_t func);
static void iot2050_pinmux_select_input(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_output(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_inout(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_hiz(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_pull_up(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_pull_down(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_select_pull_disable(uint8_t group, uint16_t pinIndex);
static uint32_t iot2050_pinmux_get_raw_reg_value(uint8_t group, uint16_t pinIndex);
static void iot2050_pinmux_set_raw_reg_value(uint8_t group, uint16_t pinIndex, uint32_t value);
static void iot2050_pinmux_dump_info(uint8_t group, uint16_t pinIndex);

#define MAIN_PINMUX_REG_NUM                 (195)
#define MAIN_PINMUX_REG_LENGTH              (MAIN_PINMUX_REG_NUM<<2)
#define MAIN_PINMUX_REG_PHY_BASE_ADDRESS    (0x0011c000)

#define WAKUP_PINMUX_REG_NUM                (70)
#define WAKUP_PINMUX_REG_LENGTH             (WAKUP_PINMUX_REG_NUM<<2)
#define WAKUP_PINMUX_REG_PHY_BASE_ADDRESS   (0x4301c000)

#define PAGE_SIZE 4096UL
#define PAGE_MASK (PAGE_SIZE - 1)

#define REG_MUXMODE_POS             (0)
#define REG_MUXMODE_MASK            (0x0F << REG_MUXMODE_POS)
#define REG_MUXMODE_GET(reg)        ((reg & REG_MUXMODE_MASK) >> REG_MUXMODE_POS)
#define REG_MUXMODE(mode)           ((mode << REG_MUXMODE_POS) & REG_MUXMODE_MASK)

#define REG_PULL_ENABLE_POS         (16)
#define REG_PULL_ENABLE_MASK        (0x01 << REG_PULL_ENABLE_POS)
#define REG_PULL_ENABLE_GET(reg)    ((reg & REG_PULL_ENABLE_MASK) >> REG_PULL_ENABLE_POS)
#define REG_PULL_ENABLE             (0x00 << REG_PULL_ENABLE_POS)
#define REG_PULL_DISABLE            (0x01 << REG_PULL_ENABLE_POS)
#define REG_PULL_IS_ENABLED(reg)    (!REG_PULL_ENABLE_GET(reg))
#define REG_PULL_IS_DISABLED(reg)   (!REG_PULL_IS_ENABLED(reg))

#define REG_PULL_SELECT_POS         (17)
#define REG_PULL_SELECT_MASK        (0x01 << REG_PULL_SELECT_POS)
#define REG_PULL_SELECT_GET(reg)    ((reg & REG_PULL_SELECT_MASK) >> REG_PULL_SELECT_POS)
#define REG_PULL_UP                 (0x01 << REG_PULL_SELECT_POS)
#define REG_PULL_DOWN               (0x00 << REG_PULL_SELECT_POS)
#define REG_PULL_IS_UP(reg)         (REG_PULL_SELECT_GET(reg))
#define REG_PULL_IS_DOWN(reg)       (!REG_PULL_IS_UP(reg))

#define REG_INPUT_ENABLE_POS        (18)
#define REG_INPUT_ENABLE_MASK       (0x01 << REG_INPUT_ENABLE_POS)
#define REG_INPUT_ENABLE_GET(reg)   ((reg & REG_INPUT_ENABLE_MASK) >> REG_INPUT_ENABLE_POS)
#define REG_INPUT_ENABLE            (0x01 << REG_INPUT_ENABLE_POS)
#define REG_INPUT_DISABLE           (0x00 << REG_INPUT_ENABLE_POS)
#define REG_INPUT_IS_ENABLED(reg)   (REG_INPUT_ENABLE_GET(reg))
#define REG_INPUT_IS_DISABLED(reg)  (!REG_INPUT_IS_ENABLED(reg))

#define REG_OUTPUT_ENABLE_POS       (21)
#define REG_OUTPUT_ENABLE_MASK      (0x01 << REG_OUTPUT_ENABLE_POS)
#define REG_OUTPUT_ENABLE_GET(reg)  ((reg & REG_OUTPUT_ENABLE_MASK) >> REG_OUTPUT_ENABLE_POS)
#define REG_OUTPUT_ENABLE           (0x00 << REG_OUTPUT_ENABLE_POS)
#define REG_OUTPUT_DISABLE          (0x01 << REG_OUTPUT_ENABLE_POS)
#define REG_OUTPUT_IS_ENABLED(reg)  (!REG_OUTPUT_ENABLE_GET(reg))
#define REG_OUTPUT_IS_DISABLED(reg) (!REG_OUTPUT_IS_ENABLED(reg))

#define REG_UPDATE(address, mask, value)        (*address = ((*address) & (~mask)) | (value))
#define GROUP_IS_VALID(group)                   (group < MAX_GROUP_NUMBER)

#define PIN_OUTPUT		(PULL_DISABLE)
#define PIN_OUTPUT_PULLUP	(PULL_UP)
#define PIN_OUTPUT_PULLDOWN	0
#define PIN_INPUT		(INPUT_EN | PULL_DISABLE)
#define PIN_INPUT_PULLUP	(INPUT_EN | PULL_UP)
#define PIN_INPUT_PULLDOWN	(INPUT_EN)

static IOT2050PinMuxHandler_t iot2050PinMuxHandler = {
    .super = {
        .initialized = false,
        .ops = {
            .init = iot2050_pinmux_init,
            .deinit = iot2050_pinmux_deinit,
            .select_func = iot2050_pinmux_select_func,
            .select_input = iot2050_pinmux_select_input,
            .select_output = iot2050_pinmux_select_output,
            .select_inout = iot2050_pinmux_select_inout,
            .select_hiz = iot2050_pinmux_select_hiz,
            .select_pull_up = iot2050_pinmux_select_pull_up,
            .select_pull_down = iot2050_pinmux_select_pull_down,
            .select_pull_disable = iot2050_pinmux_select_pull_disable,
            .get_raw_reg_value = iot2050_pinmux_get_raw_reg_value,
            .set_raw_reg_value = iot2050_pinmux_set_raw_reg_value,
            .dump_info = iot2050_pinmux_dump_info
        }
    },
    .memFd = -1,
    .mapAddress[GROUP_MAIN_DOMAIN] = MAP_FAILED,
    .mapAddress[GROUP_WAKUP_DOMAIN] = MAP_FAILED,
    .pinMuxRegBase[GROUP_MAIN_DOMAIN] = MAP_FAILED,
    .pinMuxRegBase[GROUP_WAKUP_DOMAIN] = MAP_FAILED
};

PinMuxInterface_t*
iot2050_pinmux_get_instance(void) 
{
    return (PinMuxInterface_t *)&iot2050PinMuxHandler;
}

static bool
iot2050_pinmux_init(void)
{
    uint32_t pageMask = sysconf(_SC_PAGE_SIZE) - 1;

    /* Open memory */
    DEBUG_PRINT("Open device\n");
    if((iot2050PinMuxHandler.memFd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        DEBUG_PRINT("Open device: %s\n", strerror(errno));
        goto _FATAL;
    }

    /* Map */
    DEBUG_PRINT("Map main mux register base\n");
    iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN] = mmap(0, MAIN_PINMUX_REG_LENGTH,
                                                    PROT_READ | PROT_WRITE, MAP_SHARED,
                                                    iot2050PinMuxHandler.memFd,
                                                    MAIN_PINMUX_REG_PHY_BASE_ADDRESS & ~pageMask);
    if(iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN] == MAP_FAILED) {
        DEBUG_PRINT("Pinmux main domain map failed: %s\n", strerror(errno));
        goto _FATAL;
    }
    iot2050PinMuxHandler.pinMuxRegBase[GROUP_MAIN_DOMAIN] = iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN] + 
                                        (MAIN_PINMUX_REG_PHY_BASE_ADDRESS & pageMask);
    DEBUG_PRINT("\tPage address: %p\n", iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN]);
    DEBUG_PRINT("\tIn-page Offset: %x\n", MAIN_PINMUX_REG_PHY_BASE_ADDRESS & pageMask);
    DEBUG_PRINT("\tReg address: %p\n", iot2050PinMuxHandler.pinMuxRegBase[GROUP_MAIN_DOMAIN]);
    DEBUG_PRINT("Map wakup mux register base\n");
    iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN] = mmap(0, WAKUP_PINMUX_REG_LENGTH,
                                                    PROT_READ | PROT_WRITE, MAP_SHARED,
                                                    iot2050PinMuxHandler.memFd,
                                                    WAKUP_PINMUX_REG_PHY_BASE_ADDRESS & ~pageMask);
    if(iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN] == MAP_FAILED) {
        DEBUG_PRINT("Pinmux wakup domain map failed: %s\n", strerror(errno));
        goto _FATAL;
    }
    iot2050PinMuxHandler.pinMuxRegBase[GROUP_WAKUP_DOMAIN] = iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN] + 
                                        (WAKUP_PINMUX_REG_PHY_BASE_ADDRESS & pageMask);
    DEBUG_PRINT("\tPage address: %p\n", iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN]);
    DEBUG_PRINT("\tIn-page Offset: %x\n", WAKUP_PINMUX_REG_PHY_BASE_ADDRESS & pageMask);
    DEBUG_PRINT("\tReg address: %p\n", iot2050PinMuxHandler.pinMuxRegBase[GROUP_WAKUP_DOMAIN]);
    iot2050PinMuxHandler.super.initialized = true;
    return true;
_FATAL:
    iot2050_pinmux_deinit();
    return false;
}

static void
iot2050_pinmux_deinit(void)
{
    if(iot2050PinMuxHandler.memFd > 0)
        close(iot2050PinMuxHandler.memFd);
    iot2050PinMuxHandler.memFd = -1;
    if(iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN] != MAP_FAILED)
        munmap((void *)iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN], MAIN_PINMUX_REG_LENGTH);
    iot2050PinMuxHandler.mapAddress[GROUP_MAIN_DOMAIN] = MAP_FAILED;
    iot2050PinMuxHandler.pinMuxRegBase[GROUP_MAIN_DOMAIN] = MAP_FAILED;

    if(iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN] != MAP_FAILED)
        munmap((void *)iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN], WAKUP_PINMUX_REG_LENGTH);
    iot2050PinMuxHandler.mapAddress[GROUP_WAKUP_DOMAIN] = MAP_FAILED;
    iot2050PinMuxHandler.pinMuxRegBase[GROUP_WAKUP_DOMAIN] = MAP_FAILED;
    iot2050PinMuxHandler.super.initialized = false;
}

static void
iot2050_pinmux_select_func(uint8_t group, uint16_t pinIndex, uint8_t func) {
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_MUXMODE_MASK, REG_MUXMODE(func));
    }
}

static void
iot2050_pinmux_select_input(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_INPUT_ENABLE_MASK, REG_INPUT_ENABLE);
        REG_UPDATE(regVirtualAddress, REG_OUTPUT_ENABLE_MASK, REG_OUTPUT_DISABLE);
    }
}

static void
iot2050_pinmux_select_output(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_INPUT_ENABLE_MASK, REG_INPUT_DISABLE);
        REG_UPDATE(regVirtualAddress, REG_OUTPUT_ENABLE_MASK, REG_OUTPUT_ENABLE);
    }
}

static void
iot2050_pinmux_select_inout(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_INPUT_ENABLE_MASK, REG_INPUT_ENABLE);
        REG_UPDATE(regVirtualAddress, REG_OUTPUT_ENABLE_MASK, REG_OUTPUT_ENABLE);
    }
}

static void
iot2050_pinmux_select_hiz(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_INPUT_ENABLE_MASK, REG_INPUT_DISABLE);
        REG_UPDATE(regVirtualAddress, REG_OUTPUT_ENABLE_MASK, REG_OUTPUT_DISABLE);
    }
}


static void
iot2050_pinmux_select_pull_up(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_PULL_ENABLE_MASK, REG_PULL_ENABLE);
        REG_UPDATE(regVirtualAddress, REG_PULL_SELECT_MASK, REG_PULL_UP);
    }
}

static void
iot2050_pinmux_select_pull_down(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_PULL_ENABLE_MASK, REG_PULL_ENABLE);
        REG_UPDATE(regVirtualAddress, REG_PULL_SELECT_MASK, REG_PULL_DOWN);
    }
}

static void
iot2050_pinmux_select_pull_disable(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        REG_UPDATE(regVirtualAddress, REG_PULL_ENABLE_MASK, REG_PULL_DISABLE);
    }
}

static uint32_t
iot2050_pinmux_get_raw_reg_value(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        return *regVirtualAddress;
    }
    return -1;
}

static void
iot2050_pinmux_set_raw_reg_value(uint8_t group, uint16_t pinIndex, uint32_t value)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        *regVirtualAddress = value;
    }
}

static void
iot2050_pinmux_dump_info(uint8_t group, uint16_t pinIndex)
{
    volatile uint32_t *regVirtualAddress;

    if(GROUP_IS_VALID(group)) {
        regVirtualAddress = iot2050PinMuxHandler.pinMuxRegBase[group] + pinIndex;
        fprintf(stderr, "PinmuxReg Domain %s, Index %d, Raw 0x%08x\n",
                        group?"Wakup":"Main",
                        pinIndex,
                        iot2050_pinmux_get_raw_reg_value(group, pinIndex));
        if(REG_INPUT_IS_ENABLED(*regVirtualAddress)) {
            fprintf(stderr, "\tInput: enabled\n");
        }
        else if(REG_OUTPUT_IS_ENABLED(*regVirtualAddress)) {
            fprintf(stderr, "\tOutput: enabled\n");
        }
        else {
            fprintf(stderr, "\tOutput: Hiz\n");
        }

        if(REG_PULL_IS_ENABLED(*regVirtualAddress)) {
            if(REG_PULL_IS_UP(*regVirtualAddress)) {
                fprintf(stderr, "\tPull Status: up\n");
            }
            else {
                fprintf(stderr, "\tPull Status: down\n");
            }
        }
        else {
            fprintf(stderr, "\tPull Status: disabled\n");
        }
        fprintf(stderr, "\tMode: %d\n", REG_MUXMODE_GET(*regVirtualAddress));
    }
}
