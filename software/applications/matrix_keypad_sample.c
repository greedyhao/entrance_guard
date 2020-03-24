/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-21     greedyhao       the first version
 */

#include "matrix_keypad.h"
#include "drv_gpio.h"

static struct keypad _pad;
static uint8_t pad_col[] = {GET_PIN(B, 5), GET_PIN(B, 4), GET_PIN(B, 3)};
static uint8_t pad_row[] = {GET_PIN(B, 9), GET_PIN(B, 8), GET_PIN(B, 7), GET_PIN(B, 6)};


void keypad_thread_entry(void* p)
{
    while(1)
    {
        /* 5ms */
        rt_thread_delay(RT_TICK_PER_SECOND/200);
        keypad_ticks();
    }
}

int matrix_keypad_test(void)
{
    rt_thread_t thread = RT_NULL;

    keypad_init(&_pad, pad_col, sizeof(pad_col)/sizeof(uint8_t), pad_row, sizeof(pad_row)/sizeof(uint8_t));

    /* Create background ticks thread */
    thread = rt_thread_create("pad", keypad_thread_entry, RT_NULL, 1024, 10, 10);
    if(thread == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(thread);

    return RT_EOK;
}
INIT_APP_EXPORT(matrix_keypad_test);
