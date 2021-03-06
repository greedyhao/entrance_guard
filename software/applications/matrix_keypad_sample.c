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
#include "entry_key.h"

static struct keypad _pad;
static uint8_t pad_col[] = {GET_PIN(B, 5), GET_PIN(B, 4), GET_PIN(B, 3)};
static uint8_t pad_row[] = {GET_PIN(B, 9), GET_PIN(B, 8), GET_PIN(B, 7), GET_PIN(B, 6)};

static void pad_event_send(uint8_t value)
{
    if (value == KEYPAD_NUM_POUND)
        rt_event_send(get_key_det_evt(), EVT_GRD_DET_PW);
}

static void pad_event_recv(int32_t timeout)
{
    rt_event_recv(get_key_det_evt(), EVT_GRD_DET_PW, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, timeout, NULL);
}

static void keypad_thread_entry(void* p)
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

    _pad.event_send = pad_event_send;
    _pad.event_recv = pad_event_recv;
    keypad_init(&_pad, pad_col, sizeof(pad_col)/sizeof(uint8_t), pad_row, sizeof(pad_row)/sizeof(uint8_t));

    /* Create background ticks thread */
    thread = rt_thread_create("pad", keypad_thread_entry, RT_NULL, 512, 10, 10);
    if(thread == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(thread);

    return RT_EOK;
}
INIT_APP_EXPORT(matrix_keypad_test);
