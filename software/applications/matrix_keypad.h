/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-21     greedyhao       the first version
 */
#ifndef _MATRIX_KEYPAD_H_
#define _MATRIX_KEYPAD_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>

//According to your need to modify the constants.
#define KEYPAD_TICKS_INTERVAL    5 //ms
#define KEYPAD_DEBOUNCE_TICKS    3 //MAX 8

#define KEYPAD_FIFO_SIZE        10
#define KEYPAD_FIFO_EMPTY       0x00
#define KEYPAD_FIFO_FULL        0xff

struct keypad_fifo {
    uint8_t buffer[KEYPAD_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
};

struct keypad {
    uint16_t ticks;
    uint16_t pad_level;
    uint8_t  repeat;
    uint8_t  state;
    uint8_t  debounce_cnt;
    uint8_t  col_dir;
    uint8_t *col;
    uint8_t  col_size;
    uint8_t *row;
    uint8_t  row_size;
    uint8_t  x;
    uint8_t  y;
};

void keypad_init(struct keypad *handle, uint8_t *col, uint8_t col_size, uint8_t *row, uint8_t row_size);
void keypad_ticks(void);
uint8_t keypad_get_value(void);

#endif /* _MATRIX_KEYPAD_H_ */
