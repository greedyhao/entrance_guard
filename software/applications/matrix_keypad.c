#include "matrix_keypad.h"

#define DBG_TAG              "kepad.src"
 #define DBG_LVL              DBG_INFO
//#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

static struct keypad_fifo _fifo = {0};
static struct keypad *_handle;
static rt_sem_t keypad_sem = NULL;

/**
 *
 * @return return KEYPAD_FIFO_EMPTY if buffer empty
 */
static uint8_t keypad_fifo_read(void)
{
    if (0 == _fifo.count) return KEYPAD_FIFO_EMPTY;
    _fifo.count--;
    _fifo.tail = (_fifo.tail + 1) % KEYPAD_FIFO_SIZE;
    return _fifo.buffer[_fifo.tail];
}

/**
 *
 * @param value write to buffer
 * @return return KEYPAD_FIFO_FULL if buffer full
 */
static uint8_t keypad_fifo_write(uint8_t value)
{
    if (KEYPAD_FIFO_SIZE == _fifo.count) return KEYPAD_FIFO_FULL;
    _fifo.count++;
    _fifo.head = (_fifo.head + 1) % KEYPAD_FIFO_SIZE;
    _fifo.buffer[_fifo.head] = value;
    return _fifo.buffer[_fifo.head];
}

/**
 *
 * @param handle
 * @param col_dir   0:col in 1:col out
 */
static void keypad_pin_cfg(struct keypad *handle)
{
    uint8_t i;
    if (handle->col_dir == 1) {
        for (i=0 ; i<handle->col_size; i++)
        {
            rt_pin_mode(handle->col[i], PIN_MODE_OUTPUT);
            rt_pin_write(handle->col[i], PIN_HIGH);
        }
        for (i=0 ; i<handle->row_size; i++)
        {
            rt_pin_mode(handle->row[i], PIN_MODE_INPUT_PULLDOWN);
        }
    }else {
        for (i=0 ; i<handle->row_size; i++)
        {
            rt_pin_mode(handle->row[i], PIN_MODE_OUTPUT);
            rt_pin_write(handle->row[i], PIN_HIGH);
        }
        for (i=0 ; i<handle->col_size; i++)
        {
            rt_pin_mode(handle->col[i], PIN_MODE_INPUT_PULLDOWN);
        }
    }
}

/**
 *
 * @param handle
 * @param col_dir   0:col in 1:col out
 */
static uint16_t keypad_pin_read(struct keypad *handle)
{
    uint16_t ret = 0;
    uint8_t i;
    if (handle->col_dir == 1) {
        for (i=0 ; i<handle->row_size; i++)
        {
            if (rt_pin_read(handle->row[i]) != 0)
                ret = i+1;
        }
    }else {
        for (i=0 ; i<handle->col_size; i++)
        {
            if (rt_pin_read(handle->col[i]) != 0)
                ret = i+1;
        }
    }
    return ret;
}

static void keypad_handler(struct keypad *handle)
{
    uint16_t read_gpio_level = keypad_pin_read(handle);

    //ticks counter working..
    if((handle->state) > 0)
    {
        handle->ticks++;
    }

    /*------------button debounce handle---------------*/
    if(read_gpio_level != handle->pad_level)
    {
        //not equal to prev one
        //continue read 3 times same new level change
        if(++(handle->debounce_cnt) >= KEYPAD_DEBOUNCE_TICKS)
        {
            handle->pad_level = read_gpio_level;
            handle->debounce_cnt = 0;
        }
    }
    else
    {
        // leved not change ,counter reset.
        handle->debounce_cnt = 0;
    }

    /*-----------------State machine-------------------*/
    switch (handle->state) {
        case 0:
            if(handle->pad_level != 0) // col in not equal 0
            {
                handle->ticks  = 0;
                handle->state  = 1;
                handle->col_dir = 1;
                handle->x = handle->pad_level;
                keypad_pin_cfg(handle);

                handle->y = keypad_pin_read(handle);
                handle->col_dir = 0;
                keypad_pin_cfg(handle);
            }
            break;
        case 1:
            if(handle->pad_level != 0) // row in not equal 0; press down
            {
                handle->ticks  = 0;
                handle->state  = 2;
                handle->col_dir = 0;
            }
            break;
        case 2:
            if(handle->pad_level == 0) // col equal 0; press up
            {
                uint8_t key_val = handle->x + (handle->y-1)*handle->col_size;
                uint8_t tmp = 0;
                handle->ticks = 0;
                handle->state = 0;
                tmp = keypad_fifo_write(key_val);
                handle->callback(tmp);
                LOG_D("key_val:%d fifo:%d count:%d", key_val, tmp, _fifo.count);
            }
            break;
        default:
            break;
    }
}

void keypad_init(struct keypad *handle, uint8_t *col, uint8_t col_size, uint8_t *row, uint8_t row_size)
{
    _fifo.head = 0;
    _fifo.tail = 0;
    _fifo.count = 0;
    memset(_fifo.buffer, 0, sizeof(_fifo.buffer));

    handle->col = col;
    handle->col_size = col_size;
    handle->row = row;
    handle->row_size = row_size;
    handle->col_dir = 0;
    handle->x = 0;
    handle->y = 0;

    _handle = handle;
    keypad_pin_cfg(handle);
}

void keypad_ticks(void)
{
    keypad_handler(_handle);
}

uint8_t keypad_get_value(void)
{
    return keypad_fifo_read();
}

uint8_t keypad_get_len(void)
{
    return _fifo.count;
}

void keypad_get_n_value(uint8_t *buffer, uint8_t len)
{
    uint8_t i;
    for (i=0; i<len; i++)
    {
        buffer[i] = keypad_fifo_read();
    }
}

#if 0
void keypad_pin_read_test(int argc, char**argv)
{
    if (!strcmp(argv[1], "1")) {
        rt_kprintf("col_dir:1\n");
        _handle->col_dir = 1;
    }else if (!strcmp(argv[1], "0")) {
        rt_kprintf("col_dir:0\n");
        _handle->col_dir = 0;
    }
    keypad_pin_cfg(_handle);
    uint8_t tmp = keypad_pin_read(_handle);
    rt_kprintf("keypad_pin_read: %d\n", tmp);
}
MSH_CMD_EXPORT(keypad_pin_read_test, "keypad pin read");

void keypad_get_value_test(void *p)
{
    uint8_t tmp = keypad_get_value();
    rt_kprintf("keypad_get_value: %d\n", tmp);
}
MSH_CMD_EXPORT(keypad_get_value_test, "keypad get value");
#endif

