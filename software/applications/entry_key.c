#include "entry_key.h"
#include <rtthread.h>
#include <rtdevice.h>

static entry_key_t key_table[KEY_TYPES_MAX] = {0};

/**
 * @brief Get the key object
 * 
 * @param key_type 
 * @return entry_key_t
 */
entry_key_t get_key_obj(enum entry_key_type key_type)
{
    return key_table[key_type];
}

/**
 * @brief Regist the key object
 * 
 * @param key_type 
 * @param key_obj 
 */
void reg_key_obj(enum entry_key_type key_type, entry_key_t key_obj)
{
    RT_ASSERT(key_obj != RT_NULL);
    if (key_table[key_type] != 0)
    {
        rt_kprintf("Warning! Type of %d has been registered!", (uint8_t)key_type);
    }

    key_table[key_type] = key_obj; 
}

uint32_t check_auth(void)
{
    
}
