#include "entry_key.h"
#include "as608.h"
#include <math.h>

#define AS60X_UART_NAME "uart3"

static struct entry_key en_key_fp = {0};
// static uint32_t 
static uint8_t fp_id[KEY_ID_MAX] = {0};
// static uint32_t fp_wr_flag = 0; 
// static uint8_t best_wr_idx = 0;

static void fp_add_key(uint8_t id)
{
    if (id > KEY_ID_MAX)
    {
        rt_kprintf("id out of range!\n");
        return;
    }
    fp_id[id] = 1;
    as60x_str_fp_to_flash(id);
}

static void fp_del_key(uint8_t id)
{
    as60x_delet_fp_n_id(id, 1);
}

static uint8_t fp_ver_key(void)
{

}

static void rt_hw_fp_port(void)
{
    as60x_set_hand_shake_baud(115200);
    as60x_init(AS60X_UART_NAME);

    en_key_fp.add_key = fp_add_key;
    en_key_fp.del_key = fp_del_key;
    en_key_fp.ver_key = fp_ver_key;
    reg_key_obj(ENTRY_KEY_FP, &en_key_fp);

    en_key_fp.enable = 1;
}
INIT_COMPONENT_EXPORT(rt_hw_fp_port)
