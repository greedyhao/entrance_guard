#include "entry_key.h"
#include "as608.h"  // as608 packages
#include <math.h>

#define DBG_TAG              "key.fp"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#define AS60X_UART_NAME "uart3"

static struct entry_key en_key_fp = {0};
static uint8_t fp_id[KEY_ID_MAX] = {0};

static void fp_add_key(uint16_t id)
{
    if (id > KEY_ID_MAX)
    {
        LOG_E("id out of range!");
        return;
    }
    fp_id[id] = 1;
    as60x_str_fp_to_flash(id);
}

static void fp_del_key(uint16_t id)
{
    as60x_delet_fp_n_id(id, 1);
    fp_id[id] = 0;
}

static uint16_t fp_ver_key(void)
{
    uint16_t id = 0;
    uint16_t score = 0;
    as60x_search_fp_in_flash(&id, &score);
    return ((score < 0x10 || id >= KEY_VER_ERROR) ? KEY_VER_ERROR : id);
}

static uint8_t fp_has_key(uint16_t id)
{
    return (fp_id[id] != 0) ? 1 : 0;
}

static int rt_hw_fp_port(void)
{
    as60x_set_hand_shake_baud(115200);
    as60x_init(AS60X_UART_NAME);

    en_key_fp.add_key = fp_add_key;
    en_key_fp.del_key = fp_del_key;
    en_key_fp.ver_key = fp_ver_key;
    en_key_fp.has_key = fp_has_key;
    reg_key_obj(ENTRY_KEY_FP, &en_key_fp);

    fp_wak_evt_reg(get_key_det_evt(), EVT_KEY_DET_FP);
    return 0;
}
INIT_ENV_EXPORT(rt_hw_fp_port);
