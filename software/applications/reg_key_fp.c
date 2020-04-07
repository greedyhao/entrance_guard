#include "entry_key.h"
#include "as608.h"  // as608 packages
#include "easyflash.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define DBG_TAG              "key.fp"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#ifndef AS60X_UART_NAME
#define AS60X_UART_NAME "uart2"
#endif

static struct entry_key en_key_fp = {0};

static void show_fp_key_in_flash()
{
    char buff[KEY_IN_FLASH_LEN] = {""};
    ef_get_env_blob("fp-key", buff, KEY_IN_FLASH_LEN, NULL);
    rt_kprintf("fp-key:%s\n", buff);
}
MSH_CMD_EXPORT(show_fp_key_in_flash, "show_fp_key_in_flash");

//1. [x]验证函数指针传递参数是否正确
//2. []与底层函数接口进行对接
//3. []对底层函数不匹配的接口进行改造
static void fp_add_key(user_info_t info)
{
    uint16_t id = atoi(info->name);
    rt_kprintf("fp_add_key:%s atoi:0x%x\n", info->name, id);
    as60x_str_fp_to_flash(id);
    strncpy(info->key_fp, info->name, FP_KEY_LEN);
    fills_to_len_with_zero(info->key_fp, FP_KEY_LEN);
    str_key_to_flash("fp_key", info->name, info->key_fp);
}

static void fp_del_key(user_info_t info)
{
    rt_kprintf("fp_del_key:%s atoi:%d\n", info->name, atoi(info->name));
    uint16_t id = atoi(info->name);
    as60x_delet_fp_n_id(id, 1);
    strcpy(info->key_pw, "xxxxxx");
    del_key_in_flash("fp-key", info->name, info->key_fp); ///> delete password in flash
}

static uint8_t fp_ver_key(char *name)
{
    uint8_t ret = USR_CHECK_AUTH_ERR;
    uint16_t id = 0;
    uint16_t score = 0;
    rt_event_send(get_key_det_evt(), EVT_GRD_DET_FP);
    as60x_search_fp_in_flash(&id, &score);
    if (score > 20)
    {
        snprintf(name, NAME_BUF_LEN, "%d", id);
        ret = USR_CHECK_AUTH_OK;
    }
    return ret;
}

static int rt_hw_fp_port(void)
{
    as60x_set_hand_shake_baud(115200);
    as60x_init(AS60X_UART_NAME);

    en_key_fp.add_key = fp_add_key;
    en_key_fp.del_key = fp_del_key;
    en_key_fp.ver_key = fp_ver_key;
    reg_key_obj(ENTRY_KEY_FP, &en_key_fp);

    fp_wak_evt_reg(get_key_det_evt(), EVT_GRD_DET_FP);
    return 0;
}
INIT_APP_EXPORT(rt_hw_fp_port);
