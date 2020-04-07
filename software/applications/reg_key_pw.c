#include "entry_key.h"
#include "matrix_keypad.h"
#include "easyflash.h"

#define DBG_TAG              "key.pw"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

static struct entry_key en_key_pw = {0};

static void show_pw_key_in_flash()
{
    char buff[KEY_IN_FLASH_LEN] = {""};
    ef_get_env_blob("pw-key", buff, KEY_IN_FLASH_LEN, NULL);
    rt_kprintf("pw-key:%s\n", buff);
}
MSH_CMD_EXPORT(show_pw_key_in_flash, "show_pw_key_in_flash");

static void transform_keypad_input(char *tmp_buf, uint8_t buff_len)
{
    uint8_t len = 0;
    uint8_t i = 0;
    keypad_get_n_value_b(tmp_buf, &len);
    if (len > (PW_KEY_LEN+1))
        i = len - (PW_KEY_LEN+1);

    strncpy(tmp_buf, tmp_buf+i, PW_KEY_LEN);
    keypad_num_to_str(tmp_buf, PW_KEY_LEN);
    memset(tmp_buf+PW_KEY_LEN, '\0', buff_len-PW_KEY_LEN);
}

static void pw_add_key(user_info_t info)
{
    LOG_D("pw add key:");
    char tmp_buf[KEYPAD_FIFO_SIZE] = {""};
    transform_keypad_input(tmp_buf, KEYPAD_FIFO_SIZE);
    strncpy(info->key_pw, tmp_buf, PW_KEY_LEN);
    str_key_to_flash("pw-key", info->name, info->key_pw); ///> store password into flash
}

static void pw_del_key(user_info_t info)
{
    strcpy(info->key_pw, "xxxxxx");
    del_key_in_flash("pw-key", info->name, info->key_pw); ///> delete password in flash
}

static void tmp_transform_keypad_input(char *tmp_buf, uint8_t len)
{
//    uint8_t len = len;
    uint8_t i = 0;
//    keypad_get_n_value_b(tmp_buf, &len);
    if (len > (PW_KEY_LEN+1))
        i = len - (PW_KEY_LEN+1);

    strncpy(tmp_buf, tmp_buf+i, PW_KEY_LEN);
    keypad_num_to_str(tmp_buf, PW_KEY_LEN);
    memset(tmp_buf+PW_KEY_LEN, '\0', KEYPAD_FIFO_SIZE-PW_KEY_LEN);
}

// TODO 修复 PW 类型接口无法匹配现有架构
// - 可以考虑通过特定按键进入密码匹配模式
//static uint8_t pw_ver_key(char *name)
uint8_t pw_ver_key(char *name, char *key, uint8_t len)
{
    uint8_t ret = USR_CHECK_AUTH_ERR;
//    RT_ASSERT(name != NULL);
//    char tmp_buf[KEYPAD_FIFO_SIZE] = {""};
//    transform_keypad_input(tmp_buf, KEYPAD_FIFO_SIZE);
//    if_key_in_flash("pw-key", name, tmp_buf);
//    rt_kprintf("name:%s key:%s\n", name, tmp_buf);
    tmp_transform_keypad_input(key, len);
    ret = if_key_in_flash("pw-key", name, key);
    rt_kprintf("name:%s key:%s\n", name, key);
    return ret;
}
MSH_CMD_EXPORT(pw_ver_key, "pw_ver_key");

static int rt_hw_pw_port(void)
{
    en_key_pw.add_key = pw_add_key;
    en_key_pw.del_key = pw_del_key;
//    en_key_pw.ver_key = pw_ver_key;
    reg_key_obj(ENTRY_KEY_PW, &en_key_pw);

    return 0;
}
INIT_APP_EXPORT(rt_hw_pw_port);
