#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <board.h>
#include "entry_key.h"
#include "fal.h"
#include "easyflash.h"
#include "matrix_keypad.h"

#define DBG_TAG              "key.main"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

static uint8_t wait_key_single(char *name, rt_int32_t timeout);

//static const char key_desc[CFG_KEY_TOTAL_NUM][3] = { "PW","FP","RF","FD" };
static entry_key_t key_table[KEY_TYPES_MAX] = {0};
static rt_event_t kdet_evt = RT_NULL;
static enum wkng_mode_type flag_wkng_mode = WKNG_MODE_NORM; // working mode 0:root 1:normal 2:idle
static uint8_t flag_add_key = 0;

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
 * @brief Get the key det evt object
 * 
 * @return rt_event_t 
 */
rt_event_t get_key_det_evt(void)
{
    return kdet_evt;
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
    if (key_table[key_type] != RT_NULL)
    {
        LOG_W("Warning! Type of %d has been registered!", (uint8_t)key_type);
    }

    key_table[key_type] = key_obj; 
}

/**
 * @brief Check authority
 * 
 * @return uint16_t 
 */
uint16_t check_auth(void)
{
    uint16_t id = KEY_VER_ERROR;
    uint32_t set = 0;

//    rt_event_recv(kdet_evt, (EVT_KEY_DET_FP|EVT_KEY_DET_RF), RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &set);
    if (flag_add_key)
        return id;
    switch (set)
    {
//    case EVT_KEY_DET_FP:
//        id = key_table[ENTRY_KEY_FP]->ver_key();
//        break;
//    case EVT_KEY_DET_RF:
//        id = key_table[ENTRY_KEY_RF]->ver_key();
    default:
        break;
    }
    return id;
}

void check_auth_test(void *p)
{
    uint16_t id = KEY_VER_ERROR;
    while (1)
    {
        id = check_auth();
        LOG_I("id:%x", id);
    }
}
//MSH_CMD_EXPORT(check_auth_test, "check auth test");

/**
 * @brief Add authority
 * 
 * @param key_type 
 * @param id 
 */
void add_auth(enum entry_key_type key_type, uint16_t id)
{
    flag_add_key = 1;
    RT_ASSERT(key_table[key_type] != RT_NULL);
//    key_table[key_type]->add_key(id);
    flag_add_key = 0;
}

void add_auth_test(void *p)
{
    // add_auth(ENTRY_KEY_FP, 0x01);
    add_auth(ENTRY_KEY_RF, 0x01);
}
MSH_CMD_EXPORT(add_auth_test, "add auth test");

/**
 * @brief control door open
 * @param lock_state
 * @ref USR_CHECK_AUTH_OK  open
 * @ref USR_CHECK_AUTH_ERR do nothing
 */
static void lock_control(uint16_t lock_state)
{
    static uint8_t tick;
    if (lock_state == USR_CHECK_AUTH_OK) {
        tick = 41;
        rt_pin_write(LOCK_PIN_NUM, PIN_HIGH);
        LOG_I("door open!");
    }
    if (tick > 0) {
        tick--;
    }
    if (tick == 1) {
        rt_pin_write(LOCK_PIN_NUM, PIN_LOW);
        LOG_I("door close!");
    }
}

void fills_to_len_with_zero(char *key, uint8_t len)
{
    char zero_fill[PW_KEY_LEN+1] = {"000000"};
    uint8_t n_len = strlen(key);
    uint8_t i,j = 0;
    if (n_len != len)
    {
        i = PW_KEY_LEN - n_len;
        LOG_D("key before filled:%s", key);
        while (i < PW_KEY_LEN)
        {
            zero_fill[i] = key[j];
            i++;
            j++;
        }
        strncpy(key, zero_fill, PW_KEY_LEN);
        LOG_D("key after filled:%s", key);
    }
}

//void fills_to_len_with_zero_test()
//{
//    char key[USER_KEY_LEN] = {"111"};
//    fills_to_len_with_zero(key, PW_KEY_LEN);
//}
//MSH_CMD_EXPORT(fills_to_len_with_zero_test, "fills_to_len_with_zero_test");

uint8_t if_key_in_flash(const char *key_type, char *name, const char *key)
{
    RT_ASSERT(name != NULL);
    char buff[KEY_IN_FLASH_LEN] = {""}; // key-type name(NAME_STR_LEN):key(PW_KEY_LEN),name:key,('\0')
    char *name_p = NULL;
    ef_get_env_blob(key_type, buff, KEY_IN_FLASH_LEN, NULL);
    name_p = strstr(buff, key);

    if (name_p != NULL)
    {
        strncpy(name, name_p-(NAME_STR_LEN+1), NAME_STR_LEN);
        return USR_CHECK_AUTH_OK;
    }
    return USR_CHECK_AUTH_ERR;
}

/**
 * @brief store key into flash(key_type), format with name(NAME_STR_LEN):key(XX_KEY_LEN). cooperate with the add_key func.
 * @param key_type
 * @param name
 * @param key
 */
void str_key_to_flash(const char *key_type, const char *name, const char *key)
{
    char *buff_name_p = NULL;
    const char *key_p = key;
    char buff[KEY_IN_FLASH_LEN] = {""}; // key-type name(NAME_STR_LEN):key(PW_KEY_LEN),name:key,('\0')
    uint8_t i = 0;
    ef_get_env_blob(key_type, buff, KEY_IN_FLASH_LEN, NULL);

    LOG_D("before %s %s:%s", __func__, key_type, buff);
    buff_name_p = strstr(buff, name);
    if (buff_name_p != NULL) // 修改存储在 flash 中的 key_type 的信息
    {
        // buff_name_p 为找到 name 的位置，(NAME_STR_LEN+1)为 name: 的长度
        while(*key_p != '\0')
        {
            *(buff_name_p+(NAME_STR_LEN+1)+i) = *(key_p++);
            i++;
        }
    }
    else
    {
        strncat(buff, name, NAME_STR_LEN);
        strcat(buff, ":");
        strncat(buff, key, PW_KEY_LEN);
        strcat(buff, ",");
    }
    LOG_D("after %s %s:%s", __func__, key_type, buff);
    ef_set_env_blob(key_type, buff, KEY_IN_FLASH_LEN); // 修改后的 key_type 回写到 flash
}

/**
 * @brief delete the selected key in flash. cooperate with the del_key func.
 * @param key_type
 * @param name
 * @param key
 */
void del_key_in_flash(const char *key_type, const char *name, const char *key)
{
    char *buff_name_p = NULL;
    char buff[KEY_IN_FLASH_LEN] = {""}; // key-type name(NAME_STR_LEN):key(PW_KEY_LEN),name:key,('\0')
    ef_get_env_blob(key_type, buff, KEY_IN_FLASH_LEN, NULL);

    LOG_D("before %s %s:%s", __func__, key_type, buff);
    buff_name_p = strstr(buff, name);
    if (buff_name_p != NULL) // 修改存储在 flash 中的 key_type 的信息，由于没有相关需求，功能正确性未验证
    {
//        if (*(buff_name_p+NAME_STR_LEN+PW_KEY_LEN+2) != '\0') // TODO 需要删除空的环境变量吗，可以考虑
            strncpy(buff_name_p, buff_name_p+NAME_STR_LEN+PW_KEY_LEN+2, KEY_IN_FLASH_LEN-1);
//        else
//        {
//            ef_del_env(key_type);
//            return;
//        }
    }
    LOG_D("after %s %s:%s", __func__, key_type, buff);
    ef_set_env_blob(key_type, buff, KEY_IN_FLASH_LEN); // 修改后的 key_type 回写到 flash
}

/**
 *
 * @param flag 逆运行标志位
 * @ref 0 value_buf->info
 * @ref 1 info->value_buf
 * @param info
 * @param value_buf
 * @param buf_len
 */
static void value_wrap(uint8_t flag, user_info_t info, void *value_buf, size_t buf_len)
{
    RT_ASSERT(value_buf != NULL);
    char desc_buf[2] = {0};
    uint8_t tmp;
    char *tmp_str = NULL;
    uint8_t i;
    char *key_point[CFG_KEY_TOTAL_NUM] = {0};

    key_point[0] = info->key_pw;
    key_point[1] = info->key_fp;
    key_point[2] = info->key_rf;
    key_point[3] = info->key_fd;

    if (flag == 1) {
        for (i=ENTRY_KEY_PW; i < ENTRY_KEY_FD; i++)
        {
            if (key_point[i] == NULL) continue;
            snprintf(desc_buf, 2, "%d", i);
            strcat(value_buf, desc_buf);
            strncat(value_buf, key_point[i], USER_KEY_LEN-1);
            strcat(value_buf, ",");
        }
    }
    else {
        tmp_str = strtok(value_buf, ",");
        do {
            tmp = tmp_str[0]-'0';
            strncpy(key_point[tmp], tmp_str+1, USER_KEY_LEN);
            tmp_str = strtok(NULL, ",");
        } while (tmp_str != NULL);
    }
}

/**
 *
 * @param buffer malloc space first!
 * @param len
 */
static void guard_get_value_from_outcome(char *buffer, uint8_t *len)
{
    keypad_get_n_value_b(buffer, len);
}

static void get_user_info_from_flash(user_info_t info)
{
    char buff[USER_INFO_SIZE] = {0};
    //    search info->name in flash env
    ef_get_env_blob(info->name, buff, USER_INFO_SIZE, NULL);
    if (strlen(buff) > 0) { // Fulfill info->name
        rt_kprintf("user:%s is in flash, buff:%s\n", info->name, buff);
        value_wrap(0, info, buff, USER_INFO_SIZE);
    }
}

static void get_user_info_from_flash_test()
{
    struct user_info info = {"111", "0", "0", "0", "0"};
    get_user_info_from_flash(&info);
    rt_kprintf("get_user_info_from_flash_test: %s %s %s %s\n", info.key_pw, info.key_fp, info.key_rf, info.key_fd);
}
MSH_CMD_EXPORT(get_user_info_from_flash_test, "get_user_info_from_flash_test");

static void input_user_name(char *name)
{
//    RT_ASSERT(info->name != NULL);
    char buff[KEYPAD_FIFO_SIZE] = {""};

    LOG_I("input user name:");
    uint8_t len = 0;
    uint8_t i = 0;

    guard_get_value_from_outcome(buff, &len);
    if (len > NAME_STR_LEN+1) // 111#
        i = len - NAME_STR_LEN+1;
    keypad_num_to_str(buff, NAME_STR_LEN);
    strncpy(name, buff+i, NAME_STR_LEN);
}

/**
 * @brief prefix store in root
 * @param root space must larger than ROOT_USER_LEN
 * @param name root user name
 */
static void prefix_root_env(char *root, const char *name)
{
    rt_kprintf("root:%s strlen(root):%d root check\n", root, strlen(root));
    strncat(root, name, NAME_STR_LEN);
    strcat(root, ",");
}

static void add_root_auth(const char *name)
{
    char root[ROOT_USER_LEN] = {""}; // 1 is the split of ,
    ef_get_env_blob("root", root, ROOT_USER_LEN, NULL); // format: 111,222,333,

    prefix_root_env(root, name);
    ef_set_env_blob("root", root, ROOT_USER_LEN);
}

static rt_err_t check_root_auth(const user_info_t info)
{
    char root[ROOT_USER_LEN] = {0}; // 1 is the split of ,
    ef_get_env_blob("root", root, ROOT_USER_LEN, NULL); // format: 111,222,333,

    if (strlen(root) < NAME_STR_LEN+1) { // zero root user
        prefix_root_env(root, info->name);
        ef_set_env_blob("root", root, ROOT_USER_LEN);
        return RT_EOK;
    }

    if (strstr(root, info->name) != NULL) {
        return RT_EOK;
    }

    return RT_ERROR;
}

/**
 *
 * @return user option
 * - USR_OP_ERROR error
 * - USR_OP_ADD_NUSR add user
 * - USR_OP_ADD_RUSR add root user
 * - USR_OP_DEL_NUSR delete user
 */
static uint8_t input_user_option()
{
    LOG_I("input user option(1:add user 2:add root user 3:delete user ):");
    char buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = 0;
    uint8_t i = 0;

    guard_get_value_from_outcome(buffer, &len);
    if (len > 2)
        i = len - 2;
    return buffer[i];
}

/**
 *
 * @param info
 * @param option
 */
static void input_user_key_type(user_info_t info, uint8_t option)
{
//    1. enter the key type
//    2. connect to low level key option
    LOG_I("input user key type:(1:pw 2:fp 3:rf)");
    char buff[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = 0;
    uint8_t i = 0;

    guard_get_value_from_outcome(buff, &len);
    if (len > PW_KEY_LEN)
        i = len - PW_KEY_LEN;
    switch (option) {
        case USR_OP_ADD_NUSR:
            key_table[buff[i]-1]->add_key(info);
            break;
        case USR_OP_ADD_RUSR:
            // TODO 添加 root 验证
            input_user_name(buff);
            add_root_auth(buff);
            break;
        case USR_OP_DEL_NUSR:
            key_table[buff[i]-1]->del_key(info);
            break;
        default:
            break;
    }
}

static void save_user_info(user_info_t info)
{
    char buff[USER_INFO_SIZE] = {0};
    value_wrap(1, info, buff, USER_INFO_SIZE);
    LOG_D("save_user_info:%s", buff);
    ef_set_env_blob(info->name, buff, USER_INFO_SIZE);
}

void stop_test(void *p)
{
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
}
MSH_CMD_EXPORT(stop_test, "stop test");

/**
 *
 * @param mode
 */
void guard_set_wkng_mode(enum wkng_mode_type mode)
{
    flag_wkng_mode = mode;
}

/**
 * password event process
 */
static uint8_t pw_evt_proc(char *name)
{
    char buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t ret = USR_CHECK_AUTH_ERR;
    uint8_t len = 0;
    rt_event_send(kdet_evt, EVT_GRD_DET_PW);
    guard_get_value_from_outcome(buffer, &len);
    LOG_D("pad data len:%d", len);

#if DBG_LVL == DBG_LOG
//    for (int i = 0; i < len; ++i) {
//        rt_kprintf("%d ", buffer[i]);
//    }
//    rt_kprintf("\n");
#endif

    if (2 == len)
    {
        if (KEYPAD_NUM_STAR == buffer[0])
            guard_set_wkng_mode(WKNG_MODE_ROOT);
    }

    if (len >= PW_KEY_LEN)
    {
        //TODO 考虑 pw_ver_key 这里直接传递前面 guard_get_value_from_outcome 读出的内容
        ret = pw_ver_key(name, buffer, len);
//        ret = key_table[ENTRY_KEY_PW]->ver_key(name);
        rt_kprintf("name:%s buffer:%s\n", name, buffer);
    }
    return ret;
}

static uint8_t fp_evt_proc(char *name)
{
    uint8_t ret = USR_CHECK_AUTH_ERR;
    ret = key_table[ENTRY_KEY_FP]->ver_key(name);
    LOG_D("ret:%d name:%s", ret, name);
    return ret;
}

static uint8_t rf_evt_proc(char *name)
{
    uint8_t ret = USR_CHECK_AUTH_ERR;
    ret = key_table[ENTRY_KEY_RF]->ver_key(name);
//    LOG_D("ret:%d name:%s", ret, name);
    return ret;
}

static uint8_t wait_key_single(char *name, rt_int32_t timeout)
{
    rt_uint32_t set = 0;
    uint8_t ret = USR_CHECK_AUTH_ERR;

//    TODO 预留云端控制的接口
    rt_event_recv(kdet_evt, EVT_GRD_DET_PW|EVT_GRD_DET_FP|EVT_GRD_DET_RF|EVT_GRD_DET_FD, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, timeout, &set);
    switch (set) {
        case EVT_GRD_DET_PW:
            LOG_D("get keypad event");
            ret = pw_evt_proc(name);
            break;
        case EVT_GRD_DET_FP:
            LOG_D("get finger print event");
            ret = fp_evt_proc(name);
            break;
        case EVT_GRD_DET_RF:
            ret = rf_evt_proc(name);
            break;
        case EVT_GRD_DET_FD:
            LOG_D("get face event");
            break;
        default:
            break;
    }
    return ret;
}

static void guard_wkng_thread(void *p)
{
    struct user_info info = {"", "0", "0", "0", "0"};
    char name[NAME_BUF_LEN] = {""};
    rt_err_t ret = RT_EOK;
    uint8_t key_ver_ok = USR_CHECK_AUTH_ERR;
    uint8_t option = 0;

    while (1)
    {
        switch (flag_wkng_mode) {
            case WKNG_MODE_ROOT:
                input_user_name(info.name);
                get_user_info_from_flash(&info);
                ret = check_root_auth(&info);
                if (!ret) LOG_I("root check ok!");
                else
                {
                    LOG_I("root check fail!");
                    guard_set_wkng_mode(WKNG_MODE_NORM);
                    continue;
                }
                option = input_user_option();
                input_user_key_type(&info, option);
                save_user_info(&info);
                guard_set_wkng_mode(WKNG_MODE_NORM);
                break;
            case WKNG_MODE_NORM:
                key_ver_ok = wait_key_single(name, 50); //event?
                lock_control(key_ver_ok); ///< 门锁控制，自动关门
//                 TODO 上报开门的 name 和 key-type
                break;
            case WKNG_MODE_IDLE:
//                rt_kprintf("C");
                break;
            default:
                break;
        }
    }

}

static int entry_guard_init(void)
{
    rt_pin_mode(LOCK_PIN_NUM, PIN_MODE_OUTPUT);
//    rt_pin_write(LOCK_PIN_NUM, PIN_HIGH);
//    rt_thread_mdelay(2000);
    rt_pin_write(LOCK_PIN_NUM, PIN_LOW);

    kdet_evt = rt_event_create("kdet-evt", RT_IPC_FLAG_FIFO);
    if (RT_NULL == kdet_evt)
        LOG_E("create kdet_evt failed!\n");

    rt_thread_t tid = RT_NULL;
    tid = rt_thread_create("grd-tid", guard_wkng_thread, RT_NULL, 2048, 14, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return 0;
}
INIT_ENV_EXPORT(entry_guard_init);
