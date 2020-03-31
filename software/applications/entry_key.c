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
    case EVT_KEY_DET_FP:
        id = key_table[ENTRY_KEY_FP]->ver_key();
        break;
    case EVT_KEY_DET_RF:
        id = key_table[ENTRY_KEY_RF]->ver_key();
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
    key_table[key_type]->add_key(id);
    flag_add_key = 0;
}

void add_auth_test(void *p)
{
    // add_auth(ENTRY_KEY_FP, 0x01);
    add_auth(ENTRY_KEY_RF, 0x01);
}
MSH_CMD_EXPORT(add_auth_test, "add auth test");

/**
 *
 * @param flag 0:value_buf->info 1:info->value_buf
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
    key_point[2] = info->key_rc;
    key_point[3] = info->key_fd;

    if (flag == 1) {
        for (i=ENTRY_KEY_PW; i < ENTRY_KEY_FD; i++)
        {
            if (key_point[i] == NULL) continue;
            sprintf(desc_buf, "%d", i);
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

void value_wrap_test()
{
    struct user_info info = {"111", "0", "0", "0", "0"};
    char buff[USER_INFO_SIZE] = {0};
    ef_get_env_blob(info.name, buff, USER_INFO_SIZE, NULL);
    if (strlen(buff) > 0) { // Fulfill info->name
        rt_kprintf("user:%s is in flash, buff:%s\n", info.name, buff);
        value_wrap(0, &info, buff, USER_INFO_SIZE);
    }
    rt_kprintf("value_wrap_test: %s %s %s %s\n", info.key_pw, info.key_fp, info.key_rc, info.key_fd);
}
MSH_CMD_EXPORT(value_wrap_test, "value_wrap_test");

static void get_value_from_outcome(uint8_t *buffer, uint8_t* len)
{
    rt_event_recv(kdet_evt, EVT_GRD_DET_PW, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
    *len = keypad_get_len();
    keypad_get_n_value(buffer, *len);
}

static void input_user_name(user_info_t info)
{
//    RT_ASSERT(info->name != NULL);
    char user_name[USER_NAME_LEN] = "001";
    char buff[USER_INFO_SIZE] = {0};

    LOG_I("input user name:");
    uint8_t buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = 0;
    uint8_t i = 0;
    uint8_t j = 0;

    get_value_from_outcome(buffer, &len);
    if (len > USER_NAME_LEN)
        i = len - USER_NAME_LEN;
    for (; i < len-1; ++i) {
        user_name[j++] = keypad_num_table[buffer[i]];
    }
    strcpy(info->name, user_name);
    rt_kprintf("%s\n", info->name);

    ef_get_env_blob(info->name, buff, USER_INFO_SIZE, NULL);
    if (strlen(buff) > 0) { // Fulfill info->name
        rt_kprintf("user:%s is in flash, buff:%s\n", info->name, buff);
        value_wrap(0, info, buff, USER_INFO_SIZE);
    }
}

static rt_err_t check_root_auth(const user_info_t info)
{
    char root[(USER_NAME_LEN+1)*3] = {0};
    ef_get_env_blob("root", root, (USER_NAME_LEN+1)*3, NULL);

    if (strlen(root) < USER_NAME_LEN) { // zero root user
        ef_set_env_blob("root", info->name, USER_NAME_LEN);
        return RT_EOK;
    }

    if (strstr(root, info->name) != NULL) {
        return RT_EOK;
    }

    return RT_ERROR;
}

/**
 *
 * @return 0:error 1:add user
 */
static uint8_t input_user_option()
{
    LOG_I("input user option:");
    uint8_t buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = 0;
    uint8_t i = 0;

    get_value_from_outcome(buffer, &len);
    if (len > 2)
        i = len - 2;
    return buffer[i];
}

static void input_user_key_type()
{
//    TODO make some choice here
//    1. enter the key type
//    2. connect to low level key option
    LOG_I("input user key type:");
    uint8_t buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = 0;
    uint8_t i = 0;

    get_value_from_outcome(buffer, &len);
    if (len > PW_KEY_LEN)
        i = len - PW_KEY_LEN;
    for (; i < len; ++i) {
        rt_kprintf("%d ", buffer[i]);
    }
    rt_kprintf("\n");
}

static void save_user_info(user_info_t info)
{
    char buff[USER_INFO_SIZE] = {0};
    value_wrap(1, info, buff, USER_INFO_SIZE);
    LOG_D("save_user_info:%s", buff);
    ef_set_env_blob(info->name, buff, USER_INFO_SIZE);
}

rt_err_t guard_add_usr(user_info_t info)
{
    rt_err_t ret = RT_EOK;
    input_user_key_type();
    save_user_info(info);
    return ret;
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

//void guard_menu(void)
//{
//    keypad_get_value();
//}

/**
 * password event process
 */
static void pw_evt_proc(void)
{
    uint8_t buffer[KEYPAD_FIFO_SIZE] = {0};
    uint8_t len = keypad_get_len();
    LOG_D("pad data len:%d", len);
    keypad_get_n_value(buffer, len);

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
}

static void wait_key_single(rt_int32_t timeout)
{
    rt_uint32_t set = 0;
    rt_event_recv(kdet_evt, EVT_GRD_DET_PW|EVT_GRD_DET_FP|EVT_GRD_DET_RF|EVT_GRD_DET_FD, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, timeout, &set);
    switch (set) {
        case EVT_GRD_DET_PW:
            LOG_D("get keypad event");
            pw_evt_proc();
            break;
        case EVT_GRD_DET_FP:
            rt_kprintf("get finger print event\n");
            break;
        case EVT_GRD_DET_RF:
            rt_kprintf("get RF event\n");
            break;
        case EVT_GRD_DET_FD:
            rt_kprintf("get face event\n");
            break;
        default:
            break;
    }
}

static void guard_wkng_thread(void *p)
{
    struct user_info info = {"", "1", "0", "0", "0"};
    rt_err_t ret = RT_EOK;
    uint8_t option = 0;

    while (1)
    {
        switch (flag_wkng_mode) {
            case WKNG_MODE_ROOT:
                input_user_name(&info);
                ret = check_root_auth(&info);
                if (!ret) LOG_I("root check ok!");
                option = input_user_option();
                switch (option) {
                    case 1:
                        rt_kprintf("add user\n");
                        guard_add_usr(&info);
                        break;
                    default:
                        break;
                }
                guard_set_wkng_mode(WKNG_MODE_NORM);
                break;
            case WKNG_MODE_NORM:
                wait_key_single(50); //event?
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
    kdet_evt = rt_event_create("kdet-evt", RT_IPC_FLAG_FIFO);
    if (RT_NULL == kdet_evt)
        LOG_E("create kdet_evt failed!\n");

    rt_thread_t tid = RT_NULL;
    tid = rt_thread_create("grd-tid", guard_wkng_thread, RT_NULL, 2048, 14, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return 0;
}
INIT_COMPONENT_EXPORT(entry_guard_init);
