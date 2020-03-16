#include "entry_key.h"
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG              "key.main"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

static const char key_desc[CFG_KEY_USING_NUM][3] = { "PW","FP","RF","FD" };
static char *key_point[CFG_KEY_USING_NUM] = {0}; // 需要空间
static entry_key_t key_table[KEY_TYPES_MAX] = {0};
static rt_event_t kdet_evt = RT_NULL;
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

    rt_event_recv(kdet_evt, (EVT_KEY_DET_FP|EVT_KEY_DET_RF), RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &set);
    if (flag_add_key) // 娣诲姞閽ュ寵鏃剁瓑寰�
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

static size_t value_wrap(uint8_t flag, user_info_t info, void *value_buf, size_t buf_len)
{
    char desc_buf[2];
    uint8_t tmp;
    char *tmp_str = NULL;
    size_t len = 0;
    uint8_t i;

    if (flag == 1) {
        for (i=ENTRY_KEY_PW; i < ENTRY_KEY_FD; i++)
        {
            if (key_point[i] == NULL) continue;
            sprintf(desc_buf, "%d", i);
            strcpy(value_buf, desc_buf);
            strncat(value_buf, key_point[i], USER_KEY_LEN-1);
            strcat(value_buf, ",");
        }
    }
    else {
        tmp_str = strtok(value_buf, ",");
        do {
            tmp = tmp_str[0]-'0';
            strncpy(key_point[tmp], tmp_str+1, USER_KEY_LEN);
            strncpy(tmp_str, strtok(NULL, ","), USER_KEY_LEN);
            tmp_str = strtok(NULL, ",");
        } while (tmp_str != NULL);
    }
}

static void input_id(user_info_t info)
{
    char user_name[4] = "001";
    char buff[USER_KEY_LEN];
    size_t size;

#ifdef CFG_USING_KEYBOARD

#else
    memcpy(info->name, user_name, USER_NAME_LEN);
#endif
    ef_get_env_blob(info->name, NULL, 0, &size);
    if (size != 0) {
        ef_get_env_blob(info->name, buff, size, NULL);
    }
}
static void check_user_auth();
static void input_key();
static void save_info(user_info_t info)
{
//    ef_set_env_blob(info->name, )
}

void add_user(void)
{
    struct user_info info = {0};
    input_id(&info);
    check_user_auth();
    input_key();
    save_info(&info);
}

static int entry_key_init(void)
{
#if CFG_KEY_USING_PW
    key_point[ENTRY_KEY_PW] = calloc(USER_KEY_LEN, sizeof(char));
#endif
#if CFG_KEY_USING_FP
    key_point[ENTRY_KEY_FP] = calloc(USER_KEY_LEN, sizeof(char));
#endif
#if CFG_KEY_USING_RF
    key_point[ENTRY_KEY_RF] = calloc(USER_KEY_LEN, sizeof(char));
#endif
#if CFG_KEY_USING_FD
    key_point[ENTRY_KEY_FD] = calloc(USER_KEY_LEN, sizeof(char));
#endif

    kdet_evt = rt_event_create("kdet-evt", RT_IPC_FLAG_FIFO);
    if (RT_NULL == kdet_evt)
        LOG_E("create kdet_evt failed!\n");

    rt_thread_t tid = RT_NULL;
    tid = rt_thread_create("kdet-tid", check_auth_test, RT_NULL, 512, 14, 10);
    if (tid != RT_NULL)
//        rt_thread_startup(tid);
    return 0;
}
//INIT_COMPONENT_EXPORT(entry_key_init);
