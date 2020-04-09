#include "entry_key.h"
#include "mfrc522.h" // rc522 packages

#include <string.h>

#define DBG_TAG              "key.rc"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#define EVENT_SCAN_START    (1 << 0)
#define EVENT_SCAN_END      (1 << 1)
#define EVENT_MB_WR_ENABLE  (1 << 2)

static rt_mailbox_t rf_mb = NULL;

static Uid *uid;
static rt_event_t rf_evt = RT_NULL;
static uint8_t anti_repeat = 0;

static struct entry_key en_key_rc = {0};

static void show_rf_key_in_flash()
{
    char buff[KEY_IN_FLASH_LEN] = {""};
    ef_get_env_blob("rf_key", buff, KEY_IN_FLASH_LEN, NULL);
    rt_kprintf("rf_key:%s\n", buff);
}
MSH_CMD_EXPORT(show_rf_key_in_flash, "show_rf_key_in_flash");

// TODO 添加写 flash
static void rc_add_key(user_info_t info)
{
//    uint16_t id = atoi(info->name);
    char *str = NULL;

    rt_event_send(rf_evt, EVENT_MB_WR_ENABLE);
    rt_mb_recv(rf_mb, (rt_uint32_t *)&str, RT_WAITING_FOREVER);
    strncpy(info->key_rf, str+2, RF_KEY_LEN);
    str_key_to_flash("rf_key", info->name, info->key_rf);
}

static void rc_del_key(user_info_t info)
{
//    uint16_t id = atoi(info->name);
    char *str = NULL;
//    rt_event_send(rf_evt, EVENT_MB_WR_ENABLE);
//    rt_mb_recv(rf_mb, (rt_uint32_t *)&str, RT_WAITING_FOREVER);
    strcpy(info->key_rf, "xxxxxx");
    del_key_in_flash("rf_key", info->name, info->key_rf); ///> delete password in flash
}

static uint8_t rc_ver_key(char *name)
{
    uint8_t ret = USR_CHECK_AUTH_ERR;
    char *str = NULL;

    if (anti_repeat > 0)
    {
        anti_repeat--;
        return ret;
    }
    else
    {
        anti_repeat = 10;
    }

    rt_event_send(rf_evt, EVENT_MB_WR_ENABLE);
    rt_mb_recv(rf_mb, (rt_uint32_t *)&str, RT_WAITING_FOREVER);
    ret = if_key_in_flash("rf_key", name, str+2);
    LOG_D("rc key ver, name:%s key:%s", name, str+2);
    return ret;
}

static void uid_into_buff(Uid *uid, char *buff, uint8_t buff_size)
{
    char tmp_buff[3] = {""};
    memset(buff, '\0', buff_size);
    for (byte i = 0; i < uid->size; i++)
    {
        if(uid->uidByte[i] < 0x10)
            strcat(buff, "0");
        snprintf(tmp_buff, 3, "%x", uid->uidByte[i]);
        strncat(buff, tmp_buff, 2);
    }
}

static void rc_init_thread(void *param)
{
    char id_buff[USER_KEY_LEN] = {""};
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
    PCD_Init();		// Init MFRC522
    rt_thread_mdelay(4);
    uid = get_uid();

    rf_mb = rt_mb_create("rc-mb", 1, RT_IPC_FLAG_FIFO);
    if (NULL == rf_mb)
    {
        LOG_E("rf_mb create fail!");
        goto _FAIL;
    }

    while (1)
    {
        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
        if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial())
        {
            rt_thread_mdelay(50);
            continue;
        }

        // Dump debug info about the card; PICC_HaltA() is automatically called
//        rt_thread_mdelay(500);
        uid_into_buff(uid, id_buff, USER_KEY_LEN);
//        rt_kprintf("id_buff:%s %x %x %x %x\n", id_buff, uid->uidByte[0], uid->uidByte[1], uid->uidByte[2], uid->uidByte[3]);

        if (!rt_event_recv(rf_evt, EVENT_MB_WR_ENABLE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, NULL))
        {
//            LOG_I("read uid:%u", tmp_id);
//            rt_mb_send(rf_mb, tmp_id);
            rt_mb_send(rf_mb, (rt_uint32_t)&id_buff);
        }
        else
        {
            rt_event_send(get_key_det_evt(), EVT_GRD_DET_RF);
        }
        // break;
    }
_FAIL:
     PCD_End();
}

static int rt_hw_rc_port(void)
{
    en_key_rc.add_key = rc_add_key;
    en_key_rc.del_key = rc_del_key;
    en_key_rc.ver_key = rc_ver_key;
    reg_key_obj(ENTRY_KEY_RF, &en_key_rc);

    rf_evt = rt_event_create("rfk-evt", RT_IPC_FLAG_FIFO);
    if (RT_NULL == rf_evt)
        LOG_E("create rf_evt failed!");

    rt_thread_t rc_tid = RT_NULL;
    rc_tid = rt_thread_create("rfk-tid", rc_init_thread, RT_NULL, 1024, 15, 10);
    if (rc_tid != RT_NULL)
        rt_thread_startup(rc_tid);

    return 0;
}
INIT_APP_EXPORT(rt_hw_rc_port);

