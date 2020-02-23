#include "entry_key.h"
#include "mfrc522.h" // rc522 packages

#define DBG_TAG              "key.rc"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#define EVENT_SCAN_START    (1 << 0)
#define EVENT_SCAN_END      (1 << 1)

static Uid *uid;
static rt_event_t rc_evt = RT_NULL;
static uint32_t tmp_id = 0;

static struct entry_key en_key_rc = {0};
static uint32_t rc_id[KEY_ID_MAX] = {0};

static void rc_add_key(uint16_t id)
{
    uint32_t rec = 0;
    if (id > KEY_ID_MAX)
    {
        LOG_E("id out of range!");
        return;
    }
    LOG_D("rc key add");
    rt_event_recv(rc_evt, EVENT_SCAN_END, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    rc_id[id] = tmp_id;
}

static void rc_del_key(uint16_t id)
{
    rc_id[id] = 0;
}

static uint16_t rc_ver_key(void)
{
    uint32_t rec = 0;
    uint16_t i = 0;
    LOG_D("rc key ver");
    rt_event_recv(rc_evt, EVENT_SCAN_END, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    for (i = 0; i < KEY_ID_MAX; i++)
    {
        if (rc_id[i] == tmp_id)
            return i;
    }
    return KEY_VER_ERROR;
}

static uint8_t rc_has_key(uint16_t id)
{
    return (rc_id[id] != 0) ? 1 : 0;
}

static void rc_init_thread(void *param)
{
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
    PCD_Init();		// Init MFRC522
    rt_thread_mdelay(4);
    uid = get_uid();

    while (1)
    {
        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
        if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial())
        {
            rt_thread_mdelay(50);
            continue;
        }

        // Dump debug info about the card; PICC_HaltA() is automatically called
        tmp_id = uid->uidByte[0] << 24 | uid->uidByte[1] << 16 | uid->uidByte[2] << 8 | uid->uidByte[3]; // change uid format form 8bit to 32bit
        rt_thread_mdelay(500);
        rt_event_send(get_key_det_evt(), EVT_KEY_DET_RF);
        rt_event_send(rc_evt, EVENT_SCAN_END);
        // break;
    }

    // PCD_End();
}

static int rt_hw_rc_port(void)
{
    en_key_rc.add_key = rc_add_key;
    en_key_rc.del_key = rc_del_key;
    en_key_rc.ver_key = rc_ver_key;
    en_key_rc.has_key = rc_has_key;
    reg_key_obj(ENTRY_KEY_RF, &en_key_rc);

    rc_evt = rt_event_create("rck-evt", RT_IPC_FLAG_FIFO);
    if (RT_NULL == rc_evt)
        LOG_E("create rc_evt failed!");

    rt_thread_t rc_tid = RT_NULL;
    rc_tid = rt_thread_create("rck-tid", rc_init_thread, RT_NULL, 1024, 15, 10);
    if (rc_tid != RT_NULL)
        rt_thread_startup(rc_tid);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_rc_port);

