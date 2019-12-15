#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <dev_sign_api.h>
#include <mqtt_api.h>
#include <infra_compat.h>

#include "rtthread.h"
#include "dht11.h"
#include "drv_gpio.h"

#define PRODUCT_KEY             PKG_USING_ALI_IOTKIT_PRODUCT_KEY
#define DEVICE_NAME             PKG_USING_ALI_IOTKIT_DEVICE_NAME
#define DEVICE_SECRET           PKG_USING_ALI_IOTKIT_DEVICE_SECRET

/* ALINK TSL Device attribute report */
#define ALINK_PROPERTY_POST_PUB          "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/event/property/post"        
#define ALINK_PROPERTY_POST_REPLY_SUB    "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/event/property/post_reply"
#define ALINK_PROPERTY_SET_REPLY_SUB     "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/event/property/set_reply"
#define ALINK_SERVICE_SET_SUB            "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/service/property/set"
#define ALINK_SERVICE_REMOTE_OPEN        "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/service/RemoteOpen"
#define ALINK_SERVICE_GETKEY_LIST        "/sys/"PRODUCT_KEY"/"DEVICE_NAME"/thing/service/GetKeyList"

/* These are pre-defined topics format*/
#define TOPIC_UPDATE_FMT                 "/%s/%s/update"
#define TOPIC_ERROR_FMT                  "/%s/%s/update/error"
#define TOPIC_GET_FMT                    "/%s/%s/get"
#define TOPIC_DATA_FMT                   "/%s/%s/data"

#define MQTT_MSGLEN                      (1024)

#define EXAMPLE_TRACE(fmt, ...)  \
    do { \
        HAL_Printf("%s|%03d :: ", __func__, __LINE__); \
        HAL_Printf(fmt, ##__VA_ARGS__); \
        HAL_Printf("%s", "\r\n"); \
    } while(0)

void *HAL_Malloc(uint32_t size);
void HAL_Free(void *ptr);
void HAL_Printf(const char *fmt, ...);
extern int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1]);
extern int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1]);
extern int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN]);
extern void HAL_SleepMs(uint32_t ms);
uint64_t HAL_UptimeMs(void);
int HAL_Snprintf(char *str, const int len, const char *fmt, ...);
static int ali_mqtt_pub(void);

static char __product_key[IOTX_PRODUCT_KEY_LEN + 1] = {0};
static char __device_name[IOTX_DEVICE_NAME_LEN + 1] = {0};
static char __device_secret[IOTX_DEVICE_SECRET_LEN + 1] = {0};

static int     user_argc;
static char   *user_param = NULL;

static void   *pclient;

static uint8_t is_running = 0;

/* some status in this project */
static rt_event_t event_control, event_sensor;
static rt_timer_t timer_temp_humi;
/* control event define */
#define EVENT_LOCK_STAT     (1 << 0)
/* sensor event define */
#define EVENT_TEMP_HUMI     (1 << 0)
static uint8_t lock_status = 0; /* 0: close; 1: open */

static uint8_t temp = 0;
static uint8_t humi = 0;

#define ALI_LOCK_PIN    GET_PIN(C, 13)

static void get_temp_humi_handle(void *parm)
{
    rt_event_send(event_sensor, EVENT_TEMP_HUMI);
}

static char* rt_strlwr(char *str)
 {
    if(str == NULL)
        return NULL;
         
    char *p = str;
    while (*p != '\0')
    {
        if(*p >= 'A' && *p <= 'Z')
            *p = (*p) + 0x20;
        p++;
    }
    return str;
}

static void event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt topic_info = (iotx_mqtt_topic_info_pt)msg->msg;
    if (topic_info == NULL)
    {
        rt_kprintf("Topic info is null! Exit.");
        return;
    }
    uintptr_t packet_id = (uintptr_t)topic_info->packet_id;

    switch (msg->event_type) {
        case IOTX_MQTT_EVENT_UNDEF:
            EXAMPLE_TRACE("undefined event occur.");
            break;

        case IOTX_MQTT_EVENT_DISCONNECT:
            EXAMPLE_TRACE("MQTT disconnect.");
            break;

        case IOTX_MQTT_EVENT_RECONNECT:
            EXAMPLE_TRACE("MQTT reconnect.");
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_SUCCESS:
            EXAMPLE_TRACE("subscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_TIMEOUT:
            EXAMPLE_TRACE("subscribe wait ack timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_NACK:
            EXAMPLE_TRACE("subscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_SUCCESS:
            EXAMPLE_TRACE("unsubscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_TIMEOUT:
            EXAMPLE_TRACE("unsubscribe timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_NACK:
            EXAMPLE_TRACE("unsubscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_SUCCESS:
            EXAMPLE_TRACE("publish success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_TIMEOUT:
            EXAMPLE_TRACE("publish timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_NACK:
            EXAMPLE_TRACE("publish nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
            EXAMPLE_TRACE("topic message arrived but without any related handle: topic=%.*s, topic_msg=%.*s",
                          topic_info->topic_len,
                          topic_info->ptopic,
                          topic_info->payload_len,
                          topic_info->payload);
            break;

        case IOTX_MQTT_EVENT_BUFFER_OVERFLOW:
            EXAMPLE_TRACE("buffer overflow, %s", msg->msg);
            break;

        default:
            EXAMPLE_TRACE("Should NOT arrive here.");
            break;
    }
}

static void print_topic_info(iotx_mqtt_topic_info_pt ptopic_info)
{
    /* print topic name and topic message */
    EXAMPLE_TRACE("----");
    EXAMPLE_TRACE("packetId: %d", ptopic_info->packet_id);
    EXAMPLE_TRACE("Topic: '%.*s' (Length: %d)",
                  ptopic_info->topic_len,
                  ptopic_info->ptopic,
                  ptopic_info->topic_len);
    EXAMPLE_TRACE("Payload: '%.*s' (Length: %d)",
                  ptopic_info->payload_len,
                  ptopic_info->payload,
                  ptopic_info->payload_len);
    EXAMPLE_TRACE("----");
}

static void _demo_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    print_topic_info(ptopic_info);
}

static void remote_open_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;
    rt_err_t send_ok;

    print_topic_info(ptopic_info);

    rt_kprintf("remote open!\r\n");
    lock_status = 1;
    rt_pin_write(ALI_LOCK_PIN, 0);
    send_ok = rt_event_send(event_control, EVENT_LOCK_STAT);
    if (RT_EOK != send_ok)
        rt_kprintf("send error\r\n");
}

static void getkey_list_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    print_topic_info(ptopic_info);

    rt_kprintf("get key list.\r\n");
}

/**
 * @brief update the list of status:
 *        lock_status
 * 
 * @param param 
 */
static void update_status(void)
{
    rt_uint32_t recved = 0;

    if (rt_event_recv(event_control, EVENT_LOCK_STAT, 
        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, 
        &recved) == RT_EOK)
    {
        ali_mqtt_pub();
        switch (recved)
        {
        case EVENT_LOCK_STAT:
            rt_thread_mdelay(5000);
            lock_status = 0;
            rt_pin_write(ALI_LOCK_PIN, 1);
            break;

        default:
            break;
        }
    }

    if (rt_event_recv(event_sensor, EVENT_TEMP_HUMI, 
        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, 
        &recved) == RT_EOK)
    {
        switch (recved)
        {
        case EVENT_TEMP_HUMI:
            dht11_get_temp_humi(&temp, &humi);

        default:
            break;
        }
    }

    if (recved != 0)
        ali_mqtt_pub();
}

/**
 * @brief mqtt client
 * 
 */
static void mqtt_client(void)
{
    int rc = 0;
    // rt_uint32_t recved = 0;
    rt_err_t error = 0;

    iotx_conn_info_pt pconn_info;
    iotx_mqtt_param_t mqtt_params;

    char *msg_buf = NULL, *msg_readbuf = NULL;

    IOT_OpenLog("mqtt");
    IOT_SetLogLevel(IOT_LOG_DEBUG);

    if (NULL == (msg_buf = (char *)HAL_Malloc(MQTT_MSGLEN))) {
        EXAMPLE_TRACE("not enough memory");
        rc = -1;
        goto do_exit;
    }

    if (NULL == (msg_readbuf = (char *)HAL_Malloc(MQTT_MSGLEN))) {
        EXAMPLE_TRACE("not enough memory");
        rc = -1;
        goto do_exit;
    }

    HAL_GetProductKey(__product_key);
    HAL_GetDeviceName(__device_name);
    HAL_GetDeviceSecret(__device_secret);

    /* Device AUTH */
    if (0 != IOT_SetupConnInfo(__product_key, __device_name, __device_secret, (void **)&pconn_info)) {
        EXAMPLE_TRACE("AUTH request failed!");
        rc = -1;
        goto do_exit;
    }

    /* Initialize MQTT parameter */
    memset(&mqtt_params, 0x0, sizeof(mqtt_params));

    mqtt_params.port = pconn_info->port;
    mqtt_params.host = pconn_info->host_name;
    mqtt_params.client_id = pconn_info->client_id;
    mqtt_params.username = pconn_info->username;
    mqtt_params.password = pconn_info->password;
    mqtt_params.pub_key = pconn_info->pub_key;

    mqtt_params.request_timeout_ms = 2000;
    mqtt_params.clean_session = 0;
    mqtt_params.keepalive_interval_ms = 60000;
    // mqtt_params.pread_buf = msg_readbuf;
    mqtt_params.read_buf_size = MQTT_MSGLEN;
    // mqtt_params.pwrite_buf = msg_buf;
    mqtt_params.write_buf_size = MQTT_MSGLEN;

    mqtt_params.handle_event.h_fp = event_handle;
    mqtt_params.handle_event.pcontext = NULL;

    /* Convert uppercase letters in host to lowercase */
	rt_kprintf("host: %s\r\n", rt_strlwr((char*)mqtt_params.host));

    /* Construct a MQTT client with specify parameter */
    pclient = IOT_MQTT_Construct(&mqtt_params);
    if (NULL == pclient) {
        EXAMPLE_TRACE("MQTT construct failed");
        rc = -1;
        goto do_exit;
    }

    /* Subscribe the specific topic */
    rc = IOT_MQTT_Subscribe(pclient, ALINK_SERVICE_SET_SUB, IOTX_MQTT_QOS1, _demo_message_arrive, NULL);
    if (rc < 0) {
        IOT_MQTT_Destroy(&pclient);
        EXAMPLE_TRACE("IOT_MQTT_Subscribe() failed, rc = %d", rc);
        rc = -1;
        goto do_exit;
    }

    /* Subscribe the specific topic */
    rc = IOT_MQTT_Subscribe(pclient, ALINK_PROPERTY_POST_REPLY_SUB, IOTX_MQTT_QOS1, _demo_message_arrive, NULL);
    if (rc < 0) {
        goto do_exit;
    }

    rc = IOT_MQTT_Subscribe(pclient, ALINK_SERVICE_REMOTE_OPEN, IOTX_MQTT_QOS1, remote_open_handle, NULL);
    if (rc < 0) {
        goto do_exit;
    }

    rc = IOT_MQTT_Subscribe(pclient, ALINK_SERVICE_GETKEY_LIST, IOTX_MQTT_QOS1, getkey_list_handle, NULL);
    if (rc < 0) {
        goto do_exit;
    }

    IOT_MQTT_Yield(pclient, 200);

    error = rt_timer_start(timer_temp_humi);
    if (RT_EOK != error)
    {
        rt_kprintf("timer start error\r\n");
    }

    do {
        /* handle the MQTT packet received from TCP or SSL connection */
        IOT_MQTT_Yield(pclient, 200);

        /* upload the status of device */
        update_status();

        rt_kprintf("temp=%d\°C,humi=%d\%\r\n", temp, humi);

        HAL_SleepMs(2000);

    } while (is_running);

    IOT_MQTT_Yield(pclient, 200);

    IOT_MQTT_Unsubscribe(pclient, ALINK_PROPERTY_POST_REPLY_SUB);
    IOT_MQTT_Unsubscribe(pclient, ALINK_SERVICE_SET_SUB);
    IOT_MQTT_Unsubscribe(pclient, ALINK_SERVICE_REMOTE_OPEN);
    IOT_MQTT_Unsubscribe(pclient, ALINK_SERVICE_GETKEY_LIST);

    IOT_MQTT_Yield(pclient, 200);

    IOT_MQTT_Destroy(&pclient);

do_exit:
    if (rc < 0) {
        IOT_MQTT_Destroy(&pclient);
        EXAMPLE_TRACE("IOT_MQTT_Subscribe() failed, rc = %d", rc);
    }

    if (NULL != msg_buf) {
        HAL_Free(msg_buf);
    }

    if (NULL != msg_readbuf) {
        HAL_Free(msg_readbuf);
    }

    if (NULL != user_param)
        HAL_Free(user_param);
    user_param = NULL;

    IOT_DumpMemoryStats(IOT_LOG_DEBUG);
    IOT_CloseLog();

    is_running = 0;

    EXAMPLE_TRACE("out of sample!");
}

static int ali_mqtt_pub(void)
{
    int rc = 0;
    static uint16_t pub_msg_cnt = 0;

    char   msg_pub[512];
    iotx_mqtt_topic_info_t topic_msg;

    if (!is_running)
    {
        HAL_Printf("MQTT test is not running! Please start MQTT first by using the \"ali_mqtt_test start\" command");
        return 0;
    }

    /* Initialize topic information */
    memset(msg_pub, 0x0, sizeof(msg_pub));

    snprintf(msg_pub, sizeof(msg_pub), 
            "{\"id\" : \"%d\",\"version\":\"1.0\",\"params\" : "
            "{\"LockState\" : %d,"
            "\"CurrentHumidity\":\"%d\%\%\"," /* double \% just show one */
            "\"CurrentTemperature\":\"%d\℃\"},"
            "\"method\":\"thing.event.property.post\"}",
            (++pub_msg_cnt)%10000, lock_status, humi, temp);

    memset(&topic_msg, 0x0, sizeof(iotx_mqtt_topic_info_t));
    topic_msg.qos = IOTX_MQTT_QOS1;
    topic_msg.retain = 0;
    topic_msg.dup = 0;
    topic_msg.payload = (void *)msg_pub;
    topic_msg.payload_len = strlen(msg_pub);

    rc = IOT_MQTT_Publish(pclient, ALINK_PROPERTY_POST_PUB, &topic_msg);
    if (rc < 0) {
        IOT_MQTT_Destroy(&pclient);
        EXAMPLE_TRACE("error occur when publish");
        rc = -1;
        return rc;
    }

    EXAMPLE_TRACE("\n publish message: \n topic: %s\n payload: %s\n rc = %d", ALINK_PROPERTY_POST_PUB, topic_msg.payload, rc);

    return 0;
}

static int ali_mqtt_main(int argc, char **argv)
{
    rt_thread_t tid1;

    user_argc = argc;
    if (2 == user_argc)
    {
        if (!strcmp("start", argv[1]))
        {
            if (1 == is_running)
            {
                HAL_Printf("MQTT test is already running! Please stop running first by using the \"ali_mqtt_test stop\" command\n");
                return 0;
            }
            is_running = 1;
        }
        else if (!strcmp("stop", argv[1]))
        {
            if (0 == is_running)
            {
                HAL_Printf("MQTT test is already stopped!\n");
                return 0;
            }
            is_running = 0;
            // stop mqtt test
            return 0;
        }
        else
        {
            HAL_Printf("Input param error! Example: ali_mqtt_test start/stop or ali_mqtt_test pub open/close\n");
            return 0;            
        }
    }
    else if(3 == user_argc)
    {
        if (!strcmp("pub", argv[1]))
        {
            user_param = (char*)rt_strdup((const char*)argv[2]);
            HAL_Printf("param:%s\n", user_param);

            // publish
            ali_mqtt_pub();
            return 0;
        }
        else
        {
            HAL_Printf("Input param error! Example: ali_mqtt_test start/stop or ali_mqtt_test pub open/close\n");
            return 0;
        }
    }
    else
    {
        HAL_Printf("Input param error! Example: ali_mqtt_test start/stop or ali_mqtt_test pub open/close\n");
        return 0;
    }

#ifdef IOTX_PRJ_VERSION
    HAL_Printf("iotkit-embedded sdk version: %s\n", IOTX_PRJ_VERSION);
#endif

    HAL_SetProductKey(PRODUCT_KEY);
    HAL_SetDeviceName(DEVICE_NAME);
    HAL_SetDeviceSecret(DEVICE_SECRET);

    rt_pin_mode(ALI_LOCK_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ALI_LOCK_PIN, 1);

    event_control = rt_event_create("evnt-ctl", RT_IPC_FLAG_FIFO);
    if (event_control == RT_NULL)
    {
        rt_kprintf("event create error!\r\n");
        return 1;
    }

    event_sensor = rt_event_create("evnt-sor", RT_IPC_FLAG_FIFO);
    if (event_sensor == RT_NULL)
    {
        rt_kprintf("event sensor error!\r\n");
        return 1;
    }

    timer_temp_humi = rt_timer_create("tim-th", get_temp_humi_handle, NULL, (rt_tick_t)10*RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);

    tid1 = rt_thread_create("ali-mqtt",
                    (void (*)(void *))mqtt_client, NULL,
                    6 * 1024, RT_THREAD_PRIORITY_MAX / 2 - 1, 10);
    if (tid1 != RT_NULL)
            rt_thread_startup(tid1);

    return 0;
}
#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(ali_mqtt_main, ali_mqtt_test, Example: ali_mqtt_test start/pub [open/close]/stop);
#endif
