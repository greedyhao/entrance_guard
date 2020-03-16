#ifndef ENTRY_KEY_H_
#define ENTRY_KEY_H_

#include <rtthread.h>
#include <rtdevice.h>

/** key configure */
#define CFG_KEY_USING_PW    0
#define CFG_KEY_USING_FP    1
#define CFG_KEY_USING_RF    1
#define CFG_KEY_USING_FD    0
//#define CFG_KEY_USING_NUM   (CFG_KEY_USING_PW + CFG_KEY_USING_FP + CFG_KEY_USING_RF + CFG_KEY_USING_FD)
#define CFG_KEY_USING_NUM   4

//#define CFG_USING_KEYBOARD

//#define KEY_TYPES_ON_SIZE   2
#define USER_NAME_LEN 4
#define USER_KEY_LEN 16
#define KEY_TYPES_MAX 10U
#define KEY_ID_MAX 8U
#define KEY_VER_WAIT_TIME 20
#define KEY_VER_ERROR 255U

#define USER_INFO_SIZE  (KEY_TYPES_ON_SIZE)*(KEY_INFO_LEN)
//static uint8_t key_types = 0; // 暂时想不出好的处理方式，先预先设置吧

#define EVT_KEY_DET_PW (1U << 0)
#define EVT_KEY_DET_FP (1U << 1)
#define EVT_KEY_DET_RF (1U << 2)
#define EVT_KEY_DET_FD (1U << 3)
/** key configure end */

enum entry_key_type
{
    ENTRY_KEY_PW, // password
    ENTRY_KEY_FP, // fingerpint
    ENTRY_KEY_RF, // rfid
    ENTRY_KEY_FD, // facedetect
};

struct user_info {
    char name[USER_NAME_LEN];
    char key_pw[USER_KEY_LEN];
    char key_fp[USER_KEY_LEN];
    char key_rc[USER_KEY_LEN];
    char key_fd[USER_KEY_LEN];
};
typedef struct user_info *user_info_t;

struct entry_key{
    // rt_slist_t key_list;// list of keys
    void (*add_key)(uint16_t id);  // add a new key
    void (*del_key)(uint16_t id);  // delet a key
    uint16_t (*ver_key)(void);  // verify the key
    uint8_t (*has_key)(uint16_t id); // check presence of the key
};
typedef struct entry_key *entry_key_t;

/**
 * @brief Get the key det evt object
 * 
 * @return rt_event_t 
 */
rt_event_t get_key_det_evt(void);

/**
 * @brief Regist the key object
 * 
 * @param key_type 
 * @param key_obj 
 */
void reg_key_obj(enum entry_key_type key_type, entry_key_t key_obj);

#endif
