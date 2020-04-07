#ifndef ENTRY_KEY_H_
#define ENTRY_KEY_H_

#include <rtthread.h>
#include <rtdevice.h>

/** key configure */
#define GRD_USER_MAX    8U

#define CFG_KEY_USING_PW    1
#define CFG_KEY_USING_FP    1
#define CFG_KEY_USING_RF    1
#define CFG_KEY_USING_FD    0
#define CFG_KEY_USING_NUM   (CFG_KEY_USING_PW + CFG_KEY_USING_FP + CFG_KEY_USING_RF + CFG_KEY_USING_FD)
#define CFG_KEY_TOTAL_NUM   4

//#define KEY_TYPES_ON_SIZE   2
#define NAME_BUF_LEN 4
#define USER_KEY_LEN 16
#define KEY_TYPES_MAX 10U
#define KEY_ID_MAX 8U
#define KEY_VER_WAIT_TIME 20
#define KEY_VER_ERROR 255U

//without '/0'
#define KEY_TYPE_LEN    6U
#define NAME_STR_LEN    3U
#define PW_KEY_LEN      6U
#define FP_KEY_LEN      6U
#define RF_KEY_LEN      6U
#define FD_KEY_LEN      6U

#define ROOT_USER_LEN       ((NAME_STR_LEN+1)*3+1) // format: 111,222,333,
#define USER_INFO_SIZE      (CFG_KEY_USING_NUM)*(USER_KEY_LEN+1) // 1是分割符的长度
#define KEY_IN_FLASH_LEN    (GRD_USER_MAX*(NAME_STR_LEN+PW_KEY_LEN+2)+1) // format: 111:111111,222:222222,
//static uint8_t key_types = 0; // 暂时想不出好的处理方式，先预先设置吧

#define EVT_GRD_DET_PW (1U << 0)
#define EVT_GRD_DET_FP (1U << 1)
#define EVT_GRD_DET_RF (1U << 2)
#define EVT_GRD_DET_FD (1U << 3)

#define EVT_KEY_DET_PW (1U << 4)
#define EVT_KEY_DET_FP (1U << 5)
#define EVT_KEY_DET_RF (1U << 6)
#define EVT_KEY_DET_FD (1U << 7)

#define USR_CHECK_AUTH_OK   0
#define USR_CHECK_AUTH_ERR  1

#define LOCK_PIN_NUM    GET_PIN(B, 14)
/** key configure end */

enum
{
    USR_OP_ERROR=0,
    USR_OP_ADD_NUSR,
    USR_OP_ADD_RUSR,
    USR_OP_DEL_NUSR,
};

enum wkng_mode_type
{
    WKNG_MODE_ROOT=0,
    WKNG_MODE_NORM,
    WKNG_MODE_IDLE,
};

enum entry_key_type
{
    ENTRY_KEY_PW, // password
    ENTRY_KEY_FP, // fingerpint
    ENTRY_KEY_RF, // rfid
    ENTRY_KEY_FD, // facedetect
};

struct user_info {
    char name[NAME_BUF_LEN];
    char key_pw[USER_KEY_LEN];
    char key_fp[USER_KEY_LEN];
    char key_rc[USER_KEY_LEN];
    char key_fd[USER_KEY_LEN];
};
typedef struct user_info *user_info_t;

struct entry_key{
    void (*add_key)(user_info_t info);  ///> add a new key
    void (*del_key)(user_info_t info);  ///> delete a key
    uint8_t (*ver_key)(char *name);  ///> verify the key, return the result and the user name
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

void fills_to_len_with_zero(char *key, uint8_t len);
uint8_t if_key_in_flash(const char *key_type, char *name, const char *key);
void str_key_to_flash(const char *key_type, const char *name, const char *key);
void del_key_in_flash(const char *key_type, const char *name, const char *key);

/**
 * @brief 这个接口暂时找不到合并的办法，先单独实现
 * @param name
 * @param key
 * @return
 */
uint8_t pw_ver_key(char *name, char *key, uint8_t len);

#endif
