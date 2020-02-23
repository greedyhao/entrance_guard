#ifndef ENTRY_KEY_H_
#define ENTRY_KEY_H_

#include <rtthread.h>
#include <rtdevice.h>

#define KEY_TYPES_MAX 10U
#define KEY_ID_MAX 8U
#define KEY_VER_WAIT_TIME 20
#define KEY_VER_ERROR 255U

#define EVT_KEY_DET_PW (1U << 0)
#define EVT_KEY_DET_FP (1U << 1)
#define EVT_KEY_DET_RF (1U << 2)
#define EVT_KEY_DET_FD (1U << 3)

enum entry_key_type
{
    ENTRY_KEY_PW, // password
    ENTRY_KEY_FP, // fingerpint
    ENTRY_KEY_RF, // rfid
    ENTRY_KEY_FD, // facedetect
};

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
