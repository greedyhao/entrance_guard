#ifndef ENTRY_KEY_H_
#define ENTRY_KEY_H_

#include <rtthread.h>
#include <rtdevice.h>

#define KEY_TYPES_MAX 10U
#define KEY_ID_MAX 8U

enum entry_key_type
{
    ENTRY_KEY_PW, // password
    ENTRY_KEY_FP, // fingerpint
    ENTRY_KEY_RF, // rfid
    ENTRY_KEY_FD, // facedetect
};

struct entry_key{
    uint8_t enable;    // numbers of key
    // rt_slist_t key_list;// list of keys
    void (*add_key)();  // add a new key
    void (*del_key)();  // delet a key
    void (*ver_key)();  // verify the key
};
typedef struct entry_key *entry_key_t;


/**
 * @brief Regist the key object
 * 
 * @param key_type 
 * @param key_obj 
 */
void reg_key_obj(enum entry_key_type key_type, entry_key_t key_obj);

#endif
