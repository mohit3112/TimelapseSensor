#ifndef GENERIC_CENTRAL_APP_
#define GENERIC_CENTRAL_APP_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>
#include "scs_client.h"

typedef struct 
{
    bt_addr_t pheriphral_addr;
    const uint8_t *pheriphral_manufacture_data_normal_mode; 
    const uint8_t *pheriphral_manufacture_data_pairing_mode;
    uint8_t pheriphral_manufacture_data_len;
    void (*pheriphral_found)(const bt_addr_le_t *addr, const struct bt_le_conn_param *conn_param);
}scan_context_t;

typedef struct 
{
    char *srv_uuid_128;
    struct bt_scs_client *scs_client;
    void (*device_ready)(void);
    void (*device_inactive)(void);
}conn_context_t;

void scan_init(bool filter, const scan_context_t *scan_context);

int scan_start(void);

void create_le_connnection(const conn_context_t *conn_context, const bt_addr_le_t *addr, const struct bt_le_conn_param *conn_param);

#ifdef __cplusplus
}
#endif

#endif /* GENERIC_CENTRAL_APP_ */