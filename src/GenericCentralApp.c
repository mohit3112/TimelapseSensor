#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <zephyr/logging/log.h>
#include <bluetooth/gatt_dm.h>
#include "scs_client.h"
#include "GenericCentralApp.h"

#define ADV_NAME_STR_MAX_LEN 25
#define ADV_MANUFACTURE_STRING 30

void scan_filter_match(struct bt_scan_device_info *device_info,
					   struct bt_scan_filter_match *filter_match,
					   bool connectable);
void scan_filter_no_match(struct bt_scan_device_info *device_info,
						  bool connectable);
void scan_connecting_error(struct bt_scan_device_info *device_info);
void scan_connecting(struct bt_scan_device_info *device_info,
					 struct bt_conn *conn);

LOG_MODULE_REGISTER(GenenicCentralApp, LOG_LEVEL_DBG);

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match, scan_connecting_error, scan_connecting);

static struct bt_conn *default_conn;
static scan_context_t default_scan_context;
static conn_context_t default_conn_context;

typedef struct
{
	char name[ADV_NAME_STR_MAX_LEN];
	uint8_t manufacturer_data[ADV_MANUFACTURE_STRING];
	uint8_t manufacturer_data_len;
} adv_data_t;

static bool adv_data_parse_cb(struct bt_data *data, void *user_data)
{
	adv_data_t *adv_data = user_data;

	uint8_t len;

	switch (data->type)
	{
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, ADV_NAME_STR_MAX_LEN - 1);
		memcpy(adv_data->name, data->data, len);
		adv_data->name[len] = '\0';
		return false;
	case BT_DATA_MANUFACTURER_DATA:
		adv_data->manufacturer_data_len = data->data_len;
		memcpy(adv_data->manufacturer_data, data->data, data->data_len);
		return false;
	default:
		return true;
	}
}

void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	if (filter_match->name.len == 0)
	{
		printk("Filters matched. Address: %s connectable: %d\n",
			   addr, connectable);
	}
	else
	{
		printk("Name %s Address: %s connectable: %d\n",
			   filter_match->name.name, addr, connectable);
	}
}

void scan_filter_no_match(struct bt_scan_device_info *device_info, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];
	adv_data_t data;
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	bt_data_parse(device_info->adv_data, adv_data_parse_cb, &data);
	if (bt_addr_cmp(&(device_info->recv_info->addr->a), &default_scan_context.pheriphral_addr) == 0)
	{
		if (memcmp(data.manufacturer_data, default_scan_context.pheriphral_manufacture_data_normal_mode, sizeof(default_scan_context.pheriphral_manufacture_data_len)) == 0 ||
			memcmp(data.manufacturer_data, default_scan_context.pheriphral_manufacture_data_normal_mode, sizeof(default_scan_context.pheriphral_manufacture_data_len)) == 0)
		{
			printk("Address: %s \n", addr);
			LOG_HEXDUMP_INF(data.manufacturer_data, data.manufacturer_data_len, "Manufacturer Data: ");
			bt_scan_stop();

			if(default_scan_context.pheriphral_found != NULL)
			{
				default_scan_context.pheriphral_found(device_info->recv_info->addr, device_info->conn_param);
			}
		
		}
	}
}
void scan_connecting_error(struct bt_scan_device_info *device_info)
{
}
void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
}

void scan_init(bool filter, const scan_context_t *scan_context)
{
	struct bt_scan_init_param param = {
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM_DEFAULT,
		.connect_if_match = 0};
	
	memcpy(&default_scan_context, scan_context, sizeof(scan_context_t));
	
	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	if (filter)
	{
		int err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
		if (err)
		{
			printk("Scanning filters cannot be set (err %d)\n", err);
		}

		err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
		if (err)
		{
			printk("Filters cannot be turned on (err %d)\n", err);
		}
	}
}

int scan_start(void)
{
	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);

	if (err)
	{
		printk("Scanning failed to start (err %d)\n", err);
	}

	return err;
}

void create_le_connnection(const conn_context_t *conn_context, const bt_addr_le_t *addr, const struct bt_le_conn_param *conn_param)
{
	int err;
	struct bt_conn *conn;
	
	memcpy(&default_conn_context, conn_context, sizeof(conn_context_t));

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, conn_param, &conn);
	if (!err)
	{
		default_conn = bt_conn_ref(conn);
		bt_conn_unref(conn);
	}
}
static void discovery_completed_cb(struct bt_gatt_dm *dm,
								   void *context)
{
	char uuid_str[37];

	const struct bt_gatt_dm_attr *gatt_service_attr =
		bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
		bt_gatt_dm_attr_service_val(gatt_service_attr);

	size_t attr_count = bt_gatt_dm_attr_cnt(dm);

	bt_uuid_to_str(gatt_service->uuid, uuid_str, sizeof(uuid_str));
	printk("Found service %s\n", uuid_str);
	printk("Attribute count: %d\n", attr_count);

	bt_gatt_dm_data_print(dm);

	if(strcmp(uuid_str, default_conn_context.srv_uuid_128) == 0)
	{
		if(context != NULL)
		{
			printk("Found SCS service");
			struct bt_scs_client *scs = context;
			bt_scs_handles_assign(dm,scs);
			bt_scs_subscribe_camera_status(scs);
		}
		else
		{
			printk("Context is NULL\n");
		}
	}

	bt_gatt_dm_data_release(dm);

	bt_gatt_dm_continue(dm, default_conn_context.scs_client);
}

static void discovery_service_not_found_cb(struct bt_conn *conn,
										   void *context)
{
	default_conn_context.device_ready();
	printk("The service could not be found during the discovery\n");
}

static void discovery_error_found_cb(struct bt_conn *conn,
									 int err,
									 void *context)
{
	printk("The discovery procedure failed with %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err)
	{
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		scan_start();
		return;
	}

	if (conn != default_conn)
	{
		return;
	}

	printk("Connected: %s\n", addr);

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err)
	{
		/* If we failed to enable security , start service discovery or else it will start on 
		   Security changed callback */
		printk("Failed to set security: %d\n", err);
		bt_gatt_dm_start(conn, NULL, &discovery_cb, default_conn_context.scs_client);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn)
	{
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	default_conn_context.device_inactive();
	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	scan_start();
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		printk("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		printk("Security failed: %s level %u err %d\n", addr, level,
			   err);
	}
	bt_gatt_dm_start(conn, NULL, &discovery_cb, default_conn_context.scs_client);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed};