/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "scs.h"
#include "scs_client.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(scs_c, LOG_LEVEL_DBG);

enum {
	SCS_C_INITIALIZED,
	SCS_C_TX_NOTIF_ENABLED,
	SCS_C_RX_WRITE_PENDING
};

static uint8_t on_received(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length)
{
	struct bt_scs_client *scs;

	/* Retrieve SCS Client module context. */
	scs = CONTAINER_OF(params, struct bt_scs_client, camera_status_notif_params);

	if (!data) {
		LOG_DBG("[UNSUBSCRIBED]");
		params->value_handle = 0;
		atomic_clear_bit(&scs->state, SCS_C_TX_NOTIF_ENABLED);
		if (scs->cb.unsubscribed) {
			scs->cb.unsubscribed(scs);
		}
		return BT_GATT_ITER_STOP;
	}

	LOG_DBG("[NOTIFICATION] data %p length %u", data, length);
	if (scs->cb.received) {
		return scs->cb.received(scs, data, length);
	}

	return BT_GATT_ITER_CONTINUE;
}

static void on_sent(struct bt_conn *conn, uint8_t err,
		    struct bt_gatt_write_params *params)
{
	struct bt_scs_client *scs_c;
	const void *data;
	uint16_t length;

	/* Retrieve SCS Client module context. */
	scs_c = CONTAINER_OF(params, struct bt_scs_client, camera_cmd_write_params);

	/* Make a copy of volatile data that is required by the callback. */
	data = params->data;
	length = params->length;

	atomic_clear_bit(&scs_c->state, SCS_C_RX_WRITE_PENDING);
	if (scs_c->cb.sent) {
		scs_c->cb.sent(scs_c, err, data, length);
	}
}

int bt_scs_client_init(struct bt_scs_client *scs_c,
		       const struct bt_scs_client_init_param *scs_c_init)
{
	if (!scs_c || !scs_c_init) {
		return -EINVAL;
	}

	if (atomic_test_and_set_bit(&scs_c->state, SCS_C_INITIALIZED)) {
		return -EALREADY;
	}

	memcpy(&scs_c->cb, &scs_c_init->cb, sizeof(scs_c->cb));

	return 0;
}

int bt_scs_client_send_camera_cmd(struct bt_scs_client *scs_c, const uint8_t *data,
		       uint16_t len)
{
	int err;

	if (!scs_c->conn) {
		return -ENOTCONN;
	}

	if (atomic_test_and_set_bit(&scs_c->state, SCS_C_RX_WRITE_PENDING)) {
		return -EALREADY;
	}

	scs_c->camera_cmd_write_params.func = on_sent;
	scs_c->camera_cmd_write_params.handle = scs_c->handles.camera_cmd;
	scs_c->camera_cmd_write_params.offset = 0;
	scs_c->camera_cmd_write_params.data = data;
	scs_c->camera_cmd_write_params.length = len;

	err = bt_gatt_write(scs_c->conn, &scs_c->camera_cmd_write_params);
	if (err) {
		atomic_clear_bit(&scs_c->state, SCS_C_RX_WRITE_PENDING);
	}

	return err;
}

int bt_scs_handles_assign(struct bt_gatt_dm *dm,
			  struct bt_scs_client *scs_c)
{
	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_SCS_SERVICE)) {
		return -ENOTSUP;
	}
	LOG_DBG("Getting handles from SCS service.");
	memset(&scs_c->handles, 0xFF, sizeof(scs_c->handles));

	/* SCS Camera Command Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_SCS_CAMERA_CMD);
	if (!gatt_chrc) {
		LOG_ERR("Missing SCS Camera Command characteristic.");
		return -EINVAL;
	}
	/* SCS Camera Command Characteristic Descriptor  */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_SCS_CAMERA_CMD);
	if (!gatt_desc) {
		LOG_ERR("Missing SCSCamera Command value descriptor in characteristic.");
		return -EINVAL;
	}
	LOG_DBG("Found handle for SCS Camera Command characteristic.");
	scs_c->handles.camera_cmd = gatt_desc->handle;
	
	/* SCS Camera Status Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_SCS_CAMERA_STATUS);
	if (!gatt_chrc) {
		LOG_ERR("Missing SCS Camera Status characteristic.");
		return -EINVAL;
	}
	/* SCS Camera Status Characteristic Descriptor */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_SCS_CAMERA_STATUS);
	if (!gatt_desc) {
		LOG_ERR("Missing SCS Camera Status value descriptor in characteristic.");
		return -EINVAL;
	}
	LOG_DBG("Found handle for SCS Camera Status characteristic.");
	scs_c->handles.camera_status = gatt_desc->handle;

    /* SCS Camera Status CCC */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		LOG_ERR("Missing SCS Camera Status CCC in characteristic.");
		return -EINVAL;
	}
	LOG_DBG("Found handle for CCC of SCS Camera Status characteristic.");
	scs_c->handles.camera_status_ccc = gatt_desc->handle;

	/* Assign connection instance. */
	scs_c->conn = bt_gatt_dm_conn_get(dm);
	return 0;
}

int bt_scs_subscribe_camera_status(struct bt_scs_client *scs_c)
{
	int err;

	if (atomic_test_and_set_bit(&scs_c->state, SCS_C_TX_NOTIF_ENABLED)) {
		return -EALREADY;
	}

	scs_c->camera_status_notif_params.notify = on_received;
	scs_c->camera_status_notif_params.value = BT_GATT_CCC_NOTIFY;
	scs_c->camera_status_notif_params.value_handle = scs_c->handles.camera_status;
	scs_c->camera_status_notif_params.ccc_handle = scs_c->handles.camera_status_ccc;
	atomic_set_bit(scs_c->camera_status_notif_params.flags,
		       BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(scs_c->conn, &scs_c->camera_status_notif_params);
	if (err) {
		LOG_ERR("Subscribe failed (err %d)", err);
		atomic_clear_bit(&scs_c->state, SCS_C_TX_NOTIF_ENABLED);
	} else {
		LOG_DBG("[SUBSCRIBED]");
	}

	return err;
}

void bt_scs_trigger_capture(struct bt_scs_client *scs_c)
{
	uint8_t data[2];
	data[0] = 0x01;
	data[1] = 0x09; // 0x7
	bt_scs_client_send_camera_cmd(scs_c, data,2);
}