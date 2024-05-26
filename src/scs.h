#ifndef BT_SCS_H_
#define BT_SCS_H_

/**
 * @file
 * @defgroup bt_scs Nordic UART (SCS) GATT Service
 * @{
 * @brief Nordic UART (SCS) GATT Service API.
 */

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UUID of the SCS Service. **/
#define BT_UUID_SCS_VAL \
	BT_UUID_128_ENCODE(0x8000ff00, 0xff00, 0xffff, 0xffff, 0xffffffffffff)

/** @brief UUID of the camera command characteristic. **/
#define BT_UUID_SCS_CAMERA_CMD_VAL (0xff01)

/** @brief UUID of the camera staus characteristic. **/
#define BT_UUID_SCS_CAMERA_STATUS_VAL (0xff02)

#define BT_UUID_SCS_SERVICE         BT_UUID_DECLARE_128(BT_UUID_SCS_VAL)
#define BT_UUID_SCS_CAMERA_CMD      BT_UUID_DECLARE_16(BT_UUID_SCS_CAMERA_CMD_VAL)
#define BT_UUID_SCS_CAMERA_STATUS   BT_UUID_DECLARE_16(BT_UUID_SCS_CAMERA_STATUS_VAL)

/** @brief SCS send status. */
enum bt_scs_send_status {
	/** Send notification enabled. */
	BT_SCS_SEND_STATUS_ENABLED,
	/** Send notification disabled. */
	BT_SCS_SEND_STATUS_DISABLED,
};


/**@brief Get maximum data length that can be used for @ref bt_scs_send.
 *
 * @param[in] conn Pointer to connection Object.
 *
 * @return Maximum data length.
 */
static inline uint32_t bt_scs_get_mtu(struct bt_conn *conn)
{
	/* According to 3.4.7.1 Handle Value Notification off the ATT protocol.
	 * Maximum supported notification is ATT_MTU - 3 */
	return bt_gatt_get_mtu(conn) - 3;
}

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_SCS_H_ */