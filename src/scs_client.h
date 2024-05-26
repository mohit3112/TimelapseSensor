#ifndef BT_SCS_CLIENT_H_
#define BT_SCS_CLIENT_H_

/**
 * @file
 * @defgroup bt_scs_client Bluetooth LE GATT SCS Client API
 * @{
 * @brief API for the Bluetooth LE GATT Sony Camera Service (SCS) Client.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>

/** @brief Handles on the connected peer device that are needed to interact with
 * the device.
 */
struct bt_scs_client_handles {

        /** Handle of the SCS camera cmd  characteristic, as provided by
	 *  a discovery.
         */
	uint16_t camera_cmd;

        /** Handle of the SCS camera staus  characteristic, as provided by
	 *  a discovery.
         */
	uint16_t camera_status;

        /** Handle of the CCC descriptor of the SCS Rx characteristic,
	 *  as provided by a discovery.
         */
	uint16_t camera_status_ccc;
};

struct bt_scs_client;

/** @brief SCS Client callback structure. */
struct bt_scs_client_cb {
	/** @brief Data received callback.
	 *
	 * The data has been received as a notification of the SCS camera staus 
	 * Characteristic.
	 *
	 * @param[in] scs  SCS Client instance.
	 * @param[in] data Received data.
	 * @param[in] len Length of received data.
	 *
	 * @retval BT_GATT_ITER_CONTINUE To keep notifications enabled.
	 * @retval BT_GATT_ITER_STOP To disable notifications.
	 */
	uint8_t (*received)(struct bt_scs_client *scs, const uint8_t *data, uint16_t len);

	/** @brief Data sent callback.
	 *
	 * The data has been sent and written to the SCS camera cmd  Characteristic.
	 *
	 * @param[in] scs  SCS Client instance.
	 * @param[in] err ATT error code.
	 * @param[in] data Transmitted data.
	 * @param[in] len Length of transmitted data.
	 */
	void (*sent)(struct bt_scs_client *scs, uint8_t err, const uint8_t *data, uint16_t len);

	/** @brief camera staus  notifications disabled callback.
	 *
	 * camera staus  notifications have been disabled.
	 *
	 * @param[in] scs  SCS Client instance.
	 */
	void (*unsubscribed)(struct bt_scs_client *scs);
};

/** @brief SCS Client structure. */
struct bt_scs_client {

        /** Connection object. */
	struct bt_conn *conn;

        /** Internal state. */
	atomic_t state;

        /** Handles on the connected peer device that are needed
         * to interact with the device.
         */
	struct bt_scs_client_handles handles;

        /** GATT subscribe parameters for SCS camera status Characteristic. */
	struct bt_gatt_subscribe_params camera_status_notif_params;

        /** GATT write parameters for SCS camera cmd Characteristic. */
	struct bt_gatt_write_params camera_cmd_write_params;

        /** Application callbacks. */
	struct bt_scs_client_cb cb;
};

/** @brief SCS Client initialization structure. */
struct bt_scs_client_init_param {

        /** Callbacks provided by the user. */
	struct bt_scs_client_cb cb;
};

/** @brief Initialize the SCS Client module.
 *
 * This function initializes the SCS Client module with callbacks provided by
 * the user.
 *
 * @param[in,out] scs    SCS Client instance.
 * @param[in] init_param SCS Client initialization parameters.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_scs_client_init(struct bt_scs_client *scs,
		       const struct bt_scs_client_init_param *init_param);

/** @brief Send data to the server.
 *
 * This function writes to the camera cmd  Characteristic of the server.
 *
 * @note This procedure is asynchronous. Therefore, the data to be sent must
 * remain valid while the function is active.
 *
 * @param[in,out] scs SCS Client instance.
 * @param[in] data Data to be transmitted.
 * @param[in] len Length of data.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_scs_client_send_camera_cmd(struct bt_scs_client *scs, const uint8_t *data,
		       uint16_t len);

/** @brief Assign handles to the SCS Client instance.
 *
 * This function should be called when a link with a peer has been established
 * to associate the link to this instance of the module. This makes it
 * possible to handle several links and associate each link to a particular
 * instance of this module. The GATT attribute handles are provided by the
 * GATT DB discovery module.
 *
 * @param[in] dm Discovery object.
 * @param[in,out] scs SCS Client instance.
 *
 * @retval 0 If the operation was successful.
 * @retval (-ENOTSUP) Special error code used when UUID
 *         of the service does not match the expected UUID.
 * @retval Otherwise, a negative error code is returned.
 */
int bt_scs_handles_assign(struct bt_gatt_dm *dm,
			  struct bt_scs_client *scs);

/** @brief Request the peer to start sending notifications for the camera staus 
 *	   Characteristic.
 *
 * This function enables notifications for the SCS camera staus  Characteristic at the peer
 * by writing to the CCC descriptor of the SCS camera staus  Characteristic.
 *
 * @param[in,out] scs SCS Client instance.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_scs_subscribe_camera_status(struct bt_scs_client *scs);

void bt_scs_trigger_capture(struct bt_scs_client *scs_c);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_SCS_CLIENT_H_ */