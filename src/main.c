#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log_instance.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/ring_buffer.h>
#include "GenericCentralApp.h"
#include "scs_client.h"
#include "iir_filter.h"
#include "max30102_driver_i2c.h"

#define I2C1_NODE DT_NODELABEL(max30102)
#define SENSOR_NODE DT_NODELABEL(max301021)
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NONE DT_ALIAS(led2)
#define GPIO_BATTERY_READ_ENABLE 14
#define FOCUS_TIMEOUT K_MSEC(5000)

LOG_MODULE_REGISTER(Main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED2_NONE, gpios);
const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct device *max30102_dev = DEVICE_DT_GET(SENSOR_NODE);
static const struct device *gpio_rdivider_en = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct k_work_delayable scs_timeout;
static struct k_work_delayable proximity_timeout;

static struct bt_scs_client scs_client;

const bt_addr_t pheriphral_addr = {
	.val = {0x3a, 0xe0, 0xdb, 0xef, 0x40, 0xd0}};

const uint8_t pheriphral_manufacture_data_normal_mode[] = {0x2d, 0x01, 0x03, 0x00, 0x64, 0x00, 0x45, 0x31, 0x22, 0xaf, 0x00, 0x21, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t pheriphral_manufacture_data_pairing_mode[] = {0x2d, 0x01, 0x03, 0x00, 0x64, 0x00, 0x45, 0x31, 0x22, 0xef, 0x00, 0x21, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define BT_UUID_LBS_VAL BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY), 800, 801, NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static bool rx_throttled;
static uint8_t cmd;
static void  camera_ready(void);
static void camera_inactive(void);

const conn_context_t conn_context = {
        .srv_uuid_128 = "8000ff00-ff00-ffff-ffff-ffffffffffff",
        .scs_client = &scs_client,
        .device_ready = camera_ready,
        .device_inactive = camera_inactive
};

enum {
        SENSOR_STATUS_PROXIMITY,
        SENSOR_STATUS_FAR,
};

enum{
        TRIGGER_STATUS_INACTIVE,
        TRIGGER_STATUS_ACTIVE,
        TRIGGER_STATUS_READY
};

uint8_t trigger_status; 
uint8_t sensor_status;

static iir_filter_data_t *filt;

void mainloop(int err);

static void camera_ready(void)
{
        trigger_status = TRIGGER_STATUS_READY;
        sensor_status = SENSOR_STATUS_FAR;
}

static void camera_inactive(void)
{
        trigger_status = TRIGGER_STATUS_INACTIVE;
}

static void camera_cmd_sent(struct bt_scs_client *scs, uint8_t err,
					const uint8_t *const data, uint16_t len)
{
        
	ARG_UNUSED(scs);
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	if (err) 
        {
		printk("ATT error code: 0x%02X\n", err);
	}
        else
        {
                LOG_HEXDUMP_INF(data,len,"cmd sent callback");
                printk("cmd send ok\n");
        }
}

static void scs_timeout_cb(struct k_work *item)
{
        uint8_t cmd[2];
        printk("Focus Timeout\n");
        cmd[0] = 0x01;
	cmd[1] = 0x08;
	bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        trigger_status = TRIGGER_STATUS_READY;
}
static uint8_t camera_status_received(struct bt_scs_client *scs,
						const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(scs);
        uint8_t status[]= {0x02,0x3f,0x20};
        uint8_t cmd[2];

        //TODO: Proper state machine to serialize the gatt trigger 
        if (memcmp(data,status, len) == 0 && trigger_status == TRIGGER_STATUS_ACTIVE)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x09;
                bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        }
        status[2] = 0x40;
        if (memcmp(data,status, len) == 0 && trigger_status == TRIGGER_STATUS_ACTIVE)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x09;
                bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        }

        status[1] = 0xa0;
        status[2] = 0x20; 
        if(memcmp(data,status,len) == 0 && trigger_status == TRIGGER_STATUS_ACTIVE)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x08;
	        bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        }

        status[1] = 0xa0;
        status[2] = 0x00;
        if(memcmp(data,status,len) == 0 && trigger_status == TRIGGER_STATUS_ACTIVE)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x06;
	        bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
                trigger_status = TRIGGER_STATUS_READY;
                k_work_cancel_delayable(&scs_timeout);
        }

        LOG_HEXDUMP_INF(data,len,"Received");
	return BT_GATT_ITER_CONTINUE;
}

struct bt_scs_client_init_param init = {
        .cb = {
                .received = camera_status_received,
                .sent = camera_cmd_sent}};

void device_found(const bt_addr_le_t *addr, const struct bt_le_conn_param *conn_param)
{    
        create_le_connnection(&conn_context, addr, conn_param);
}

const scan_context_t scan_context = {
        .pheriphral_addr = 
        {
                .val = {0x3a, 0xe0, 0xdb, 0xef, 0x40, 0xd0}
        },
        .pheriphral_manufacture_data_normal_mode = pheriphral_manufacture_data_normal_mode,
        .pheriphral_manufacture_data_pairing_mode = pheriphral_manufacture_data_pairing_mode,
        .pheriphral_manufacture_data_len = 21,
        .pheriphral_found = device_found
};

#define CALIBRATION_DURATION_MS 10000

void reset_to_uf2(void) {
  printk("resetting .. \n");      
  NRF_POWER->GPREGRET = 0x57; // 0xA8 OTA, 0x4e Serial
  sys_reboot(SYS_REBOOT_COLD);  // NVIC_SystemReset(); or sd_nvic_SystemReset();
}

static void proximity_timeout_cb(struct k_work *item)
{
       sensor_status = SENSOR_STATUS_FAR;
       max30102_reset(max30102_dev);
       k_msleep(10);
       max30102_configure(max30102_dev,7,20); 
}

void mainloop(int err)
{
        float high_point = 0 , low_point = 2000000;
        float proximity;
        uint32_t calibration_start = k_uptime_get_32() + 10000;
        uint32_t calibration_stop = k_uptime_get_32() + 30000;
        uint32_t current_time;

	
        while (1)
        {
                switch (cmd)
                {
                case 'r':
                        reset_to_uf2();
                        break;
                case 'h':
                        max30102_reset(max30102_dev);
                        k_msleep(10);
                        max30102_configure(max30102_dev,7,20);
                        calibration_start = k_uptime_get_32()+ 10000;
                        calibration_stop = k_uptime_get_32() + 20000;
                        high_point = 0;
                        low_point = 2000000;
                        iir_filter_reset(filt);
                        printk("Resetting, wait for 10s...");
                        break;
                case 'l':
                        max30102_reset(max30102_dev);
                        k_msleep(10);
                        max30102_configure(max30102_dev,7,15);
                        calibration_start = k_uptime_get_32()+ 10000;
                        calibration_stop = k_uptime_get_32() + 20000;
                        high_point = 0;
                        low_point = 2000000;
                        iir_filter_reset(filt);
                        printk("Resetting, wait for 10s...");
                        break;
                default:
                        break;
                }
                cmd = '\0';
                if (!err)
                {
                        proximity = max30102_read_ir_data(max30102_dev);
                        proximity = iir_filter(filt, proximity);

                        current_time = k_uptime_get_32();
                         
                        if(current_time > calibration_start && current_time < calibration_stop)
                        {
                                gpio_pin_set_dt(&led_blue,1);
                                gpio_pin_set_dt(&led_red, 0);
                                gpio_pin_set_dt(&led_green,0);
                                if(high_point < proximity)
                                {
                                        high_point = proximity;
                                }

                                if(low_point > proximity)
                                {
                                        low_point = proximity;
                                }

                                printk(">proximity : %f , high point : %f , low point : %f  trigger: %d,  sensor : %d\n", proximity,high_point,low_point, trigger_status,sensor_status);
                        }   
                        else if(current_time > calibration_stop && 
                               (((high_point -500) - (low_point+500)) > 500) && 
                               ((high_point-500) > (low_point+500)))         
                        {      
                                if(trigger_status == TRIGGER_STATUS_ACTIVE)
                                {
                                        gpio_pin_set_dt(&led_green,1);
                                        gpio_pin_set_dt(&led_red, 0);
                                        gpio_pin_set_dt(&led_blue,0);
                                }
                                else
                                {
                                        gpio_pin_set_dt(&led_blue,0);
                                        gpio_pin_set_dt(&led_red, 1);
                                        gpio_pin_set_dt(&led_green,0);
                                }
                                
                                if(proximity > (high_point - 500) && sensor_status == SENSOR_STATUS_FAR)
                                {       
                                        printk("extruder proximity detected\n");
                                        sensor_status = SENSOR_STATUS_PROXIMITY;
                                        k_work_schedule(&proximity_timeout,K_MSEC(10000));
                                        printk(">proximity : %f , high point : %f , low point : %f  trigger: %d,  sensor : %d\n", proximity,high_point,low_point, trigger_status,sensor_status);
                                        if(trigger_status == TRIGGER_STATUS_READY)
                                        {
                                                trigger_status = TRIGGER_STATUS_ACTIVE;
                                                k_work_schedule(&scs_timeout, FOCUS_TIMEOUT);
                                                bt_scs_trigger_capture(&scs_client);
                                        }
                                }
                                if(proximity < (low_point + 500) && sensor_status == SENSOR_STATUS_PROXIMITY)
                                {
                                        k_work_cancel_delayable(&proximity_timeout);
                                        printk(">proximity : %f , high point : %f , low point : %f  , trigger: %d,  sensor : %d\n", proximity,high_point,low_point, trigger_status,sensor_status);
                                        printk("extruder far \n");
                                        sensor_status = SENSOR_STATUS_FAR;
                                }
                        }     
                        else if(current_time > calibration_stop)
                        {
                            // if calibration failed increase calibration time by 10 s 
                            printk("calibration failed , retrying ..");   
                            calibration_stop =  k_uptime_get_32() + 10000;   
                        }
                }

                k_msleep(10);
        }
}

static void interrupt_handler(const struct device *dev, void *user_data)
{

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}
                        cmd = buffer[0];
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				uart_irq_tx_disable(dev);
				continue;
			}

			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}
		}
	}
}

static void list_bonded_devices(const struct bt_bond_info *info, void *user_data)
{
        char saddr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&(info->addr), saddr, sizeof(saddr));

        printk("Bound Device found: %s \n", saddr);
}

int main(void)
{
        int err;
        bt_addr_le_t addr;

        if(!device_is_ready(gpio_rdivider_en))
        {
                return 0;
        }
        /*** Very critical for battery charging ***/ 
        /*** see https://wiki.seeedstudio.com/XIAO_BLE/#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging ***/      
        gpio_pin_configure(gpio_rdivider_en, GPIO_BATTERY_READ_ENABLE, GPIO_OUTPUT|GPIO_ACTIVE_LOW);
        gpio_pin_set(gpio_rdivider_en, GPIO_BATTERY_READ_ENABLE, 1);
        if (!device_is_ready(console_dev)) 
        {
		return 0;
	}
        /****************************************/       

        ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

        err = bt_enable(NULL);
        if (err)
        {

                printk("Bluetooth Failed !\n");
        }
        else
        {
                if(!gpio_is_ready_dt(&led_blue) ||  !gpio_is_ready_dt(&led_green) || !gpio_is_ready_dt(&led_red))
                {
                        return 0;
                }
                gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
                gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
                gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
                
                gpio_pin_set_dt(&led_blue,0);
                gpio_pin_set_dt(&led_red, 0);
                gpio_pin_set_dt(&led_green,0);

               /*       
                err = bt_addr_le_from_str("11:22:33:44:55:66","random",&addr);
                err = bt_id_create(&addr,NULL);

                err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
                printk("Bluetooth Advertising !!\n");
                */

                /* TODO: Handle bonding removal , and multiple bonded devices */
                if (IS_ENABLED(CONFIG_SETTINGS))
                {
                        settings_load();
                }
                
                bt_foreach_bond(BT_ID_DEFAULT, list_bonded_devices, NULL);

                err = bt_scs_client_init(&scs_client, &init);

	        if (err != 0) 
                {
		        printk("bt_scs_client_init failed (err %d)", err);
		        return 0;
	        }

                scan_init(false, &scan_context);
                scan_start();

                filt = iir_filter_create(iir_2_coeff);
                iir_filter_reset(filt);

                max30102_configure(max30102_dev,7,20);
        }
        
        uart_irq_callback_set(console_dev, interrupt_handler);

	/* Enable rx interrupts */
        /* TODO: Gatt server can be used for reception of CMDs from phone */
	uart_irq_rx_enable(console_dev);
        
        k_work_init_delayable(&scs_timeout, scs_timeout_cb);
        k_work_init_delayable(&proximity_timeout, proximity_timeout_cb);
        mainloop(err);
        return 0;
}
