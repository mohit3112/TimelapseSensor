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

#define I2C1_NODE DT_NODELABEL(max30102)
#define SENSOR_NODE DT_NODELABEL(max301021)
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NONE DT_ALIAS(led2)

LOG_MODULE_REGISTER(Main, LOG_LEVEL_DBG);

static const struct i2c_dt_spec bambu_timelapse_sensor_i2c = I2C_DT_SPEC_GET(I2C1_NODE);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED2_NONE, gpios);
const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct device *proximity_dev = DEVICE_DT_GET(SENSOR_NODE);

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
void setup_bambutimelapse_sensor(uint8_t mode, uint8_t ledPulseAmplitude);
int init_bambutimelapse_sensor(void)
{
        uint8_t data;
        int ret;
        uint8_t ic_id_address = 0xFF;
        if (!device_is_ready(bambu_timelapse_sensor_i2c.bus))
        {
                printk("I2C bus not ready !\n\r ");
        }
        else
        {
                ret = i2c_write_read_dt(&bambu_timelapse_sensor_i2c, &ic_id_address, 1, &data, 1);
                if (ret != 0)
                {

                        printk("can't read %d\n\r", ret);
                }
                else
                {
                        printk("data %d\n\r", data);
                        setup_bambutimelapse_sensor(7, 30);
                }
        }

        return ret;
}
void setup_bambutimelapse_sensor(uint8_t mode, uint8_t ledPulseAmplitude)
{
        uint8_t data_value[2];
        data_value[0] = 0x09;
        data_value[1] = mode;
        i2c_write_dt(&bambu_timelapse_sensor_i2c, &data_value[0], 2);
        data_value[0] = 0x0c;
        data_value[1] = ledPulseAmplitude;
        i2c_write_dt(&bambu_timelapse_sensor_i2c, &data_value[0], 2);

        if (mode == 7)
        {
                data_value[0] = 0x11;
                data_value[1] = 0x01;
                i2c_write_dt(&bambu_timelapse_sensor_i2c, &data_value[0], 2);
        }
}

float read_bambautimelapse_sensor_data(void)
{
        uint8_t data_addr, data_value[3];
        uint32_t proximity;
        float filter_proximity;
        data_addr = 0x07;
        i2c_write_read_dt(&bambu_timelapse_sensor_i2c, &data_addr, 1, &data_value[0], 3);
        proximity = data_value[2] | (data_value[1] << 8) | (data_value[0] << 16);
        filter_proximity = iir_filter(filt, proximity);
        return filter_proximity;
}

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

static uint8_t camera_status_received(struct bt_scs_client *scs,
						const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(scs);
        uint8_t status[]= {0x02,0x3f,0x20};
        uint8_t cmd[2];

        if (memcmp(data,status, len) == 0)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x09;
	        bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        }
        status[1] = 0xa0;
        status[2] = 0x20; 
        if(memcmp(data,status,len) == 0)
        {

                cmd[0] = 0x01;
	        cmd[1] = 0x08;
	        bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
        }
        status[1] = 0xa0;
        status[2] = 0x00;
        if(memcmp(data,status,len) == 0)
        {
                cmd[0] = 0x01;
	        cmd[1] = 0x06;
	        bt_scs_client_send_camera_cmd(&scs_client, cmd,2);
                trigger_status = TRIGGER_STATUS_READY;
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
 // NVIC_SystemReset();         // or sd_nvic_SystemReset();
  sys_reboot(SYS_REBOOT_COLD);
}

void mainloop(int err)
{
        float high_point = 0 , low_point = 100000;
        float proximity;
        uint32_t calibration_start = k_uptime_get_32() + 10000;
        uint32_t calibration_stop = k_uptime_get_32() + 30000;
        uint32_t current_time;

	
        while (1)
        {
                if(cmd == 'r')
                {
                        reset_to_uf2();
                }
                if (!err)
                {
                        proximity = read_bambautimelapse_sensor_data();
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

                               printk(">proximity : %f , high point : %f , low point : %f  \n", proximity,high_point,low_point);
                        }   
                        else if(current_time > calibration_stop && (high_point - low_point > 1000))        
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
                                
                                if(proximity > (high_point - 1000) && sensor_status == SENSOR_STATUS_FAR)
                                {       
                                        printk("sensor proximity detected\n");
                                        sensor_status = SENSOR_STATUS_PROXIMITY;
                                        if(trigger_status == TRIGGER_STATUS_READY)
                                        {
                                                trigger_status = TRIGGER_STATUS_ACTIVE;
                                                bt_scs_trigger_capture(&scs_client);
                                        }
                                           
                                }
                                if(proximity < (low_point + 1000) && sensor_status == SENSOR_STATUS_PROXIMITY)
                                {
                                        printk("sensor far \n");
                                        sensor_status = SENSOR_STATUS_FAR;
                                }
                        }     
                        else
                        {
                            // if calibration failed increase calibration time by 10 s    
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


int main(void)
{
        int err;
        bt_addr_le_t addr;

        if (!device_is_ready(dev)) 
        {
		return 0;
	}

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

                err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
                printk("Bluetooth Advertising !!\n");
                */
                
                if (IS_ENABLED(CONFIG_SETTINGS))
                {
                        settings_load();
                }
                
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

                err = init_bambutimelapse_sensor();
        }
        
        uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
        mainloop(err);
        return 0;
}
