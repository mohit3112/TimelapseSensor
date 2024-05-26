#include "max30102_driver_i2c.h"
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <ncs_version.h>
#include <zephyr/syscall_handler.h>
#include <zephyr/logging/log.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MAX30102 driver enabled without any devices"
#endif
#include <zephyr/drivers/i2c.h>

LOG_MODULE_REGISTER(Driver, LOG_LEVEL_DBG);

struct sensor_data {
	uint32_t ir_intensity;
    uint32_t red_intensity;
} sensordata;

struct sensor_config {
	struct i2c_dt_spec i2c;
    uint8_t mode;
    uint8_t ledPulseAmplitude;
};

static int sensor_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
{
    const struct sensor_config *sensor_config = dev->config;
    return i2c_write_read_dt(&sensor_config->i2c, &reg, 1, data, size);    
}

static int sensor_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
    const struct sensor_config *sensor_config = dev->config;
    uint8_t write_config[2];
    write_config[0] = reg;
    write_config[1] = value;
    return i2c_write_dt(&sensor_config->i2c, &write_config[0], 2);
}

static int init(const struct device *dev)
{
	int err;
    uint8_t data;
	const struct sensor_config *sensor_config = dev->config;
	err = i2c_is_ready_dt(&sensor_config->i2c);

	if (!err) 
    {
		printk("I2C bus not ready !: %d\n\r", err);
		return 0;
	}
    else
    {
        err = sensor_read_reg(dev, 0xFF, &data, 1);
        if (err != 0)
        {
            printk("can't read %d\n\r", err);
        }
        else
        {
            printk("data %d\n\r", data);
        }
    }

	return err;
}

static void sensor_configure(const struct device *dev, uint8_t mode, uint8_t led_pulse_amp)
{
    sensor_write_reg(dev, 0x09, mode);
    sensor_write_reg(dev, 0x0c, led_pulse_amp);
    if (mode == 7)
    {
        sensor_write_reg(dev, 0x11, 0x01);
    }
}

static int sensor_read_ir_data(const struct device *dev)
{
    uint8_t data_value[3];
    uint32_t proximity;
    sensor_read_reg(dev, 0x07,data_value,3);
    proximity = data_value[2] | (data_value[1] << 8) | (data_value[0] << 16);
    return proximity;
}

static const struct max30102_driver_api max30102_api_funcs = {
	.write_reg  = sensor_write_reg,
	.read_reg   = sensor_read_reg,
    .configure  = sensor_configure,
    .read_ir_data = sensor_read_ir_data
};

#define MAX30102_CONFIG_I2C(inst)				\
	{						\
		.i2c = I2C_DT_SPEC_INST_GET(inst),	\
	}

#define MAX30102_DEFINE(inst)						\
	static struct sensor_data sensor_data_##inst;			\
	static const struct sensor_config sensor_config_##inst = MAX30102_CONFIG_I2C(inst);	\
	DEVICE_DT_INST_DEFINE(inst,			\
				init,							\
				NULL,							\
				&sensor_data_##inst,	\
				&sensor_config_##inst,\
				POST_KERNEL, \
				CONFIG_DRIVER_INIT_PRIORITY, \
				&max30102_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MAX30102_DEFINE)