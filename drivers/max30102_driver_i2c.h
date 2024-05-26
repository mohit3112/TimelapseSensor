#ifndef __MAX30102_DRIVER_H__
#define __MAX30102_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>

#define DT_DRV_COMPAT maxim_max301021

typedef int  (*max30102_api_read_reg_t)(const struct device * dev, uint8_t reg, uint8_t *data, int size);
typedef int  (*max30102_api_write_reg_t)(const struct device * dev, uint8_t reg, uint8_t value);
typedef void  (*max30102_api_configure_t)(const struct device *dev, uint8_t mode , uint8_t led_pulse_amp);
typedef int  (*max30102_api_read_ir_data_t)(const struct device *dev);

struct max30102_driver_api {
	max30102_api_read_reg_t read_reg;
	max30102_api_write_reg_t write_reg;
	max30102_api_configure_t configure;
	max30102_api_read_ir_data_t read_ir_data;
};

__syscall   int  max30102_read_reg(const struct device * dev, uint8_t reg, uint8_t *data, int size);
static inline int z_impl_max30102_read_reg(const struct device * dev, uint8_t reg, uint8_t *data, int size)
{
	const struct max30102_driver_api *api = dev->api;

	__ASSERT(api->read_reg, "Callback pointer should not be NULL");

	return api->read_reg(dev, reg, data, size);
}

__syscall   int  max30102_write_reg(const struct device * dev, uint8_t reg, uint8_t value);
static inline int z_impl_max30102_write_reg(const struct device * dev, uint8_t reg, uint8_t value)
{
	const struct max30102_driver_api *api = dev->api;

	__ASSERT(api->write_reg, "Callback pointer should not be NULL");

	return api->write_reg(dev, reg, value);
}

__syscall   void  max30102_configure(const struct device *dev, uint8_t mode , uint8_t led_pulse_amp);
static inline void z_impl_max30102_configure(const struct device *dev, uint8_t mode , uint8_t led_pulse_amp)
{
	const struct max30102_driver_api *api = dev->api;

	__ASSERT(api->configure, "Callback pointer should not be NULL");

	api->configure(dev, mode, led_pulse_amp);
}

__syscall   int  max30102_read_ir_data(const struct device * dev);
static inline int z_impl_max30102_read_ir_data(const struct device * dev)
{
	const struct max30102_driver_api *api = dev->api;

	__ASSERT(api->read_ir_data, "Callback pointer should not be NULL");

	return api->read_ir_data(dev);
}

#ifdef __cplusplus
}
#endif
#include <syscalls/max30102_driver_i2c.h>
#endif 