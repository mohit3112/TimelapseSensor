#ifndef _PPG_IIR_FILTER
#define _PPG_IIR_FILTER
#include <stdint.h>
#include <stdbool.h>
// 2nd order IIR filter with given coefficient
typedef struct
{
	float a[3];
	float b[3];
	float g;
} iir_filter_coeff_t;

typedef struct
{
	uint16_t filter_length;
	uint16_t num_values_in;

	iir_filter_coeff_t iir_coeff;

	float *raw;
	float *processed;

} iir_filter_data_t;

extern const iir_filter_coeff_t iir_2_coeff;

iir_filter_data_t *iir_filter_create(iir_filter_coeff_t iir_2nd_order_coeff); // For now the order is 2(fixed). sampling freq can be 193Hz, 50 Hz
void iir_filter_reset(iir_filter_data_t *filter_data);
float iir_filter(iir_filter_data_t *filter_data, float new_value);
void iir_filter_destory(iir_filter_data_t **filter_data);

void iir_filter_change_coeff(iir_filter_data_t *filter_data, iir_filter_coeff_t iir_2nd_order_coeff); // For now the order is 2(fixed). sampling freq can be 193Hz, 50 Hz

#endif