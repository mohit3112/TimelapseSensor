#include "iir_filter.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/* 2.5 Hz iir filter */
const iir_filter_coeff_t iir_2_coeff =
	{
		.a = {1.0, -1.77863178, 0.80080265},
		.b = {0.00554272, 0.01108543, 0.00554272},
		.g = 1};

iir_filter_data_t *iir_filter_create(iir_filter_coeff_t iir_2nd_order_coeff)
{

	iir_filter_data_t *filter_data = (iir_filter_data_t *)malloc(sizeof(iir_filter_data_t));
	if (filter_data == NULL)
		return NULL;

	filter_data->filter_length = 2;

	filter_data->iir_coeff = iir_2nd_order_coeff;

	filter_data->raw = (float *)malloc(sizeof(float) * filter_data->filter_length);
	if (filter_data->raw == NULL)
	{
		free(filter_data);
		return NULL;
	}
	filter_data->processed = (float *)malloc(sizeof(float) * filter_data->filter_length);
	if (filter_data->raw == NULL)
	{
		free(filter_data);
		return NULL;
	}

	filter_data->num_values_in = 0;

	return filter_data;
}

void iir_filter_reset(iir_filter_data_t *filter_data)
{
	uint8_t i;

	if (filter_data == NULL)
		return;

	for (i = 0; i < filter_data->filter_length; i++)
	{
		filter_data->raw[i] = 0;
		filter_data->processed[i] = 0;
	}

	filter_data->num_values_in = 0;
}

float iir_filter(iir_filter_data_t *filter_data, float new_value)
{
	float filtered_value = 0.0;

	float b0, b1, b2;
	float a1, a2;
	float G;

	if (filter_data == NULL)
	{
		printk("filter_data() NULL i/p\n");
		return 0;
	}

	b0 = filter_data->iir_coeff.b[0];
	b1 = filter_data->iir_coeff.b[1];
	b2 = filter_data->iir_coeff.b[2];
	a1 = filter_data->iir_coeff.a[1];
	a2 = filter_data->iir_coeff.a[2];
	G = filter_data->iir_coeff.g;

	if (filter_data->num_values_in == filter_data->filter_length)
	{
		filtered_value = G * (b0 * new_value + b1 * filter_data->raw[1] + b2 * filter_data->raw[0]) - a1 * filter_data->processed[1] - a2 * filter_data->processed[0];

		for (uint16_t index = 0; index < (filter_data->filter_length - 1); ++index)
		{
			filter_data->raw[index] = filter_data->raw[index + 1];
			filter_data->processed[index] = filter_data->processed[index + 1];
		}
	}
	else
	{
		filter_data->num_values_in++;
	}
	filter_data->raw[1] = new_value;
	filter_data->processed[1] = filtered_value;

	return filtered_value;
}

void iir_filter_destory(iir_filter_data_t **filter_data)
{
	if (*filter_data != NULL)
	{
		free((*filter_data)->raw);
		free((*filter_data)->processed);
		free(*filter_data);
		*filter_data = NULL;
	}
}