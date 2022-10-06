/*
 * Copyright (c) 2020 Lexmark International, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tracing_user.h>
#include <zephyr/drivers/gpio.h>
#include <ksched.h>

static const struct gpio_dt_spec user7 = GPIO_DT_SPEC_GET(DT_ALIAS(user7), gpios);
static const struct gpio_dt_spec user2 = GPIO_DT_SPEC_GET(DT_ALIAS(user2), gpios);
static const struct gpio_dt_spec user3 = GPIO_DT_SPEC_GET(DT_ALIAS(user3), gpios);

static const char* rx_thread_name = "rx_q[0]";

void sys_trace_thread_switched_in_user(struct k_thread *thread)
{
	if( z_is_idle_thread_object(thread)) {
		// gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
		// gpio_pin_configure_dt(&user2, GPIO_OUTPUT_INACTIVE);
	}
	else if (thread->custom_data) {
		gpio_pin_configure_dt((struct gpio_dt_spec *) thread->custom_data, GPIO_OUTPUT_INACTIVE);
	} else if( strncmp( thread->name, rx_thread_name, strnlen( rx_thread_name, CONFIG_THREAD_MAX_NAME_LEN ) ) == 0 ) {
		gpio_pin_configure_dt(&user3, GPIO_OUTPUT_ACTIVE);
		gpio_pin_configure_dt(&user3, GPIO_OUTPUT_INACTIVE);
	}
}

void sys_trace_thread_switched_out_user(struct k_thread *thread)
{
	if( z_is_idle_thread_object(thread)) {
		// gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
	} else if (thread->custom_data) {
		gpio_pin_configure_dt((struct gpio_dt_spec *) thread->custom_data, GPIO_OUTPUT_ACTIVE);
	} else if( strncmp( thread->name, rx_thread_name, strnlen( rx_thread_name, CONFIG_THREAD_MAX_NAME_LEN ) ) == 0 ) {
		gpio_pin_configure_dt(&user3, GPIO_OUTPUT_ACTIVE);
	}
}

void sys_trace_isr_enter_user(int nested_interrupts)
{
	gpio_pin_configure_dt(&user7, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&user7, GPIO_OUTPUT_INACTIVE);
}

void sys_trace_isr_exit_user(int nested_interrupts)
{
	gpio_pin_configure_dt(&user7, GPIO_OUTPUT_ACTIVE);
}
