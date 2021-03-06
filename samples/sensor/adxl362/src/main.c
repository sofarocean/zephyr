/*
 * Copyright (c) 2019 Brett Witherspoon
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

K_SEM_DEFINE(sem, 0, 1);

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		if (sensor_sample_fetch(dev) < 0) {
			printf("Sample fetch error\n");
			return;
		}
		k_sem_give(&sem);
		break;
	case SENSOR_TRIG_MOTION:
		printf("Motion trigger\n");
		break;
	case SENSOR_TRIG_STATIONARY:
		printf("Stationary trigger\n");
		break;
	default:
		printf("Unknown trigger\n");
	}
}

void main(void)
{
	struct sensor_value accel[3];

	const struct device *dev = DEVICE_DT_GET_ONE(adi_adxl362);

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	if (IS_ENABLED(CONFIG_ADXL362_TRIGGER)) {
		struct sensor_trigger trig = { .chan = SENSOR_CHAN_ACCEL_XYZ };

		trig.type = SENSOR_TRIG_MOTION;
		if (sensor_trigger_set(dev, &trig, trigger_handler)) {
			printf("Trigger set error\n");
			return;
		}

		trig.type = SENSOR_TRIG_STATIONARY;
		if (sensor_trigger_set(dev, &trig, trigger_handler)) {
			printf("Trigger set error\n");
			return;
		}

		trig.type = SENSOR_TRIG_DATA_READY;
		if (sensor_trigger_set(dev, &trig, trigger_handler)) {
			printf("Trigger set error\n");
		}
	}

	while (true) {
		if (IS_ENABLED(CONFIG_ADXL362_TRIGGER)) {
			k_sem_take(&sem, K_FOREVER);
		} else {
			k_sleep(K_MSEC(1000));
			if (sensor_sample_fetch(dev) < 0) {
				printf("Sample fetch error\n");
				return;
			}
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0) {
			printf("Channel get error\n");
			return;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0) {
			printf("Channel get error\n");
			return;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0) {
			printf("Channel get error\n");
			return;
		}

		printf("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}
}
