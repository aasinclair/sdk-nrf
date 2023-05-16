/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "fuel_gauge.h"

#define SLEEP_TIME_MS 1000

/* nPM1300 LED register addresses */
#define LED_BASE 0x0AU
#define LED_OFFSET_MODE2 0x02U
#define LED_OFFSET_SET2 0x07U
#define LED_OFFSET_CLR2 0x08U
#define LED_MODE_HOST 0x2U

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_charger));

static const struct i2c_dt_spec pmic_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(npm1300_ek_pmic));

static int reg_write(uint8_t base, uint8_t offset, uint8_t data)
{
	uint8_t buff[] = { base, offset, data };

	return i2c_write_dt(&pmic_i2c, buff, sizeof(buff));
}

static bool configure_ui(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button1)) {
		printk("Error: button device %s is not ready\n", button1.port->name);
		return false;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button1.port->name,
		       button1.pin);
		return false;
	}

	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);

	if (!i2c_is_ready_dt(&pmic_i2c)) {
		printk("Error: i2c bus is not ready\n");
		return false;
	}

	ret = reg_write(LED_BASE, LED_OFFSET_MODE2, LED_MODE_HOST);
	if (ret != 0) {
		printk("Error: failed to configure PMIC LED\n");
		return false;
	}

	return true;
}

int main(void)
{
	if (!device_is_ready(charger)) {
		printk("Charger device not ready.\n");
		return 0;
	}

	if (fuel_gauge_init(charger) < 0) {
		printk("Could not initialise fuel gauge.\n");
	}

	if (!configure_ui()) {
		printk("UI initialisation failed.\n");
		return 0;
	}

	printk("PMIC device ok\n");

	while (1) {
		if (gpio_pin_get_dt(&button1) != 0) {
			reg_write(LED_BASE, LED_OFFSET_SET2, 1U);
		} else {
			reg_write(LED_BASE, LED_OFFSET_CLR2, 1U);
		}

		fuel_gauge_update(charger);
		k_msleep(SLEEP_TIME_MS);
	}
}
