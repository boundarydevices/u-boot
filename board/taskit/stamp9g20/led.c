/*
 * Copyright (c) 2009 Wind River Systems, Inc.
 * Tom Rix <Tom.Rix@windriver.com>
 * (C) Copyright 2009
 * Eric Benard <eric@eukrea.com>
 *
 * (C) Copyright 2012
 * Markus Hubig <mhubig@imko.de>
 * IMKO GmbH <www.imko.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/at91_pmc.h>
#include <status_led.h>

static unsigned int saved_state[3] = {STATUS_LED_OFF,
	STATUS_LED_OFF, STATUS_LED_OFF};

void coloured_LED_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;

	/* Enable the clock */
	writel(ATMEL_ID_PIOC, &pmc->pcer);

	at91_set_gpio_output(CONFIG_RED_LED, 1);
	at91_set_gpio_output(CONFIG_GREEN_LED, 1);
	at91_set_gpio_output(CONFIG_YELLOW_LED, 1);

	at91_set_gpio_value(CONFIG_RED_LED, 0);
	at91_set_gpio_value(CONFIG_GREEN_LED, 1);
	at91_set_gpio_value(CONFIG_YELLOW_LED, 0);
}

void red_led_on(void)
{
	at91_set_gpio_value(CONFIG_RED_LED, 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	at91_set_gpio_value(CONFIG_RED_LED, 0);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	at91_set_gpio_value(CONFIG_GREEN_LED, 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	at91_set_gpio_value(CONFIG_GREEN_LED, 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void yellow_led_on(void)
{
	at91_set_gpio_value(CONFIG_YELLOW_LED, 1);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_ON;
}

void yellow_led_off(void)
{
	at91_set_gpio_value(CONFIG_YELLOW_LED, 0);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
	__led_set(mask, state);
}

void __led_toggle(led_id_t mask)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_RED])
			red_led_off();
		else
			red_led_on();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_GREEN])
			green_led_off();
		else
			green_led_on();

	} else if (STATUS_LED_YELLOW == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_YELLOW])
			yellow_led_off();
		else
			yellow_led_on();
	}
}

void __led_set(led_id_t mask, int state)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == state)
			red_led_on();
		else
			red_led_off();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == state)
			green_led_on();
		else
			green_led_off();

	} else if (STATUS_LED_YELLOW == mask) {
		if (STATUS_LED_ON == state)
			yellow_led_on();
		else
			yellow_led_off();
	}
}
