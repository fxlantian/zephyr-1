/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <shell/shell.h>
#include <cam.h>
#include <spi.h>
u16_t BACK_COLOR, POINT_COLOR;
static int shell_cmd_ping(int argc, char *argv[])
{
	struct device *lcd_dev;
    lcd_dev = device_get_binding("lcd_0");
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	printk("pong\n");
	printf_lcd("pong", lcd_dev);

	return 0;
}

static int shell_cmd_params(int argc, char *argv[])
{
	struct device *lcd_dev;
    lcd_dev = device_get_binding("lcd_0");
    const char *str;
	int cnt;

	printk("argc = %d\n", argc);
	for (cnt = 0; cnt < argc; cnt++) {
		printk("  argv[%d] = %s\n", cnt, argv[cnt]);
	}
	return 0;
}

#define MY_SHELL_MODULE "sample_module"

static struct shell_cmd commands[] = {
	{ "ping", shell_cmd_ping, NULL },
	{ "params", shell_cmd_params, "print argc" },
	{ NULL, NULL, NULL }
};


void main(void)
{
	struct device *lcd_dev;
    lcd_dev = device_get_binding("lcd_0");
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    lcd_config(lcd_dev, gpio_dev);
    BACK_COLOR=0x000000;
  	POINT_COLOR=0xFCFCFC;
	SHELL_REGISTER(MY_SHELL_MODULE, commands);
}
