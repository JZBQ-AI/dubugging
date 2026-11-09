#include "hardware.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define MUX_DISABLE_PIN 9 /* P1.09：高 = 禁用所有 MUX，低 = 使能 */

static const struct device *const gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *const gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* 顺序与 DTS 中 mux-address-gpios 一致：每组为 A0、A1、A2。 */
static const gpio_pin_t mux_addr_pins[MUX_COUNT][3] = {
	{23, 20, 21}, /* EL1  */
	{7,  5,  26}, /* EL2B */
	{19, 17, 15}, /* EL3B */
	{16, 14, 13}, /* EL4  */
};

int mux_select(size_t mux, uint8_t channel)
{
	int ret;

	if (mux >= MUX_COUNT || channel > 7u) {
		return -EINVAL;
	}

	for (size_t bit = 0; bit < 3u; bit++) {
		ret = gpio_pin_set(gpio0_dev, mux_addr_pins[mux][bit],
				   (channel >> bit) & 1u);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int mux_init(void)
{
	int ret;

	/* 先禁用，避免地址切换过程中短暂接通错误电极。 */
	ret = gpio_pin_configure(gpio1_dev, MUX_DISABLE_PIN, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		return ret;
	}

	for (size_t mux = 0; mux < MUX_COUNT; mux++) {
		for (size_t bit = 0; bit < 3u; bit++) {
			ret = gpio_pin_configure(gpio0_dev,
						 mux_addr_pins[mux][bit],
						 GPIO_OUTPUT_LOW);
			if (ret < 0) {
				return ret;
			}
		}
	}

	/* 原理图中四个 MUX 的公共端 D 分别是 E5/E7/E6/E8，
	 * S5 分别接 EL1/EL2B/EL3B/EL4，所以四个 MUX 都选 S5（地址 100 = 4）：
	 *   MUX1: E5  -> EL1
	 *   MUX2: E7  -> EL2B
	 *   MUX3: E6  -> EL3B
	 *   MUX4: E8  -> EL4
	 */
	ret = mux_select(MUX_EL1, 4);
	if (ret < 0) {
		return ret;
	}
	ret = mux_select(MUX_EL2B, 4);
	if (ret < 0) {
		return ret;
	}
	ret = mux_select(MUX_EL3B, 4);
	if (ret < 0) {
		return ret;
	}
	ret = mux_select(MUX_EL4, 4);
	if (ret < 0) {
		return ret;
	}

	/* MUX_DISABLE 为高有效：拉低使能。 */
	return gpio_pin_set(gpio1_dev, MUX_DISABLE_PIN, 0);
}
