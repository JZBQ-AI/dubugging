#include "I2C.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static struct i2c_dt_spec afe_bus = I2C_DT_SPEC_GET(DT_NODELABEL(afe));

/*
 * 有的 Maxim 器件不接受 repeated START，要求“写寄存器地址 -> STOP -> 再读”。
 * 先按标准 repeated START 读，失败就退回两段式再试一次，成功就记住这个选择。
 */
static bool split_read;

void max30009_i2c_set_addr(uint8_t addr)
{
	afe_bus.addr = addr;
}

int max30009_i2c_read(uint8_t reg, uint8_t *buf, size_t len)
{
	int ret;

	if (!split_read) {
		ret = i2c_burst_read_dt(&afe_bus, reg, buf, len);
		if (ret == 0) {
			return 0;
		}

		if (i2c_write_dt(&afe_bus, &reg, 1) == 0 &&
		    i2c_read_dt(&afe_bus, buf, len) == 0) {
			split_read = true;
			printk("MAX30009: 该器件要 write+STOP+read，后续都用两段式\n");
			return 0;
		}

		return ret;
	}

	ret = i2c_write_dt(&afe_bus, &reg, 1);
	if (ret < 0) {
		return ret;
	}

	return i2c_read_dt(&afe_bus, buf, len);
}

int max30009_i2c_write(uint8_t reg, uint8_t value)
{
	uint8_t b[2] = {reg, value};

	return i2c_write_dt(&afe_bus, b, sizeof(b));
}
