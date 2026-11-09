#ifndef MAX30009_I2C_H_
#define MAX30009_I2C_H_

#include <stddef.h>
#include <stdint.h>

/*
 * MAX30009 I2C 传输层。
 * 只负责“寄存器地址 + 数据”的 I2C 读写，不包含 MAX30009 的寄存器配置。
 */

int max30009_i2c_read(uint8_t reg, uint8_t *buf, size_t len);
int max30009_i2c_write(uint8_t reg, uint8_t value);

/* 修改 I2C 从机地址，例如 ADDR 脚选择 0x68/0x69。 */
void max30009_i2c_set_addr(uint8_t addr);

#endif /* MAX30009_I2C_H_ */
