#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <stddef.h>
#include <stdint.h>

/* 外部 TMUX1108 多路选择器：每个 MUX 对应一个 EL 网络。 */
enum {
	MUX_EL1 = 0,
	MUX_EL2B,
	MUX_EL3B,
	MUX_EL4,
	MUX_COUNT
};

/*
 * channel 就是 S1~S8 的编号减 1：
 *   0=S1, 1=S2, 2=S3, 3=S4, 4=S5, 5=S6, 6=S7, 7=S8
 */
int mux_select(size_t mux, uint8_t channel);

/* 配置四个 MUX 的 GPIO，并选择 S5（E5/E7/E6/E8 -> EL1/EL2B/EL3B/EL4）。 */
int mux_init(void);

#endif /* HARDWARE_H_ */
