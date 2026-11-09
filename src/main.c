#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
//引脚定义
#define CSB_PIN   8   /* P0.08：CSB/I2C_SEL，高 = I2C 模式 */
#define ADDR_PIN  6   /* P0.06：SDO/ADDR，I2C 地址选择脚（不是 P0.04！） */
#define INT_PIN   11  /* P0.11：INT，本程序不用 */
#define LED_PIN   15  /* P1.15：板载 LED */
#define AFE_ADDR  0x69u 


static const struct device *const gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *const gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
static struct i2c_dt_spec afe_bus = I2C_DT_SPEC_GET(DT_NODELABEL(afe));

//MAX30009 寄存器定义
#define REG_STATUS1     0x00u
#define REG_STATUS2     0x01u
#define REG_FIFO_WR_PTR 0x08u
#define REG_FIFO_RD_PTR 0x09u
#define REG_FIFO_CNT1   0x0Au
#define REG_FIFO_CNT2   0x0Bu
#define REG_FIFO_DATA   0x0Cu
#define REG_FIFO_CFG1   0x0Du
#define REG_FIFO_CFG2   0x0Eu
#define REG_SYSTEM_SYNC 0x10u
#define REG_SYS_CFG1    0x11u
#define REG_PIN_FUNC    0x12u
#define REG_OUT_PIN     0x13u
#define REG_I2C_BCAST   0x14u
#define REG_PLL1        0x17u
#define REG_PLL2        0x18u
#define REG_PLL3        0x19u
#define REG_PLL4        0x1Au
#define REG_BIOZ_CFG1   0x20u /* 使能 / OSR */
#define REG_BIOZ_CFG2   0x21u /* 数字滤波 / 阈值 */
#define REG_BIOZ_CFG3   0x22u /* 激励电流 / 驱动模式 */
#define REG_BIOZ_CFG4   0x23u /* fast start */
#define REG_BIOZ_CFG5   0x24u /* AHPF / 增益 / INA */
#define REG_BIOZ_CFG6   0x25u /* 放大器 / DC restore */
#define REG_BIOZ_LOW_TH  0x26u
#define REG_BIOZ_HIGH_TH 0x27u
#define REG_BIOZ_CFG7   0x28u /* 斩波 / 相位 */
#define REG_MUX1        0x41u
#define REG_MUX2        0x42u
#define REG_MUX3        0x43u
#define REG_MUX4        0x44u /* 只读：BIST_R_ERR，不能写 */
#define REG_DC_LEADS    0x50u
#define REG_DC_LOFF_TH  0x51u
#define REG_LEAD_BIAS   0x58u
#define REG_INT_EN1     0x80u
#define REG_INT_EN2     0x81u
#define REG_PART_ID     0xFFu

#define PART_ID_EXPECTED 0x42u

/* ---- STATUS1 ---- */
#define ST1_PWR_RDY      BIT(0)
#define ST1_PHASE_LOCK   BIT(1)
#define ST1_PHASE_UNLOCK BIT(2)
#define ST1_FREQ_LOCK    BIT(3)
#define ST1_FREQ_UNLOCK  BIT(4)
#define ST1_FIFO_RDY     BIT(5)
#define ST1_A_FULL       BIT(7)
#define ST1_LOCKED_MASK  (ST1_PHASE_LOCK | ST1_PHASE_UNLOCK | \
			  ST1_FREQ_LOCK | ST1_FREQ_UNLOCK)
#define ST1_LOCKED_OK    (ST1_PHASE_LOCK | ST1_FREQ_LOCK)

/* ---- STATUS2 ---- */
#define ST2_DRV_OOR   BIT(4)
#define ST2_BIOZ_UNDR BIT(5)
#define ST2_BIOZ_OVER BIT(6)
#define ST2_LON       BIT(7)

/* ---- SYS_CFG1 ---- */
#define CFG1_RESET       BIT(0)
#define CFG1_SHDN        BIT(1) /* 1 = 关断；正常必须为 0 */
#define CFG1_DISABLE_I2C BIT(6) /* 置 1 后 I2C 失联，绝对不能写 */

/* ---- FIFO_CFG2 ---- */
#define FIFO_FLUSH    BIT(4)
#define FIFO_STAT_CLR BIT(3)

/* ---- FIFO 数据帧：24 bit = TAG[23:20] + DATA[19:0] ---- */
#define FIFO_BYTES_PER_SAMPLE 3u
#define FIFO_TAG_I            0x1u
#define FIFO_TAG_Q            0x2u
#define FIFO_MAX_WORDS        256u
#define FIFO_TAG_SHIFT        20u
#define FIFO_DATA_MASK        0x000FFFFFu
#define FIFO_DATA_SIGN_BIT    19u

/* =============================================================================
 * 3. 时钟与数字参数
 *    参考 32768 Hz × M(800) = 26.2144 MHz
 *    K=3  → 激励分频 (1<<3)×32 = 256  → 激励 102.4 kHz
 *    N=512、ADC_OSR=8<<6=512          → 采样 26.2144M/(512×512) = 100 sps
 * =============================================================================
 */
#define REF_CLK_HZ        32768u
#define PLL_MULTIPLIER    800u  /* MDIV 合法范围 427…854（32.768 kHz 参考） */
#define PLL_DIV_EXP_K     3u    /* 分频指数，K=3 → ÷8 */
#define PLL_NDIV_ENABLE   0u    /* 0 → N=512，1 → N=1024 */
#define BIOZ_ADC_OSR_CODE 6u    /* ADC OSR = 8 << 6 = 512 */
#define BIOZ_DAC_OSR_CODE 0u    /* DAC OSR = 32 << 0 = 32 */

/* PLL1: [7:6]=MDIV[9:8] [5]=NDIV [4:1]=K [0]=PLL_EN */
#define PLL1_STOP ((0xC0u) | (PLL_NDIV_ENABLE ? BIT(5) : 0u) | (PLL_DIV_EXP_K << 1))
#define PLL1_RUN  (PLL1_STOP | BIT(0))
#define PLL2_VAL  ((PLL_MULTIPLIER - 1u) & 0xFFu) /* 800-1=799=0x31F 的低 8 位 */
//选用外部晶振
#define USE_EXTERNAL_FCLK 1
#define PLL4_VAL ((USE_EXTERNAL_FCLK ? BIT(6) : 0u) | BIT(5))

/* BIOZ_CFG1: [7:6]=DAC_OSR [5:3]=ADC_OSR [2]=BG_EN [1]=Q_EN [0]=I_EN */
#define BIOZ1_BG_ONLY ((BIOZ_DAC_OSR_CODE << 6) | (BIOZ_ADC_OSR_CODE << 3) | BIT(2))
#define BIOZ1_RUN     (BIOZ1_BG_ONLY | BIT(1) | BIT(0))

/* FIFO_CFG1 是"还剩多少空位时置 A_FULL"。ADI 参考驱动的算法是
 * 写 (256 - 水位)：水位 32 → 写 224(0xE0)。本程序轮询计数，不依赖这个中断，
 * 写错也只影响 INT 脚，不影响通信。 */
#define FIFO_A_FULL_VAL 0xE0u

/* 单次最多从 FIFO 取多少 word（3 字节/word）。
 * 取 32 word = 96 字节，远小于 nRF TWIM 单次传输上限，也远小于 FIFO 深度。 */
#define FIFO_READ_MAX_WORDS 32u

/* =============================================================================
 * 4. 配置表
 * =============================================================================
 * 说明：
 *   - "CSV:" 是你快照里的原值；带 "修正" 的 4 行是按位定义改过的，原因都写在行尾。
 *   - 0x44 不在这里：它是 BIST_R_ERR，只读。CSV 里的 0xCD 是读出来的值，
 *     写进去会回读不一致，max30009_write_checked() 会直接报错退出。
 * =============================================================================
 */
struct reg_val {
	uint8_t reg;
	uint8_t val;
};

static const struct reg_val max30009_cfg[] = {
    {REG_SYS_CFG1,     0x00},   /* System Configuration 1 */
    {REG_PIN_FUNC,      0x04},   /* Pin Functional Configuration */
    {REG_OUT_PIN,       0x00},   /* Output Pin Configuration */
    {REG_I2C_BCAST,     0x00},   /* I2C Broadcast Address */

    /* --- PLL组（PLL_EN 仍然在init里单独置位，不在此表） --- */
	{REG_PLL1, 0x63},   /* mdiv=511, N=1024, K=2, PLL_EN=1 */
	{REG_PLL2, 0xFF},
	{REG_PLL3, 0x00},
	{REG_PLL4, 0x20},

    /* --- BioZ配置 --- */
    {REG_BIOZ_CFG2,     0x1E},   /* BioZ Configuration 2 */
    {REG_BIOZ_CFG3,     0x2C},   /* BioZ Configuration 3 */
    {REG_BIOZ_CFG4,     0x00},   /* BioZ Configuration 4 */
    {REG_BIOZ_CFG5,     0x73},   /* BioZ Configuration 5 */
    {REG_BIOZ_CFG6,     0xCA},   /* BioZ Configuration 6 */
    {REG_BIOZ_LOW_TH,   0x00},   /* BIOZ Low Threshold */
    {REG_BIOZ_HIGH_TH,  0xFF},   /* BIOZ High Threshold */
    {REG_BIOZ_CFG7,     0x02},   /* BioZ Configuration 7 */

    /* --- MUX通道配置，MUX4暂时注释，上电阶段不写，等PLL锁定后再配置 --- */
    {REG_MUX1,          0x02},   /* BioZ Mux Configuration 1 */
    {REG_MUX2,          0x00},   /* BioZ Mux Configuration 2 */
    {REG_MUX3,          0xA0},   /* BioZ Mux Configuration 3 */
    // {REG_MUX4,          0xCD},   // 0x44 上电阶段写入失败，延后配置

    /* --- 导联检测 / DC偏置 --- */
    {REG_DC_LEADS,      0x00},   /* DC Leads Configuration */
    {REG_DC_LOFF_TH,    0x00},   /* DC Lead Detect Threshold */
    {REG_LEAD_BIAS,     0x07},   /* Lead Bias Configuration 1 */

    /* --- 中断掩码 --- */
    {REG_INT_EN1,       0x80},   /* Interrupt Enable 1 */
    {REG_INT_EN2,       0x00},   /* Interrupt Enable 2 */
};

/* dump 列表；FIFO_DATA(0x0C) 是破坏性读，故意不放 */
static const uint8_t dump_regs[] = {
	0x00, 0x01, 0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0E,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x17, 0x18, 0x19, 0x1A,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
	0x41, 0x42, 0x43, 0x44, 0x50, 0x51, 0x58, 0x80, 0x81,
	0xFF,
};

/* =============================================================================
 * 5. 传输层
 * =============================================================================
 * nRF 的 TWIM 把 ADDRESS_NACK / DATA_NACK 统一返回 -EIO，所以 -5 只说明
 * "这一笔没被 ACK"，不区分是地址没人应答还是数据被拒绝。
 */

/*
 * 有的 Maxim 器件不接受 repeated START，要求"写寄存器地址 → STOP → 再读"。
 * 先按标准 repeated START 读，失败就退回两段式再试一次，成功就记住这个选择。
 * 这样日志里能直接看出是不是时序问题。
 */
static bool split_read;

static int reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
	int ret;
	if (!split_read) {
		ret = i2c_burst_read_dt(&afe_bus, reg, buf, len);
		if (ret == 0) {
			return 0;
		}
		if (i2c_write_dt(&afe_bus, &reg, 1) == 0 && i2c_read_dt(&afe_bus, buf, len) == 0) 
		{
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

static int reg_write(uint8_t reg, uint8_t value)
{
	uint8_t b[2] = {reg, value};

	return i2c_write_dt(&afe_bus, b, sizeof(b));
}

static bool bus_ready(void)
{
	return i2c_is_ready_dt(&afe_bus);
}

/* =============================================================================
 * 6. 读写封装
 * =============================================================================
 */
static int max30009_write(uint8_t reg, uint8_t value)
{
	int ret = reg_write(reg, value);

	if (ret < 0) {
		printk("MAX30009: write 0x%02X=0x%02X failed (%d)\n", reg, value, ret);
	}
	return ret;
}

/* 写后回读比对：总线上的偶发位错误、只读寄存器都会在这一步暴露 */
static int max30009_write_checked(uint8_t reg, uint8_t value)
{
	uint8_t back = 0;
	int ret;

	ret = reg_write(reg, value);
	if (ret < 0) {
		printk("MAX30009: write 0x%02X=0x%02X failed (%d)\n", reg, value, ret);
		return ret;
	}

	ret = reg_read(reg, &back, 1);
	if (ret < 0) {
		printk("MAX30009: readback 0x%02X failed (%d)\n", reg, ret);
		return ret;
	}
	if (back != value) {
		printk("MAX30009: 0x%02X want 0x%02X got 0x%02X\n", reg, value, back);
		return -EIO;
	}
	return 0;
}

static void max30009_dump(void)
{
	uint8_t v;
	int ret;

	printk("---- register dump ----\n");
	for (size_t n = 0; n < ARRAY_SIZE(dump_regs); n++) {
		ret = reg_read(dump_regs[n], &v, 1);
		if (ret < 0) {
			printk("0x%02X=ERR  ", dump_regs[n]);
		} else {
			printk("0x%02X=0x%02X ", dump_regs[n], v);
		}
		if ((n % 6u) == 5u) {
			printk("\n");
		}
	}
	printk("\n-----------------------\n");
}

/* 只在状态异常时调用，避免刷屏 */
static void max30009_report_status(uint8_t s1, uint8_t s2)
{
	printk("  status:");
	if (s1 & ST1_PWR_RDY) {
		printk(" PWR_RDY");
	}
	if (s1 & ST1_FREQ_LOCK) {
		printk(" FREQ_LOCK");
	}
	if (s1 & ST1_PHASE_LOCK) {
		printk(" PHASE_LOCK");
	}
	if (s1 & ST1_FREQ_UNLOCK) {
		printk(" FREQ_UNLOCK");
	}
	if (s1 & ST1_PHASE_UNLOCK) {
		printk(" PHASE_UNLOCK");
	}
	if (s1 & ST1_A_FULL) {
		printk(" A_FULL");
	}
	if (s2 & ST2_DRV_OOR) {
		printk(" DRV_OOR");
	}
	if (s2 & ST2_BIOZ_OVER) {
		printk(" BIOZ_OVER");
	}
	if (s2 & ST2_BIOZ_UNDR) {
		printk(" BIOZ_UNDR");
	}
	if (s2 & ST2_LON) {
		printk(" LON");
	}
	printk("\n");
}

//ADDR=1
static int try_addresses(int addr_level)
{
	uint8_t id = 0;
	gpio_pin_set(gpio0_dev, ADDR_PIN, addr_level);   /* ADDR=1 → 0x69 */
	k_msleep(2);
	afe_bus.addr = AFE_ADDR;
	if (reg_read(REG_PART_ID, &id, 1) == 0 && id == PART_ID_EXPECTED) 
	{
		printk("MAX30009: 找到！ADDR=%d, 地址 0x%02X, PART_ID=0x%02X\n",
		       addr_level, AFE_ADDR, id);
		return 0;
	}
	printk("MAX30009: 0x%02X 无应答（ADDR=%d）\n", AFE_ADDR, addr_level);
	return -ENODEV;
}

//芯片复位
static int max30009_soft_reset(void)
{
	int ret;
	ret = max30009_write(REG_BIOZ_CFG1, BIOZ1_BG_ONLY);
	if (ret < 0) 
	{
		return ret;
	}
	ret = max30009_write(REG_SYS_CFG1, 0x00); /* SHDN=0 */
	if (ret < 0) 
	{
		return ret;
	}
	ret = max30009_write(REG_PLL4, PLL4_VAL & ~BIT(6)); /* 参考时钟先选内部 */
	if (ret < 0) 
	{
		return ret;
	}
	ret = max30009_write(REG_PLL1, PLL1_STOP); /* PLL 关 */
	if (ret < 0)
	{
		return ret;
	}
	k_msleep(10);
	ret = max30009_write(REG_SYS_CFG1, CFG1_RESET);
	if (ret < 0) 
	{
		return ret;
	}
	k_msleep(5);
	return 0;
}

//初始化
static int max30009_init(void)
{
	uint8_t id = 0;
	uint8_t s1 = 0, s2 = 0;
	int ret;
	ret = max30009_soft_reset();
	if (ret < 0) {
		printk("MAX30009: 软复位失败 (%d)\n", ret);
		return ret;
	}
	ret = reg_read(REG_PART_ID, &id, 1);
	if (ret < 0 || id != PART_ID_EXPECTED) 
	{
		printk("MAX30009: 复位后 PART_ID 异常 (0x%02X, ret=%d)\n", id, ret);
		return (ret < 0) ? ret : -EIO;
	}
	printk("MAX30009: 复位完成，PART_ID=0x%02X\n", id);
	ret = max30009_write_checked(REG_PLL1, PLL1_STOP);
	if (ret < 0) {
		return ret;
	}
	max30009_dump();
	/* 逐条写配置并回读校验；PLL_EN 和通道位留到最后单独开 */
	for (size_t n = 0; n < ARRAY_SIZE(max30009_cfg); n++) {
		ret = max30009_write_checked(max30009_cfg[n].reg, max30009_cfg[n].val);
		if (ret < 0) {
			printk("MAX30009: 配置在第 %u 条失败 (reg 0x%02X)\n",
			       (unsigned int)n, max30009_cfg[n].reg);
			return ret;
		}
	}
	ret = max30009_write_checked(REG_BIOZ_CFG1, BIOZ1_BG_ONLY);
	if (ret < 0) {
		return ret;
	}
	k_msleep(20);   /* 等 bandgap 稳定 */
	ret = max30009_write_checked(REG_PLL1, PLL1_RUN);
	if (ret < 0) {
		return ret;
	}
	k_msleep(200);
	ret = reg_read(REG_STATUS1, &s1, 1);
	if (ret < 0) {
		return ret;
	}
	printk("STATUS1(200ms)=0x%02X\n", s1);
	if ((s1 & (ST1_FREQ_LOCK | ST1_PHASE_LOCK)) !=
	    (ST1_FREQ_LOCK | ST1_PHASE_LOCK)) {
		k_msleep(800);
		ret = reg_read(REG_STATUS1, &s1, 1);
		if (ret < 0) {
			return ret;
		}
		printk("STATUS1(1s)=0x%02X\n", s1);
	}
	if ((s1 & (ST1_FREQ_LOCK | ST1_PHASE_LOCK)) !=
	    (ST1_FREQ_LOCK | ST1_PHASE_LOCK)) {
		printk("MAX30009: PLL 未锁定，STATUS1=0x%02X\n", s1);
		return -ETIMEDOUT;
	}
	printk("MAX30009: PLL locked, STATUS1=0x%02X\n", s1);
	ret = max30009_write(REG_SYSTEM_SYNC, 0x80);   /* 时序系统复位同步 */
	if (ret < 0) {
		return ret;
	}
	k_msleep(5);
	ret = max30009_write_checked(REG_BIOZ_CFG1, BIOZ1_RUN);
	uint8_t v = 0;
	reg_read(REG_BIOZ_CFG1, &v, 1);
	printk("BIOZ_CFG1=0x%02X (expect 0x37)\n", v);
	if (ret < 0) {
		return ret;
	}
	printk("MAX30009: 激励 %u Hz，采样 %u sps\n",
	       (REF_CLK_HZ * PLL_MULTIPLIER) / ((1u << PLL_DIV_EXP_K) * 32u),
	       (REF_CLK_HZ * PLL_MULTIPLIER) /
		       ((512u << PLL_NDIV_ENABLE) * (8u << BIOZ_ADC_OSR_CODE)));
	printk("MAX30009: STATUS1=0x%02X STATUS2=0x%02X\n", s1, s2);

	return 0;
}

//读FIFO数据块
struct bioz_block {
	int32_t i_sum;
	int32_t q_sum;
	uint32_t i_cnt;
	uint32_t q_cnt;
	uint32_t other_cnt;
	uint8_t status1;
	uint8_t status2;
	uint8_t fifo_ovf;
};

static uint8_t fifo_raw[FIFO_READ_MAX_WORDS * FIFO_BYTES_PER_SAMPLE];

static int max30009_read(struct bioz_block *out)
{
	uint8_t cnt1 = 0, cnt2 = 0;
	uint16_t words, n;
	int ret;
	memset(out, 0, sizeof(*out));
	ret = reg_read(REG_FIFO_CNT1, &cnt1, 1);
	if (ret < 0) {
		return ret;
	}
	ret = reg_read(REG_FIFO_CNT2, &cnt2, 1);
	if (ret < 0) {
		return ret;
	}
	out->fifo_ovf = cnt1 & 0x7Fu;
	words = (uint16_t)(((uint16_t)(cnt1 & 0x80u) << 1) | cnt2);
	if (words == 0u) {
		return -EAGAIN;
	}
	if (words > FIFO_MAX_WORDS) {
		return -EOVERFLOW;
	}
	n = MIN(words, FIFO_READ_MAX_WORDS);
	ret = reg_read(REG_FIFO_DATA, fifo_raw, (size_t)n * FIFO_BYTES_PER_SAMPLE);
	if (ret < 0) {
		return ret;
	}
	for (uint16_t k = 0; k < n; k++) {
		const uint8_t *p = &fifo_raw[k * FIFO_BYTES_PER_SAMPLE];
		uint32_t frame = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
		uint8_t tag = (uint8_t)(frame >> FIFO_TAG_SHIFT);
		int32_t value = (int32_t)(frame & FIFO_DATA_MASK);
		if (value & BIT(FIFO_DATA_SIGN_BIT)) { /* 20 位符号扩展 */
			value |= (int32_t)0xFFF00000u;
		}
		if (tag == FIFO_TAG_I) {
			out->i_sum += value;
			out->i_cnt++;
		} else if (tag == FIFO_TAG_Q) {
			out->q_sum += value;
			out->q_cnt++;
		} else {
			out->other_cnt++;
		}
	}
	ret = reg_read(REG_STATUS1, &out->status1, 1);
	if (ret < 0) {
		return ret;
	}
	return reg_read(REG_STATUS2, &out->status2, 1);
}

/* 64 位整数开方，给 |Z| 用，避免 link libm */
static uint32_t isqrt64(uint64_t v)
{
	uint64_t rem = v;
	uint64_t root = 0;
	uint64_t bit = 1ULL << 62;

	while (bit > rem) {
		bit >>= 2;
	}
	while (bit != 0u) {
		if (rem >= root + bit) {
			rem -= root + bit;
			root = (root >> 1) + bit;
		} else {
			root >>= 1;
		}
		bit >>= 2;
	}
	return (uint32_t)root;
}

//主函数
int main(void)
{
	gpio_pin_configure(gpio0_dev, CSB_PIN, GPIO_OUTPUT_HIGH);
	gpio_pin_configure(gpio0_dev, ADDR_PIN, GPIO_OUTPUT_HIGH);
	gpio_pin_configure(gpio1_dev, LED_PIN, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set(gpio1_dev, LED_PIN, 1);
	struct bioz_block block;
	uint32_t ok = 0, empty = 0, err = 0;
	int ret;
	printk("\n*** MAX30009 BioZ 测试（I2C）***\n");
	try_addresses(1);
	ret = max30009_init();
	if (ret < 0) 
	{
		printk("MAX30009: init failed (%d)\n", ret);
		while (1) {
			k_sleep(K_FOREVER);
		}
	}

	while (1) {
		ret = max30009_read(&block);

		if (ret == -EAGAIN) {
			empty++;
		} else if (ret < 0) {
			err++;
			printk("MAX30009: read failed (%d)\n", ret);
		} else {
			int32_t i_mean = block.i_cnt ? (block.i_sum / (int32_t)block.i_cnt) : 0;
			int32_t q_mean = block.q_cnt ? (block.q_sum / (int32_t)block.q_cnt) : 0;
			uint64_t mag2 = (uint64_t)((int64_t)i_mean * i_mean) +
					(uint64_t)((int64_t)q_mean * q_mean);

			ok++;
			printk("I=%8d Q=%8d | |Z|=%7u LSB | I:%3u Q:%3u ovf:%u\n",
			       i_mean, q_mean, isqrt64(mag2), block.i_cnt, block.q_cnt,
			       block.fifo_ovf);

			if (block.other_cnt) {
				printk("  注意：%u 个未知 TAG 帧（可能帧对齐错位）\n",
				       block.other_cnt);
			}
			if (block.status2 & (ST2_DRV_OOR | ST2_BIOZ_OVER | ST2_BIOZ_UNDR)) {
				max30009_report_status(block.status1, block.status2);
			}
		}

		/* 每 10 次统计一次，区分"没数据"和"数据错" */
		if (((ok + empty + err) % 10u) == 0u) {
			printk("[stats] ok=%u empty=%u err=%u\n", ok, empty, err);
			gpio_pin_toggle(gpio1_dev, LED_PIN);
		}

		k_msleep(100);
	}
}