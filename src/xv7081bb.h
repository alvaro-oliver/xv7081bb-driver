// xv7081bb.h - Epson XV7081BB Headers
#ifndef XV7081BB_H
#define XV7081BB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Platform dependant functions, should be defined in main.c file.
typedef int32_t (*xv7081bb_spi_write_ptr)(uint8_t reg, const uint8_t *buf, uint16_t len);
typedef int32_t (*xv7081bb_spi_read_ptr)(uint8_t reg, uint8_t *buf, uint16_t len);
typedef void (*xv7081bb_delay_ms_ptr)(uint32_t ms);

typedef enum {
    XV7081BB_RES_16BITS = 0,
    XV7081BB_RES_24BITS = 1
} xv7081bb_resolution_t;

typedef struct {
    xv7081bb_spi_write_ptr write;
    xv7081bb_spi_read_ptr read;
    xv7081bb_delay_ms_ptr delay_ms;
    xv7081bb_resolution_t resolution;
} xv7081bb_ctx_t;

// Registers address
#define REG_I2C_DISABLE     0x1F
#define REG_RATE_READ     0x0A
#define REG_TEMP_READ     0x08
#define REG_DSP_SETTINGS_1  0x01
#define REG_DSP_SETTINGS_2  0x02
#define REG_DSP_SETTINGS_3  0x03

// See: Table 7.2. DSP Settings 1
#define HPF_DISABLE 0x00 // disables HPF (set EnableHpf bit to LOW)
#define HPF_0_01Hz  ((0b000 << 4) | (1 << 1))
#define HPF_0_03Hz  ((0b001 << 4) | (1 << 1))
#define HPF_0_1Hz   ((0b010 << 4) | (1 << 1))
#define HPF_0_3Hz   ((0b011 << 4) | (1 << 1))
#define HPF_1Hz     ((0b100 << 4) | (1 << 1))
#define HPF_3Hz     ((0b101 << 4) | (1 << 1))
#define HPF_10Hz    ((0b110 << 4) | (1 << 1))

// See: Table 7.3. DSP Settings 2
// LPF Order (bits 5:4)
#define XV7081BB_LPF_ORDER_2ND  (0x00)
#define XV7081BB_LPF_ORDER_3RD  (0x10)
#define XV7081BB_LPF_ORDER_4TH  (0x20)
// LPF Fc (bits 3:0)
#define XV7081BB_LPF_FC_10HZ     0x00
#define XV7081BB_LPF_FC_35HZ     0x01
#define XV7081BB_LPF_FC_45HZ     0x02
#define XV7081BB_LPF_FC_50HZ     0x03
#define XV7081BB_LPF_FC_70HZ     0x04
#define XV7081BB_LPF_FC_85HZ     0x05
#define XV7081BB_LPF_FC_100HZ    0x06
#define XV7081BB_LPF_FC_140HZ    0x07
#define XV7081BB_LPF_FC_175HZ    0x08
#define XV7081BB_LPF_FC_200HZ    0x09
#define XV7081BB_LPF_FC_285HZ    0x0A
#define XV7081BB_LPF_FC_345HZ    0x0B
#define XV7081BB_LPF_FC_400HZ    0x0C
#define XV7081BB_LPF_FC_500HZ    0x0D

// See: Table 7.4. DSP Settings 3
#define XV7081BB_ODR_DIV_1     0x00  // fs
#define XV7081BB_ODR_DIV_2     0x01  // fs/2
#define XV7081BB_ODR_DIV_4     0x02  // fs/4
#define XV7081BB_ODR_DIV_8     0x03  // fs/8
#define XV7081BB_ODR_DIV_16    0x04  // fs/16
#define XV7081BB_ODR_DIV_32    0x05  // fs/32
#define XV7081BB_ODR_DIV_64    0x06  // fs/64
#define XV7081BB_ODR_DIV_128   0x07  // fs/128

int xv7081bb_disable_i2c(xv7081bb_ctx_t *ctx);
int xv7081bb_set_resolution(xv7081bb_ctx_t *ctx, xv7081bb_resolution_t res);
int xv7081bb_configure_hpf(xv7081bb_ctx_t *ctx, uint8_t hpf_bits);
int xv7081bb_configure_lpf(xv7081bb_ctx_t *ctx, uint8_t lpf_order, uint8_t lpf_fc);
int xv7081bb_set_sample_rate(xv7081bb_ctx_t *ctx, uint8_t self_fs_bits);
int xv7081bb_read_dps_xyz(xv7081bb_ctx_t *ctx, float *x_dps, float *y_dps, float *z_dps);
int xv7081bb_read_temperature(xv7081bb_ctx_t *ctx, float *deg_c);

#ifdef __cplusplus
}
#endif

#endif // XV7081BB_H