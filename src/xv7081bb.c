// xv7081bb.c - Epson XV7081BB SPI Driver

#include "xv7081bb.h"

int xv7081bb_disable_i2c(xv7081bb_ctx_t *ctx) {
    uint8_t zero = 0x00;
    return ctx->write(REG_I2C_DISABLE, &zero, 1);
}

int xv7081bb_set_resolution(xv7081bb_ctx_t *ctx, xv7081bb_resolution_t res) {
    ctx->resolution = res;
    return 0;
}

int xv7081bb_configure_hpf(xv7081bb_ctx_t *ctx, uint8_t hpf_config)
{
    uint8_t reg;
    if (ctx->read(REG_DSP_SETTINGS_1, &reg, 1) != 0)
        return -1;

    // Clean bits: HPF_Fc[2:0] (6:4), EnableHpf (1), Reserved bits 2 and 3
    reg &= ~((0b111 << 4) | (1 << 1) | (1 << 2) | (1 << 3));

    // Apply HPF config (bits 6:4 and 1)
    reg |= hpf_config;

    // Force to default values in reserved bits
    reg &= ~(1 << 7);  // Bit 7 = 0
    reg |=  (1 << 0);  // Bit 0 = 1

    if (ctx->write(REG_DSP_SETTINGS_1, &reg, 1) != 0)
        return -1;

    return 0;
}

int xv7081bb_configure_lpf(xv7081bb_ctx_t *ctx, uint8_t lpf_order, uint8_t lpf_fc)
{
    if (!ctx || !ctx->write) return -1;
    uint8_t reg_val = lpf_order | (lpf_fc & 0x0F);
    return ctx->write(REG_DSP_SETTINGS_2, &reg_val, 1);
}

int xv7081bb_set_sample_rate(xv7081bb_ctx_t *ctx, uint8_t self_fs_bits) {
    if (self_fs_bits > 0x07) return -1;
    uint8_t value = self_fs_bits & 0x07;
    return ctx->write(REG_DSP_SETTINGS_3, &value, 1);
}

int xv7081bb_read_dps_xyz(xv7081bb_ctx_t *ctx, float *x_dps, float *y_dps, float *z_dps) {
    uint8_t buf[9] = {0};
    uint8_t bytes_per_axis = (ctx->resolution == XV7081BB_RES_24BITS) ? 3 : 2;
    uint8_t total_bytes = 3 * bytes_per_axis;

    // Although section 6.5 "Angular Rate Data Read" of the datasheet suggests multi-byte reads,
    // each byte must be read using separate SPI transactions to retrieve valid data reliably.
    for (int i = 0; i < total_bytes; i++) {
        if (ctx->read(REG_RATE_READ, &buf[i], 1) != 0)
            return -1;
    }

    float scale = (ctx->resolution == XV7081BB_RES_24BITS) ? (1.0f / 17920.0f) : (1.0f / 70.0f);

    if (ctx->resolution == XV7081BB_RES_24BITS) {
        int32_t x_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
        int32_t y_raw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | buf[5];
        int32_t z_raw = ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 8) | buf[8];
        if (x_raw & 0x800000) x_raw |= 0xFF000000; // sign extend
        if (y_raw & 0x800000) y_raw |= 0xFF000000;
        if (z_raw & 0x800000) z_raw |= 0xFF000000;
        *x_dps = x_raw * scale;
        *y_dps = y_raw * scale;
        *z_dps = z_raw * scale;
    } else {
        int16_t x_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t y_raw = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t z_raw = (int16_t)((buf[4] << 8) | buf[5]);
        *x_dps = x_raw * scale;
        *y_dps = y_raw * scale;
        *z_dps = z_raw * scale;
    }

    return 0;
}

int xv7081bb_read_temperature(xv7081bb_ctx_t *ctx, float *deg_c) {
    uint8_t buf[2];
    if (ctx->read(REG_TEMP_READ, buf, 2) != 0) return -1;

    // only 12 bits format is supported: D[11:4] in buf[0], D[3:0] in buf[1]
    int16_t raw = ((int16_t)buf[0] << 4) | (buf[1] >> 4);
    if (raw & 0x0800) raw |= 0xF000; // sign extension if negative

    // 400 = 25 °C, 16 LSB/°C
    *deg_c = ((float)raw - 400.0f) / 16.0f + 25.0f;
    return 0;
}

