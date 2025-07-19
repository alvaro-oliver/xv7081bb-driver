/** xv7081bb_example_esp32.c
 * 
 * XV7081BB gyroscope driver example for ESP32 (ESP-IDF).
 * 
 * To port this code to another platform, simply reimplement the following three functions:
 *   - platform_spi_write()
 *   - platform_spi_read()
 *   - platform_delay()
 * 
 * Álvaro Oliver, 2025
 * This is free software, provided as-is with no warranty.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "xv7081bb.h"

#define SPI_DMA_CHAN     1
#define SPI_HOST_USED    HSPI_HOST
#define SPI_CLOCK_HZ     2000000

#define GPIO_MOSI        23
#define GPIO_MISO        19
#define GPIO_SCLK        18
#define GPIO_CS           5

static spi_device_handle_t spi;
static xv7081bb_ctx_t ctx; // XV7081BB context

/**
 * @brief  Platform-specific SPI write function
 *         This function should be reimplemented for your target MCU.
 */
int32_t platform_spi_write(uint8_t reg, const uint8_t *buf, uint16_t len) {
    uint8_t tx[len + 1];
    tx[0] = reg & 0x7F;  // Bit 7 = 0 for write
    memcpy(&tx[1], buf, len);

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = NULL
    };
    return spi_device_transmit(spi, &t) == ESP_OK ? 0 : -1;
}

/**
 * @brief  Platform-specific SPI read function
 *         This function should be reimplemented for your target MCU.
 */
int32_t platform_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0x00, sizeof(tx));
    tx[0] = reg | 0x80;  // Bit 7 = 1 for read

    spi_transaction_t t = {
        .length = 8 * (len + 1),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) return -1;
    memcpy(buf, &rx[1], len);  // Skip dummy byte
    return 0;
}

/**
 * @brief  Platform-specific delay in milliseconds
 *         Replace this with your platform's delay function.
 */
void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void app_main(void) {

    // --- Initialize SPI bus and device ---
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 3,  // SPI Mode 3 (CPOL=1, CPHA=1)
        .spics_io_num = GPIO_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CHAN));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi));
    platform_delay(20);

    // --- Initialize XV7081BB context with platform functions ---
    ctx.write = platform_spi_write;
    ctx.read = platform_spi_read;
    ctx.delay_ms = platform_delay;
    ctx.resolution = XV7081BB_RES_16BITS;  // Default resolution

    // --- Device configuration sequence ---
    xv7081bb_disable_i2c(&ctx);  // Required to enable SPI communication
    platform_delay(10);

    xv7081bb_set_resolution(&ctx, XV7081BB_RES_24BITS);  // Optional: switch to 24-bit resolution

    xv7081bb_configure_hpf(&ctx, HPF_DISABLE);  // Set high-pass filter frequency (optional), disabled by default
    xv7081bb_configure_lpf(&ctx, XV7081BB_LPF_ORDER_2ND, XV7081BB_LPF_FC_400HZ);  // Set LPF: 2nd order, 400 Hz. Default values are 2nd order, 100 Hz

    /**
     * Sample Rate configuration:
     *   The internal ADC runs at 13.770 kHz (H/L variants) or 14.160 kHz (J variant).
     *   You can identify your device's variant using the "Marking Description" section in the datasheet.
     */
    xv7081bb_set_sample_rate(&ctx, XV7081BB_ODR_DIV_32);  // ~430 Hz for H/L variant

    // --- Main data acquisition loop ---
    while (1) {
        float x, y, z, temp;

        xv7081bb_read_dps_xyz(&ctx, &x, &y, &z);     // Read angular velocity (°/s)
        xv7081bb_read_temperature(&ctx, &temp);      // Read internal temperature (°C)

        printf("%.4f, %.4f, %.4f, %.2f\n", x, y, z, temp); 
        platform_delay(100);  // 100 ms delay between reads
    }
}
