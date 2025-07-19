# XV7081BB SPI Driver for Embedded Platforms

This repository provides a lightweight C driver for the [EPSON XV7081BB](https://download.epsondevice.com/td/pdf/app/XV7081BB_en.pdf) high-performance gyroscope, using the **SPI interface only**.  
It includes platform-independent source files and a working example for **ESP32 (ESP-IDF)**.

> ⚠️ Only SPI **4-wire mode** is supported.

---

## 📁 Repository Structure

```
xv7081bb-driver/
├── src/                        # Driver source code
│   ├── xv7081bb.c
│   └── xv7081bb.h
├── examples/                  # Usage examples
│   └── xv7081bb_example_esp32.c
├── docs/
│   └── XV7081BB_datasheet.pdf
├── README.md
└── LICENSE
```

---

## 🧩 Features

- Supports 16-bit and 24-bit output resolutions
- Configurable low-pass and high-pass filters
- Configurable output data rate (ODR)
- On-chip temperature readout
- Portable to any embedded platform (STM32, ESP32, nRF52, etc.)

---

## 🚀 Getting Started

### 1. Add to your project

Copy the contents of the `src/` directory into your project and include `xv7081bb.h`.

### 2. Implement platform-specific functions

You must provide three hardware-dependent functions:

```c
int32_t platform_spi_write(uint8_t reg, const uint8_t *buf, uint16_t len);
int32_t platform_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);
void platform_delay(uint32_t ms);
```

These are used by the driver through the `xv7081bb_ctx_t` context struct.

> ✅ A complete example for ESP32 is available under `examples/xv7081bb_example_esp32.c`.

---

## 📎 Notes

- Only SPI **4-wire mode** is supported.
- To use SPI mode, you must first **disable I²C** by calling xv7081bb_disable_i2c().
- The raw output data rate (fs) depends on the chip variant:  
  - **13.770 kHz** for H/L versions  
  - **14.160 kHz** for J version  
  You can identify your device's variant using the "Marking Description" section in the datasheet.
  Use the `ODR_DIV_n` macros to scale this down to your desired output rate.

---

## 📜 License

MIT License. See [LICENSE](LICENSE) file for details.

> Developed by Álvaro Oliver, 2025  
> Use at your own risk. Contributions welcome!
