<div align="center">

<img src="docs/image.png" alt="ameba-ui graphics middleware" width="800">

# ameba-ui

**The official graphics & display middleware for Realtek Ameba series chips — LVGL, panels, touch and imaging.**

[![LVGL](https://badgen.net/badge/LVGL/9.3/blue)](https://lvgl.io)
[![Language](https://badgen.net/badge/language/C/blue)](https://github.com/Ameba-AIoT/ameba-ui/search?l=c)
[![Based on](https://badgen.net/badge/based%20on/ameba-rtos/blue)](https://github.com/Ameba-AIoT/ameba-rtos)
[![Last Commit](https://badgen.net/github/last-commit/Ameba-AIoT/ameba-ui/master?icon=github)](https://github.com/Ameba-AIoT/ameba-ui/commits/master)
[![Gitee Mirror](https://badgen.net/badge/mirror/Gitee/c71d23?icon=git)](https://gitee.com/ameba-aiot/ameba-ui)

[English](README.md) · [中文版](README_CN.md) · [Documentation](https://aiot.realmcu.com/en/latest/rtos/index.html) · [ameba-rtos](https://github.com/Ameba-AIoT/ameba-rtos)

</div>

ameba-ui is the official graphics and display middleware for Realtek Ameba series SoCs. It bundles the LVGL graphics library, a display abstraction layer with a rich set of panel drivers, a touch input subsystem, and imaging/font libraries. It is consumed as the `component/ui` submodule of [ameba-rtos](https://github.com/Ameba-AIoT/ameba-rtos).

## 🔌 Supported Chips

| Chip                   | Interfaces           |         master         |      release/v1.2      |
|:---------------------- |:-------------------- |:----------------------:|:----------------------:|
| RTL8721F| MIPI-DSI / RGB / SPI | ![alt text][supported] | ![alt text][supported] |
| RTL8730E| MIPI-DSI / RGB / SPI | ![alt text][supported] | ![alt text][supported] |

[supported]: https://img.shields.io/badge/-supported-green "supported"

> LVGL itself is portable and can run on other Ameba chips; the bundled display and touch drivers are validated on AmebaGreen2 and AmebaSmart.

## ✨ Key Features

Built around LVGL v9.3 with a ready-made Ameba HAL port, ameba-ui works out of the box. It provides a unified display abstraction layer and panel manager covering MIPI-DSI, RGB and SPI interfaces, with bundled drivers for a range of mainstream panel controllers and capacitive touch controllers, all wired through a single input manager. It also integrates common third-party libraries — image decoding (JPEG / PNG / QOI), FreeType font rasterization, and a lightweight embedded flash database — that can be enabled as needed in menuconfig.

## 📚 Documentation

For the latest documentation, see the [FreeRTOS SDK and User Guide](https://aiot.realmcu.com/en/latest/rtos/index.html).

For more information on the Ameba series chips, visit the [product page](https://aiot.realmcu.com/en/product/index.html).

## 📦 Usage

ameba-ui is used as the `component/ui` submodule of ameba-rtos rather than built standalone.

**1. Get ameba-rtos with submodules (XDK)**

```bash
git clone --recurse-submodules https://github.com/Ameba-AIoT/ameba-rtos.git
```

If you already cloned ameba-rtos without submodules:

```bash
git submodule update --init --recursive component/ui
```

**2. Set up the SDK environment**

```bash
source env.sh   # Linux
env.bat         # Windows
```

**3. Enable the UI in Kconfig**

```bash
ameba.py soc <soc_name>   # e.g. RTL8721F, RTL8730E
ameba.py menuconfig
```

In `menuconfig`, open **Graphics Libraries Configuration** and:

- Enable **Graphics UI**
- **Panel Selection** — choose the panel matching your hardware
- **Touch Controller Selection** — enable touch and choose the controller (CST328 / GT911 / TDDI)
- **Third Party Libraries** — enable image/format libraries as needed (libjpeg-turbo, libpng, QOI, zlib)

**4. Build**

```bash
ameba.py build
```

## 🌐 Accelerate with Gitee

For users who can access [Gitee](https://gitee.com), a mirror is available at [ameba-ui](https://gitee.com/ameba-aiot/ameba-ui) to improve download speed if GitHub is slow.

## 💬 Feedback

* For questions or suggestions during development, visit the [Ameba AIoT Forum](https://forum.amebaiot.com/).
* For bugs or feature requests, [check the GitHub Issues](https://github.com/Ameba-AIoT/ameba-ui/issues). Please check existing issues before opening a new one.
