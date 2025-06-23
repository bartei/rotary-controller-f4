# Rotary Controller (F4)

[![Discord](https://img.shields.io/discord/1386014070632878100?style=social)](https://discord.gg/69Qr5PSu) [![Shop at Provvedo](https://img.shields.io/badge/Shop-Provvedo-blue?logo=shopify&style=flat-square)](https://www.provvedo.com/shop)


This repository contains the **firmware** for a rotary controller board based on the **STM32F411** microcontroller ([github.com][1]). It provides Digital Read Out (DRO) and single-axis control for CNC-style rotary tables.

🛒 **Purchase all boards from our shop:** [Provvedo Shop](https://www.provvedo.com/shop)

---

## ⚙️ Features

* Utilizes **STM32CubeMX** for hardware configuration (.ioc file included)
* Modular firmware structure with FreeRTOS support
* Supports ST‑Link V2 and Raspberry Pi + OpenOCD programming
* Optimized for high-speed encoder + stepper motor control

---

## 🛠️ Build & Flash

### Requirements

* CMake & C/C++ toolchain (e.g. `arm-none-eabi-gcc`, `make`)
* ST-Link v2 or Raspberry Pi with OpenOCD

### Build

```bash
git clone https://github.com/bartei/rotary-controller-f4.git
cd rotary-controller-f4
cmake -DCMAKE_BUILD_TYPE=Release .
make -j$(nproc)
```

### Clean

```bash
make clean
```

### Flash

* **ST‑Link V2**:

  ```bash
  st-flash --format ihex write rotary-controller-f4.hex
  ```

* **Raspberry Pi + OpenOCD**:

  ```bash
  openocd -f ./raspberry.cfg
  ```

  The default `raspberry.cfg` configures SWD over GPIO pins 24/25 + GND. Ensure GND wiring is the **same length** as SWCLK/SWDIO for reliability. Modify the GPIO pins in `raspberry.cfg` if needed.

---

## 🔧 Hardware Configuration

* `.ioc` file for use with STM32CubeMX included
* Pin assignments for encoder, buttons, LEDs, SWD, etc. reviewed and tested
* Memory layout defined by `STM32F411CEUX_FLASH.ld` and `STM32F411CEUX_RAM.ld`

---

## 🧩 PCB & Schematic

Firmware integrates with hardware design available at:

* **PCB repo**: bartei/rotary-controller-pcb — includes Proteus schematic, BOM (with pricing), and fab files; KiCad version in progress ([github.com][2], [github.com][3], [github.com][1])

Together, they form a complete controller + UI system when paired with:

* `rotary-controller-python` — a Raspberry Pi Kivy-based DRO + control UI

---

## 🛎️ Usage Notes

* Works as a **single-axis rotary DRO**
* FreeRTOS scheduler handles encoder sampling loop
* GPIO/button routines support nudge and rotary button functions
* SWD pins must be appropriately wired and matched in length

---

## 📘 Resources & Links

* [Firmware repo](https://github.com/bartei/rotary-controller-f4)
* [PCB repo (Proteus/KiCad)](https://github.com/bartei/rotary-controller-pcb)
* [Raspberry Pi UI with Kivy](https://github.com/bartei/rotary-controller-python)
* Join the community on **Discord**

---

## ✅ Next Steps

1. Test hardware interface in CubeMX; verify pin assignments
2. Build and flash firmware, connect to DRO UI app
3. Utilize FreeRTOS for real-time sampling and control
4. Contribute improvements — e.g. KiCad support, UI features, multi-axis

---

## 📝 Contact & Support

Need help? Join our **Discord** community for support, discussions, and updates.


---

Let me know if you'd like additions like block diagrams, pinout tables, or usage screenshots!

[1]: https://github.com/bartei/rotary-controller-f4"
[2]: https://github.com/bartei/rotary-controller-pcb"
[3]: https://github.com/bartei/rotary-controller-python"
