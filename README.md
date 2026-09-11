# CAN_GAUGE

# 🏁 CAN Fuel Level Gauge

**A tiny, track-car-ready fuel level display you mount to the inside of your window.**

Built around an ESP32-S3 and a 1.85" round LCD, this board reads a fuel level sensor (0–5 V analog, NTC, or a sensor already on the CAN bus) and shows it instantly — no more guessing, no more shaking the car to hear how much is left in the tank.

> Designed and built from scratch by **NR Developments** for a BMW E36 race car.

---

## 🎯 Why this exists

On a lot of race cars — this BMW E36 included — the fuel tank isn't where you'd expect it. Here it lives in the boot, fitted with a foam baffle to stop fuel surge under cornering and braking. That foam is great for safety, but it makes it almost impossible to tell how much fuel is actually in there just by looking, and constantly getting in and out of the car to check is a pain — especially between sessions or during a pit stop.

The plan: run a line from the tank up to the rear window, mount this board **on the inside** of the glass, and read the fuel level straight off the display **from outside the car**. Walk up, glance at the window, know exactly how much fuel you have. No opening the boot, no removing panels, no more spongy uncertainty.

---

## ✨ Features

- **ESP32-S3** brain — Wi-Fi + BLE
- **1.85" round display** — clean, glanceable, and it disappears nicely behind tinted glass
- **Flexible sensor input** — works with:
  - a **0–5 V analog sensor** (e.g. a resistive fuel level sender), scaled and conditioned on-board for the ESP32's 3.3 V ADC
  - an **NTC thermistor** or similar, by swapping a couple of resistors around the input op-amp
  - a **CAN-connected sensor**, where this board just listens on the bus and displays whatever value comes in
- **CAN bus (500 kb)** — send your local sensor reading out to the rest of the car, or receive a reading from elsewhere and display it. Switchable 120 Ω termination resistor on board.
- **Automotive-grade power input** — takes 12 V straight from the car, with reverse-polarity protection, a TVS clamp, and a buck + LDO stage (5 V then 3.3 V) so it's happy living on a noisy vehicle electrical system
- **Ignition-aware** — a dedicated ignition (switched 12 V) input so the board can wake up or go quiet with the rest of the car's electronics
- **USB-C** — for programming, debugging, and power on the bench, with auto-reset/auto-bootloader circuitry so flashing is a one-click affair (no button-mashing)
- **OTA-friendly** — reflash over Wi-Fi once it's mounted and wired in, no need to pull it back out
- **Single automotive connector** — power, ignition, CAN H/L, sensor supply and sensor signal all come in on one Molex connector for a tidy, serviceable install

---

## 🧠 How it works

```
                 ┌──────────────────────────┐
  12V + IGN ───▶ │   Power (buck + LDO)      │ ──▶ 5V / 3.3V rails
                 └──────────────────────────┘
                              │
 Sensor (0-5V/NTC) ─▶ [ signal conditioning ] ─▶ ┌───────────────┐
                                                   │   ESP32-S3     │ ──▶ 1.85" round display
 CAN H/L ─────────▶ [ CAN transceiver, 500kb ] ─▶ │                │
                                                   └───────────────┘
                                                        │      │
                                                     USB-C    Wi-Fi
                                                   (program)  (OTA / future features)
```

The board can work two ways, depending on how you wire your sensor:

1. **Local sensor mode** — plug a sensor directly into the board's sensor input, read it locally, show it on the display, and optionally broadcast the value on the CAN bus for other devices (loggers, dashboards, etc.) to use.
2. **CAN listener mode** — leave the sensor input unused and instead have the sensor connected elsewhere on the car's CAN bus. This board just subscribes to the relevant message and displays it.

---

## 🔌 Hardware overview

| Block | Details |
|---|---|
| **MCU** | ESP32-S3-WROOM-1-N |
| **Display** | 1.85" round LCD, SPI interface (SCLK / MOSI / MISO / DC / RST / CS + backlight control) |
| **Power in** | 12 V automotive, reverse-polarity diode + TVS protection |
| **Regulation** | 12 V → 5 V buck converter (0.6 A max) → 3.3 V LDO (0.5 A max) |
| **CAN** | High-speed CAN transceiver, 500 kb/s, ESD-protected, switchable 120 Ω termination |
| **Sensor input** | Op-amp based conditioning stage; scales a 0–5 V signal down to the 3.3 V domain |
| **USB** | USB-C, used for programming/debug; includes auto-reset circuitry and USB ESD protection |
| **External I/O** | Single Molex connector: +12 V, GND, Ignition, CAN H, CAN L, Sensor 5 V, Sensor signal |
| **Test points** | UART TX/RX/DTR/RTS/GND broken out for easy debugging |
| **Status LEDs** | Power and status indicators on board |

### 🧪 Sensor options

The board ships configured for a **0–5 V analog sensor** by default (e.g. a typical resistive fuel level sender), which gets scaled down to a safe 3.3 V range for the ESP32's ADC.

Want to use an **NTC thermistor** (or a similar resistive sensor) instead? Just remove the two resistors around the input op-amp stage that set up the 0–5 V scaling — the input then behaves as a simple resistive divider suitable for an NTC. See the schematic (`Sensor.SchDoc`) for exactly which components to pull.

### 🔗 Connector pinout

The board connects to the car through a single 8-way Molex connector:

| Pin | Signal | Notes |
|---|---|---|
| 1 | Sensor signal | Local analog sensor input |
| 2 | Sensor 5 V | Supply for an external sensor |
| 3 | GND | |
| 4 | Ignition | Switched 12 V (key-on) |
| 5 | CAN L | |
| 6 | CAN H | |
| 7 | +12 V | Permanent battery feed |

---

## 🖥️ Programming

Two ways to flash and update firmware:

- **USB-C** — plug in, hit upload. Auto-reset circuitry handles entering bootloader mode automatically, so you don't need to hold any buttons.
- **Wi-Fi (OTA)** — once the board is mounted and wired into the car, push firmware updates over the air without pulling it back out.

Firmware source, build instructions, and flashing tools live in [`/firmware`](./firmware) *(coming soon)*.

---

## 🛠️ Repository structure

```
.
├── hardware/          # Schematics, PCB layout, BOM, Gerbers
│   ├── CAN_PDM_SCH_PCB.PDF
│   └── ...
├── firmware/           # ESP32-S3 firmware (coming soon)
└── README.md
```

---

## 📋 Status

This is a first-revision board, built and tested on a real race car. Expect iteration as real-world track use turns up things to improve — issues and pull requests are welcome.

- [x] Power design (12 V in, reverse-polarity + TVS protected, 5 V / 3.3 V rails)
- [x] CAN bus interface with switchable termination
- [x] Analog sensor front-end (0–5 V, NTC-adaptable)
- [x] ESP32-S3 core with USB-C programming + auto-reset
- [x] Round 1.85" display support
- [ ] Firmware / display UI
- [ ] Wi-Fi OTA workflow
- [ ] Enclosure / window-mount hardware

---

## 📷 Gallery

coming soon

---

## 🤝 Contributing

Found a bug, have an improvement, or built your own version for a different car? Issues and pull requests are welcome.


## 📬 Contact

Built by **NR Developments**. Questions, ideas, or just want to talk race-car electronics? Open an issue or reach out.
