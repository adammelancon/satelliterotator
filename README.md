 ![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)    ![Protocol](https://img.shields.io/badge/protocol-GS--232-green)    ![Gpredict](https://img.shields.io/badge/compatible-Gpredict-brightgreen)    ![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg) 


# Gpredict GS-232 Satellite Rotator Controller

This repository contains two complementary components for a DIY satellite tracking rotator system:

* Arduino-based firmware for controlling a pan/tilt head using the GS-232 protocol.
* A Python-based proxy server that listens for rotor commands from applications like Gpredict and forwards them to the microcontroller using the GS-232 protocol on a serial connection.

<img src="https://raw.githubusercontent.com/adammelancon/satelliterotator/refs/heads/main/images/IMG_9037.jpeg" width="300">


---

## 📁 Directory Overview

### `gs232rotator/`

* Platform: ESP32-S3 (Arduino)
* Implements a subset of the [GS-232 rotor control protocol](file:///C:/Users/adamm/Downloads/GS232A.pdf "GS-232 rotor control protocol")
* Controls relays for azimuth and elevation motion
* I am using a BNO055 IMU sensor for compass bearing (AZ) and tilt (EL)

### `gpredictpythonrotatorproxy/`

* Lightweight Python 3 socket server
* Listens on TCP port 7777
* Emulates a subset of the `rotctld` (Hamlib) commands
* Communicates with the ESP32-S3 controller via serial to control the motors via GS-232

---

## ⚙️ Setup Instructions

1. **Flash the firmware** in `gs232rotator/` to your ESP32-S3 using Arduino IDE.
2. **Run the Python proxy** on your computer:
   
   The included `run.bat` script handles dependencies automatically using `uv`. Just double-click it or run:

   ```bash
   gpredictpythonrotatorproxy\run.bat
   ```
   
   *(If `uv` is not installed, the script will offer to install it for you).*
3. **Configure Gpredict**:
   * Name `RS232Proxy`
   * Host: `127.0.0.1`
   * Port:`7777`
   * Az Type: `0 > 180 > 360`

---
## 🧭 Sensor Calibration

The BNO055 sensor must be calibrated to ensure accurate azimuth (AZ) and elevation (EL) readings.

On first boot, if no calibration offsets are found in EEPROM, the system will automatically enter calibration mode and display calibration progress on the OLED screen. Once calibration is complete, offsets are saved to EEPROM.

You can manually re-run calibration anytime by sending the `CALIBRATE` command over serial.

### 🛠 Calibration Procedure

1. Power on the system. If no offsets are stored, it will enter calibration mode automatically.
2. Rotate the system gently in all directions (pan and tilt) until all four systems reach `3/3/3/3`:
   - SYS (System)
   - GYR (Gyroscope)
   - ACC (Accelerometer)
   - MAG (Magnetometer)
3. When calibration is complete, you'll see `CALIBRATION DONE!` on the display.
4. The offsets are written to EEPROM and used automatically on future boots.
5. You can view the current offsets using the `CALSTATS` serial command.
6. If desired, copy the printed offsets and hard-code them by enabling `USE_HARDCODED = true` in your firmware and updating the `defaultOffsets` struct.

### 🔤 Related Serial Commands

| Command     | Description                          |
|-------------|--------------------------------------|
| `CALIBRATE` | Starts the calibration routine       |
| `CALSTATS`  | Prints current calibration offsets   |

---
## 🧾 Serial Command Reference

The ESP32 controller accepts a mix of GS-232-style and custom serial commands over USB (or virtual serial via proxy).

### 🎛 GS-232 Compatible Commands

| Command     | Description                                 |
|-------------|---------------------------------------------|
| `AZxxx.x`   | Set target azimuth (e.g. `AZ123.4`)         |
| `ELxxx.x`   | Set target elevation (e.g. `EL45.6`)        |
| `AZ`        | Report current azimuth                     |
| `EL`        | Report current elevation                   |
| `P`         | Report current position as `AZ EL`         |
| `SA`        | Stop azimuth movement                      |
| `SE`        | Stop elevation movement                    |
| `VE`        | Report firmware version (returns `VEAdamESP32v1.0`) |
| `Q` or `q`  | Disconnect from Gpredict / rotctld         |

### 🛠 Custom Utility Commands

| Command     | Description                                 |
|-------------|---------------------------------------------|
| `HELP`      | Show list of available commands             |
| `HOME`      | Move to Home position (Az 180, El 45)       |
| `DANCE`     | Perform calibration movements (L/R/U/D)     |
| `DEBUG`     | Toggle raw sensor debug output to serial    |
| `CALIBRATE` | Start calibration and store offsets to EEPROM |
| `CALSTATS`  | Print current calibration offsets           |

> All responses end with a `\r` (carriage return), as expected by Gpredict/rotctld.
>  
> Invalid or unknown commands will return `RPRT -1\r`.

---

## 🔌 Hardware Used

* [ Panasonic WV-7230](https://archive.org/details/manuallib-id-2720404 " Panasonic WV-7230") pan/tilt head (relay-controlled)
* [4-channel relay module](https://www.amazon.com/dp/B08PP8HXVD "4-channel relay module")
* [ESP32-S3 or similar microcontroller](https://www.amazon.com/dp/B0DG8L7MQ9 "ESP32-S3 or similar microcontroller")
* [ESP Breakout Board](https://www.amazon.com/dp/B0CD2512JV "ESP Breakout Board")
* [SSD1306 0.96 I2C OLED Board](https://www.amazon.com/dp/B09T6SJBV5 "SSD1306 0.96 I2C OLED Board")
* [BNO055](https://www.adafruit.com/product/2472 "BNO055") (IMU with absolute orientation for compass bearing)
* [24v AC/AC adapter](https://www.amazon.com/dp/B01N3ALUBS "24v AC/AC adapter")
* [Project Box](https://www.amazon.com/dp/B09DD8HH1L "Project Box")
* [24VAC 5VDC Step down converter](https://www.amazon.com/dp/B0BB8YWBHX "24VAC 5VDC Step down converter")
* [Mounting Kit](https://www.amazon.com/dp/B0CQR4XBPC "Mounting Kit")


---

## 📝 License

This project is licensed under the [MIT License](LICENSE).

---

## 🤝🏼 Maintainer

Created by [Adam Melancon](https://github.com/adammelancon)
