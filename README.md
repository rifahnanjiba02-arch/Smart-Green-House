# Smart Greenhouse

STM32F103C8T6 firmware for monitoring and automatically controlling a small greenhouse.

## Hardware

- PA0: light sensor ADC
- PA1: soil-moisture sensor ADC
- PA6: window servo
- PA8: active buzzer
- PA9: pump MOSFET control
- PA10: fan MOSFET control
- PB6/PB7: I2C OLED, AHT20 and BMP280

## Automatic behaviour

- Pump: starts below 30% soil moisture, runs for 1 second, then waits 3 seconds before checking again.
- Buzzer: gives two ticks after light remains below 20% for 3 seconds.
- Window: opens at 30 C or 78% humidity; closes at 29 C and 75% humidity.
- Fan: starts at 32 C or 82% humidity; stops at 31 C and 80% humidity.
- Pump, fan and moving servo are mutually exclusive. Sensors, OLED and buzzer continue operating independently.

## Build

Open `MDK-ARM/smart-green-house.uvprojx` in Keil uVision, select `GREENHOUSE_TEST_FULL` in `Core/Src/main.c`, then rebuild and flash the target.

The pump is powered from the USB-derived 5 V rail through a MOSFET. Do not power the pump from the STM32 3.3 V pin.
