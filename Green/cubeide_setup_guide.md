# Complete STM32CubeIDE Settings & Wiring Guide

## Project: Multi-Sensor Environmental Monitor
**MCU:** STM32F103C8T6 (Blue Pill) @ **72 MHz**

> [!IMPORTANT]
> Your system clock is **72 MHz** (HSE 8MHz × PLL ×9). This is critical — the servo timer reference you shared (Prescaler=15, Period=9999) was designed for an **8 MHz clock** and will produce **450 Hz** instead of 50 Hz on your board. The correct values for 72 MHz are given below.

---

## Part 1: Complete STM32CubeIDE (.ioc) Configuration

### Step 1 — RCC (Already Configured ✅)

1. **Pinout & Configuration → System Core → RCC**
2. High Speed Clock (HSE): **Crystal/Ceramic Resonator**
3. **Clock Configuration tab** — verify these values:

| Parameter | Value |
|---|---|
| HSE | 8 MHz |
| PLL Source | HSE |
| PLL Mul | ×9 |
| SYSCLK | 72 MHz |
| AHB Prescaler | /1 → 72 MHz |
| APB1 Prescaler | /2 → 36 MHz |
| APB2 Prescaler | /1 → 72 MHz |
| APB1 Timer Clocks | 72 MHz (auto-doubled) |

---

### Step 2 — SYS (Already Configured ✅)

1. **Pinout & Configuration → System Core → SYS**
2. Debug: **Serial Wire**
3. Timebase Source: **SysTick**

---

### Step 3 — I2C1 (Already Configured ✅) — For OLED + BMP280+AHT20

The SSD1306 OLED, BMP280, and AHT20 all share this single I2C bus (they have different addresses, so no conflict).

1. **Pinout & Configuration → Connectivity → I2C1**
2. I2C Mode: **I2C**
3. **Configuration → Parameter Settings:**

| Parameter | Value |
|---|---|
| I2C Speed Mode | **Fast Mode** |
| I2C Clock Speed | **400000** (400 kHz) |
| Addressing Mode | 7-bit |

**Pins** (auto-assigned):
- **PB6** → I2C1_SCL
- **PB7** → I2C1_SDA

---

### Step 4 — ADC1 (🆕 NEW — Enable This) — For TEMT6000

1. **Pinout & Configuration → Analog → ADC1**
2. Check **IN0** to enable Channel 0 on **PA0**
3. **Configuration → Parameter Settings:**

| Parameter | Value |
|---|---|
| Scan Conversion Mode | **Disabled** |
| Continuous Conversion Mode | **Disabled** |
| Discontinuous Conversion Mode | **Disabled** |
| External Trigger Conversion | **Software Start** (SWSTART) |
| Data Alignment | **Right** |
| Number of Conversion | **1** |

4. **Configuration → Rank Settings:**

| Parameter | Value |
|---|---|
| Rank | 1 |
| Channel | **Channel 0** |
| Sampling Time | **239.5 Cycles** |

> [!TIP]
> Using 239.5 cycles (maximum) gives the best accuracy for the light sensor since it doesn't need fast sampling.

**Pin**: **PA0** → ADC1_IN0

---

### Step 5 — TIM1 (Already Configured — Optional)

TIM1 was originally used for DHT11 microsecond delays. Since DHT11 is removed, TIM1 is **no longer needed**. You have two options:

- **Option A (Safe):** Leave it as-is. It doesn't hurt anything.
- **Option B (Clean):** Remove TIM1 in CubeIDE to free resources.

If keeping it:
1. **Pinout & Configuration → Timers → TIM1**
2. Clock Source: **Internal Clock**
3. Prescaler: **71**
4. Counter Period: **65535**

---

### Step 6 — TIM2 (🆕 NEW — Enable This) — For Servo Motor PWM

1. **Pinout & Configuration → Timers → TIM2**
2. Clock Source: **Internal Clock**
3. Channel2: **PWM Generation CH2**
4. **Configuration → Parameter Settings:**

| Parameter | Value | Why |
|---|---|---|
| Prescaler | **71** | 72 MHz ÷ (71+1) = **1 MHz** (1 µs per tick) |
| Counter Period | **19999** | 1 MHz ÷ (19999+1) = **50 Hz** (20 ms period) |
| Counter Mode | Up | — |
| Auto-reload Preload | Disable | — |

5. **Configuration → PWM Generation Channel 2:**

| Parameter | Value |
|---|---|
| Mode | PWM mode 1 |
| Pulse (initial duty) | **500** |
| CH Polarity | High |
| CH Idle State | Reset |

> [!WARNING]
> **Do NOT use** Prescaler=15 and Period=9999 from the reference. That reference was designed for an **8 MHz clock**. Your board runs at **72 MHz**. Using those values would produce **450 Hz** PWM instead of the required **50 Hz** for the servo, and the motor will **NOT work correctly**.
>
> **Calculation proof:**
> - Reference (8 MHz): 8,000,000 ÷ 16 ÷ 10,000 = **50 Hz** ✅
> - Your board (72 MHz) with same values: 72,000,000 ÷ 16 ÷ 10,000 = **450 Hz** ❌
> - Your board (72 MHz) with correct values: 72,000,000 ÷ 72 ÷ 20,000 = **50 Hz** ✅

**Pin**: **PA1** → TIM2_CH2 (auto-assigned)

#### Servo Pulse-to-Angle Mapping (at Period=19999)

| Angle | Pulse Width | Compare Value |
|---|---|---|
| 0° (Closed) | 0.5 ms | **500** |
| 45° | 1.0 ms | 1000 |
| 90° (Center) | 1.5 ms | **1500** |
| 135° | 2.0 ms | 2000 |
| 180° (Open) | 2.5 ms | **2500** |

---

### Step 7 — GPIO: Remove PB9 (Optional Cleanup)

PB9 was configured as GPIO_Output for the DHT11 data pin. Since DHT11 is removed:

1. **Click PB9** in the Pinout view → set to **Reset_State** (or just leave it, it won't cause issues)

---

### Step 8 — Project Manager Settings

1. **Project Manager → Code Generator**
2. ✅ Check: **Generate peripheral initialization as a pair of .c/.h files per peripheral** (recommended)
3. ✅ Check: **Keep User Code when re-generating**

---

### Step 9 — Generate Code

1. Press **Ctrl+S** to save the .ioc file
2. Click **Yes** when prompted to generate code
3. CubeIDE will auto-generate `MX_ADC1_Init()` and `MX_TIM2_Init()` functions

> [!CAUTION]
> After regenerating code, you **MUST delete** the manual `MX_ADC1_Init()` and `MX_TIM2_Init()` functions from **USER CODE 4** in `main.c` (lines ~590–675 currently). CubeIDE will generate its own versions of these functions, and having duplicates will cause **compilation errors**.
>
> Also remove the two `static void MX_ADC1_Init(void);` and `static void MX_TIM2_Init(void);` lines we manually added to the function prototypes section (around lines 59 and 62) — CubeIDE will generate those too.

---

### Summary of All CubeIDE Peripheral Settings

```
┌──────────────────────────────────────────────────────┐
│  PERIPHERAL        STATUS       PURPOSE              │
├──────────────────────────────────────────────────────┤
│  RCC (HSE+PLL)     ✅ Existing   72 MHz system clock │
│  SYS (SWD)         ✅ Existing   Debug interface     │
│  I2C1 (Fast)       ✅ Existing   OLED + BMP280+AHT20 │
│  TIM1              ⚪ Optional   (Legacy, not used)  │
│  ADC1 IN0          🆕 NEW        TEMT6000 light      │
│  TIM2 CH2 PWM      🆕 NEW        Servo motor         │
│  PB9 GPIO_Output   ⚪ Optional   (Legacy DHT11 pin) │
└──────────────────────────────────────────────────────┘
```

---

## Part 2: Complete Wiring Guide

### Pin Assignment Map (STM32F103C8T6)

```
                    STM32F103C8T6 (Blue Pill)
                   ┌─────────────────────┐
                   │         USB         │
                   │  ┌───────────────┐  │
             3.3V ─┤  │               │  ├─ GND
              GND ─┤  │               │  ├─ GND
              PA0 ─┤──│─── TEMT6000   │  ├─ 3.3V
              PA1 ─┤──│─── SERVO SIG  │  ├─
                   │  │               │  │
             PA13 ─┤──│─── SWDIO      │  ├─
             PA14 ─┤──│─── SWCLK      │  ├─
                   │  │               │  │
              PB6 ─┤──│─── I2C1_SCL ──│──├─ (OLED + BMP280+AHT20)
              PB7 ─┤──│─── I2C1_SDA ──│──├─ (OLED + BMP280+AHT20)
                   │  │               │  │
              PD0 ─┤──│─── HSE_IN     │  │
              PD1 ─┤──│─── HSE_OUT    │  │
                   │  └───────────────┘  │
                   └─────────────────────┘
```

---

### Device 1: SSD1306 OLED Display (I2C, Address: 0x78)

| OLED Pin | Connect To | Wire Color (typical) |
|---|---|---|
| **VCC** | STM32 **3.3V** | Red |
| **GND** | STM32 **GND** | Black |
| **SCL** | STM32 **PB6** | Yellow |
| **SDA** | STM32 **PB7** | Blue |

> [!NOTE]
> The SSD1306 I2C address is typically **0x78** (0x3C << 1). Some modules use 0x7A. Check your module's silkscreen or datasheet.

---

### Device 2: BMP280+AHT20 Combo Module (I2C)

This is a **single PCB** with two sensor ICs. It has only **4 pins** — much simpler than a standalone BME280.

| Module Pin | Connect To | Function |
|---|---|---|
| **VCC** | STM32 **3.3V** | Power (supports 3.3V and 5V) |
| **GND** | STM32 **GND** | Ground |
| **SCL** | STM32 **PB6** | I2C Clock — shared with OLED |
| **SDA** | STM32 **PB7** | I2C Data — shared with OLED |

> [!IMPORTANT]
> The combo module contains **two separate sensors** at different I2C addresses:
> - **BMP280** at address **0x76** (shifted: **0xEC**) — reads Temperature + Pressure
> - **AHT20** at address **0x38** (shifted: **0x70**) — reads Temperature + Humidity
>
> No SDO or CSB pin configuration needed — the addresses are fixed on the combo module.

> [!TIP]
> The module supports both 3.3V and 5V logic levels. Use **3.3V** since that matches the STM32.

---

### Device 3: TEMT6000 Light Sensor (Analog)

| TEMT6000 Pin | Connect To | Function |
|---|---|---|
| **VCC** (V) | STM32 **3.3V** | Power |
| **GND** (G) | STM32 **GND** | Ground |
| **SIG** (S) | STM32 **PA0** | Analog signal → ADC1_IN0 |

> [!NOTE]
> The reference uses 5V on Arduino, but our STM32 runs at **3.3V**. The TEMT6000 works fine at 3.3V — the output voltage range will be 0–3.3V instead of 0–5V. Our code already maps correctly to **3300 mV** (not 5000 mV).

---

### Device 4: Servo Motor (SG90 / MG90S / MG996R)

| Servo Wire | Color | Connect To | Notes |
|---|---|---|---|
| **Signal** | Orange or Yellow | STM32 **PA1** | PWM from TIM2_CH2 |
| **VCC** | Red | **External 5V supply** | ⚠️ NOT from STM32! |
| **GND** | Brown or Black | **Common GND** | Shared with STM32 GND |

> [!CAUTION]
> **Never power the servo directly from the STM32's 3.3V or 5V pin.** Servos draw 200–700 mA (up to 1.5A stall), which will brown-out or damage the STM32.
>
> **Correct setup:**
> - Use an external 5V power supply (or USB power bank) for the servo VCC
> - Connect the GND of the external supply to the STM32 GND (common ground)
> - The 3.3V PWM signal from PA1 is enough to drive standard servos

---

### Complete Wiring Summary Diagram

```
    ┌─────────────────┐
    │   STM32F103C8T6  │
    │   (Blue Pill)    │
    │                  │
    │  3.3V ──────────────────┬──── OLED VCC
    │                  │      ├──── BMP280+AHT20 VCC
    │                  │      └──── TEMT6000 VCC
    │                  │
    │  GND ───────────────────┬──── OLED GND
    │                  │      ├──── BMP280+AHT20 GND
    │                  │      ├──── TEMT6000 GND
    │                  │      └──── Servo GND ──── Ext.5V GND
    │                  │
    │  PB6 (SCL) ─────────────┬──── OLED SCL
    │                  │      └──── BMP280+AHT20 SCL
    │                  │
    │  PB7 (SDA) ─────────────┬──── OLED SDA
    │                  │      └──── BMP280+AHT20 SDA
    │                  │
    │  PA0 (ADC) ─────────────────── TEMT6000 SIG
    │                  │
    │  PA1 (PWM) ─────────────────── Servo Signal
    │                  │
    │  PA13 (SWDIO) ──────────────── ST-Link (debug)
    │  PA14 (SWCLK) ──────────────── ST-Link (debug)
    │                  │
    │  PD0 ◄──── 8MHz Crystal ────► PD1
    │                  │
    └─────────────────┘

                    ┌─────────┐
                    │ Ext. 5V │──── Servo VCC (Red)
                    │ Supply  │──── GND (shared with STM32)
                    └─────────┘
```

---

## Part 3: I2C Bus Device Summary

All three devices on the I2C1 bus have **different addresses**, so there is no conflict:

| Device | Chip | 7-bit Address | 8-bit (shifted) Address | Measures |
|---|---|---|---|---|
| SSD1306 OLED | SSD1306 | 0x3C | **0x78** | Display |
| BMP280 | BMP280 | 0x76 | **0xEC** | Temperature + Pressure |
| AHT20 | AHT20 | 0x38 | **0x70** | Temperature + Humidity |

> [!TIP]
> If you ever see communication errors with all devices on the bus, add **4.7 kΩ pull-up resistors** on the SDA and SCL lines to 3.3V. Many breakout modules already include these built-in, so check first.

---

## Part 4: Checklist Before Flashing

- [ ] CubeIDE: ADC1 IN0 enabled with settings from Step 4
- [ ] CubeIDE: TIM2 CH2 PWM enabled with **Prescaler=71, Period=19999**
- [ ] CubeIDE: I2C1 in Fast Mode (already done)
- [ ] CubeIDE: Code regenerated (Ctrl+S → Generate)
- [ ] Code: Removed duplicate `MX_ADC1_Init()` and `MX_TIM2_Init()` from USER CODE 4
- [ ] Wiring: BMP280+AHT20 module connected (VCC, GND, SCL→PB6, SDA→PB7)
- [ ] Wiring: TEMT6000 SIG connected to **PA0**
- [ ] Wiring: Servo signal connected to **PA1**
- [ ] Wiring: Servo powered from **external 5V** (not STM32)
- [ ] Wiring: **Common GND** between STM32 and external power supply
- [ ] Wiring: OLED and BMP280+AHT20 share PB6/PB7
