# Claude Code Instructions for invpend_BLDC

## Project Overview

This is an **STM32F401RE-based inverted pendulum control system** using a BLDC motor with Field-Oriented Control (FOC). The system runs a 1 kHz control loop that communicates with a Raspberry Pi via SPI to receive torque commands and transmit sensor feedback.

### Architecture Summary

```
┌───────────────────┐     SPI (1 kHz)     ┌───────────────────┐
│   Raspberry Pi    │◄───────────────────►│   STM32F401RE     │
│   (Controller)    │  torque_cmd / fb    │   (Low-Level)     │
└───────────────────┘                     └─────────┬─────────┘
                                                    │
                    ┌───────────────────────────────┴───────────────────────────────┐
                    │                                                               │
              ┌─────▼─────┐                                                  ┌──────▼──────┐
              │  TIM3     │                                                  │  MC SDK     │
              │  Encoder  │                                                  │  (FOC)      │
              │  PB4/PB5  │                                                  │  TIM1 PWM   │
              └─────┬─────┘                                                  └──────┬──────┘
                    │                                                               │
              ┌─────▼─────┐                                                  ┌──────▼──────┐
              │ Pendulum  │                                                  │  BLDC Motor │
              │ (2400 CPR)│                                                  │  + Encoder  │
              └───────────┘                                                  └─────────────┘
```

## Build System

- **Toolchain**: CMake + GCC ARM (arm-none-eabi-gcc)
- **Build directory**: `build/Debug/` or `build/Release/`
- **Target**: `invpend_BLDC.elf`

### Build Commands

```bash
# Configure (from project root)
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug

# Build
cmake --build build/Debug

# Clean build
cmake --build build/Debug --target clean
```

## Key Source Files

| File | Purpose |
|------|---------|
| [Src/main.c](Src/main.c) | System init, peripheral setup, main loop |
| [Src/pendulum_control.c](Src/pendulum_control.c) | 1 kHz state machine, SPI protocol, encoder reading |
| [Src/torque_control.c](Src/torque_control.c) | Torque→Current conversion, MC SDK interface |
| [Src/chrono.c](Src/chrono.c) | DWT-based cycle timing (84 MHz resolution) |
| [Src/motorcontrol.c](Src/motorcontrol.c) | MC SDK initialization |
| [invpend_BLDC.ioc](invpend_BLDC.ioc) | STM32CubeMX configuration |

### Header Files

| File | Key Definitions |
|------|-----------------|
| [Inc/app_config.h](Inc/app_config.h) | `TEST_MODE_NO_MOTOR`, `DEBUG_PENDULUM_ENCODER`, `DEBUG_PRINT_INTERVAL_MS` |
| [Inc/pendulum_control.h](Inc/pendulum_control.h) | State machine states, SPI protocol (8 bytes) |
| [Inc/torque_control.h](Inc/torque_control.h) | `MOTOR_KT_NM_PER_A = 0.0234`, `MAX_CURRENT_A = 5.0` |
| [Inc/chrono.h](Inc/chrono.h) | `RCC_SYS_CLOCK_FREQ = 84000000` |

### Debug Configuration

Debug and test modes are configured in `Inc/app_config.h`:

```c
// Test mode: disable motor power stage (motor cannot spin)
#define TEST_MODE_NO_MOTOR          1

// Enable pendulum encoder debug output via UART (set to 0 to disable)
#define DEBUG_PENDULUM_ENCODER      1

// Debug print interval in milliseconds
#define DEBUG_PRINT_INTERVAL_MS     100
```

**TEST_MODE_NO_MOTOR**: When enabled:
- Motor power stage is completely disabled
- All torque commands are ignored
- State machine skips waiting for SPI and runs independently at 1 kHz
- User button (PC13) is disabled (won't start/stop motor)
- Useful for testing encoder and communication without motor risk

**DEBUG_PENDULUM_ENCODER**: When enabled:
- Pendulum encoder count and measured torque printed to UART2 (921600 baud)
- Output interval controlled by `DEBUG_PRINT_INTERVAL_MS` (default 100ms = 10 Hz)
- Useful for testing encoder connection without Raspberry Pi

### Test Mode Behavior

When `TEST_MODE_NO_MOTOR=1`, the state machine operates differently:

```
Normal mode:                          Test mode:
STATE_START → wait for SPI    →       STATE_START → immediately to READ
STATE_READ  → prepare TX buf  →       STATE_READ  → read encoders, debug print
STATE_WAIT_SPI → wait for SPI →       (wait 1ms)  → loop back to READ
STATE_CONTROL → apply torque  →       (skipped)
```

**Known behavior in test mode:**
- Measured torque shows ~47 mNm offset even with motor unpowered. This is normal - the current sense ADCs have uncalibrated DC offset. The MC SDK calibrates these when `MC_StartMotor1()` is called, which is skipped in test mode.
- The user button (PC13) is overridden to do nothing via `UI_HandleStartStopButton_cb()` in `pendulum_control.c`.

## Hardware Wiring Guide

> **Reference Documentation** (in `docs/` folder):
> - [docs/pinout_F401RE.png](docs/pinout_F401RE.png) - NUCLEO-F401RE pinout diagram
> - [docs/en.DM00105823.pdf](docs/en.DM00105823.pdf) - NUCLEO-F401RE User Manual
> - [docs/um1996-getting-started-with-xnucleoihm08m1...pdf](docs/) - X-NUCLEO-IHM08M1 Guide
> - [docs/b82e1c3b407f_1.pdf](docs/b82e1c3b407f_1.pdf) - Maxon Motor+Encoder Configuration
> - [docs/ECX flat 42 M 24V.pdf](docs/ECX%20flat%2042%20M%2024V.pdf) - Motor Datasheet

### Hardware Stack

```
┌─────────────────────────────────┐
│      Raspberry Pi (SPI Master) │  ← High-level controller
└───────────────┬─────────────────┘
                │ SPI (4 wires)
┌───────────────▼─────────────────┐
│      NUCLEO-F401RE              │  ← Low-level real-time control
├─────────────────────────────────┤
│      X-NUCLEO-IHM08M1           │  ← BLDC power stage (stacked)
└───────────────┬─────────────────┘
                │ 3-phase + encoder
┌───────────────▼─────────────────┐
│      BLDC Motor + Encoder       │
└─────────────────────────────────┘
                │
┌───────────────▼─────────────────┐
│      Pendulum + Encoder         │
└─────────────────────────────────┘
```

### Complete Pin Summary

| Function | STM32 Pin | Arduino Pin | Morpho Pin | Peripheral |
|----------|-----------|-------------|------------|------------|
| **Pendulum Encoder A** | PB4 | D5 | CN9-6 | TIM3_CH1 |
| **Pendulum Encoder B** | PB5 | D4 | CN9-5 | TIM3_CH2 |
| **Motor Encoder A** | PA15 | - | CN7-17 | TIM2_CH1 |
| **Motor Encoder B** | PB3 | D3 | CN9-4 | TIM2_CH2 |
| **SPI3 NSS** | PA4 | A2 | CN7-32 | SPI3_NSS |
| **SPI3 SCK** | PC10 | - | CN7-1 | SPI3_SCK |
| **SPI3 MISO** | PC11 | - | CN7-2 | SPI3_MISO |
| **SPI3 MOSI** | PC12 | - | CN7-3 | SPI3_MOSI |
| **UART TX** | PA2 | D1 | CN9-35 | USART2_TX |
| **UART RX** | PA3 | D0 | CN9-37 | USART2_RX |
| **Start/Stop Button** | PC13 | - | CN7-23 | GPIO (EXTI) |

---

### 1. Pendulum Encoder Wiring (2400 CPR)

Connect the pendulum quadrature encoder to TIM3:

| Encoder Wire | STM32 Pin | Arduino Header | Morpho Connector |
|--------------|-----------|----------------|------------------|
| **Channel A** | PB4 | **D5** | CN9 pin 6 |
| **Channel B** | PB5 | **D4** | CN9 pin 5 |
| **VCC** | 5V or 3.3V | CN6 pin 4 (5V) or CN6 pin 1 (3.3V) | - |
| **GND** | GND | CN6 pin 6 or 7 | - |

**Notes**:
- Internal pull-ups are enabled on PB4/PB5
- Use 5V if encoder requires it, 3.3V for 3.3V-compatible encoders
- If rotation direction is inverted, swap A and B wires

---

### 2. Motor Encoder Wiring (ENX 42 MILE - Differential)

The motor uses a **maxon ENX 42 MILE 2048IMP** encoder with **differential line driver outputs** (RS-422 style). This requires a **differential line receiver IC** to convert to single-ended signals for the STM32.

#### Encoder Connector Pinout (TE 1-215915-0, 10-pin)

| Pin | Signal | Wire Color | Description |
|-----|--------|------------|-------------|
| 1 | NC | - | Not connected |
| 2 | VCC | Yellow | 5V ±10% supply |
| 3 | GND | Brown | Ground |
| 4 | NC | - | Not connected |
| 5 | **A\\** | Gray | Channel A inverted |
| 6 | **A** | Blue | Channel A true |
| 7 | **B\\** | Green | Channel B inverted |
| 8 | **B** | Purple | Channel B true |
| 9 | NC | - | Not connected |
| 10 | NC | - | Not connected |

#### Required Hardware: Differential Line Receiver

Since the encoder outputs differential RS-422 signals, you need a **quad differential line receiver** IC:

| IC Option | Package | Notes |
|-----------|---------|-------|
| **AM26C32** | DIP-16/SOIC-16 | Common, 5V, RS-422 receiver |
| **SN75175** | DIP-16/SOIC-16 | TI, RS-422/RS-485 receiver |
| **MAX3095** | DIP-16/SOIC-16 | Maxim, ±15kV ESD protected |

#### Line Receiver Wiring

```
Encoder                Line Receiver (AM26C32)           STM32
─────────              ────────────────────────          ─────
A  (blue)  ───────────► 1A  (pin 2)  ─┐
A\ (gray)  ───────────► 1B  (pin 3)  ─┴──► 1Y (pin 1) ──► PA15 (TIM2_CH1)

B  (purple)───────────► 2A  (pin 5)  ─┐
B\ (green) ───────────► 2B  (pin 6)  ─┴──► 2Y (pin 7) ──► PB3  (TIM2_CH2)

VCC (yellow)──────────► VCC (pin 16)
GND (brown) ──────────► GND (pin 8)
                        Enable pins (4, 12) → GND (active low)
```

#### AM26C32 Pinout Reference

```
        ┌────┬────┐
   1Y ──┤ 1  │ 16 ├── VCC
   1A ──┤ 2  │ 15 ├── 4B
   1B ──┤ 3  │ 14 ├── 4A
  1EN ──┤ 4  │ 13 ├── 4Y
   2Y ──┤ 5  │ 12 ├── 4EN
   2A ──┤ 6  │ 11 ├── 3Y
   2B ──┤ 7  │ 10 ├── 3A
  GND ──┤ 8  │  9 ├── 3B
        └────┴────┘
```

#### STM32 Connection (after line receiver)

| Signal | STM32 Pin | Morpho Connector |
|--------|-----------|------------------|
| **Channel A** (from 1Y) | PA15 | CN7 pin 17 |
| **Channel B** (from 2Y) | PB3 | CN9 pin 4 (D3) |

#### Encoder Specifications

| Parameter | Value |
|-----------|-------|
| Resolution | 2048 counts/turn |
| Supply Voltage | 5V ±10% |
| Output | Differential / CMOS |
| Max Speed | 28000 rpm (electrical) |
| Current Draw | 15 mA typical |

#### Alternative: Direct Connection (Not Recommended)

If noise is not a concern, you *might* connect the single-ended outputs (A and B only, ignoring A\ and B\) directly to the STM32. The STM32F401 GPIOs PA15 and PB3 are **5V tolerant**. However, this sacrifices noise immunity.

```
⚠️ Direct connection (use differential receiver for production!)
A  (blue)   → PA15
B  (purple) → PB3
VCC (yellow)→ 5V
GND (brown) → GND
(A\ and B\ not connected)
```

---

### 3. Raspberry Pi SPI Wiring

Connect Raspberry Pi GPIO to STM32 SPI3 (slave mode):

| RPi Pin | RPi GPIO | Signal | STM32 Pin | Morpho |
|---------|----------|--------|-----------|--------|
| 24 | GPIO8 (CE0) | NSS | PA4 | CN7-32 |
| 23 | GPIO11 (SCLK) | SCK | PC10 | CN7-1 |
| 21 | GPIO9 (MISO) | MISO | PC11 | CN7-2 |
| 19 | GPIO10 (MOSI) | MOSI | PC12 | CN7-3 |
| 6 | - | GND | GND | CN7-20 |

**SPI Configuration on Raspberry Pi**:
```python
# Python spidev example
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0b00  # CPOL=0, CPHA=0
```

**Important**: Connect GND between RPi and STM32!

---

### 4. X-NUCLEO-IHM08M1 Power Stage

The IHM08M1 shield stacks directly onto the NUCLEO-F401RE. These pins are used automatically:

| Function | Pin | Notes |
|----------|-----|-------|
| PWM UH | PA8 | High-side phase U |
| PWM UL | PA7 | Low-side phase U |
| PWM VH | PA9 | High-side phase V |
| PWM VL | PB0 | Low-side phase V |
| PWM WH | PA10 | High-side phase W |
| PWM WL | PB1 | Low-side phase W |
| Phase U Current | PA0 | ADC1_IN0, shunt amplifier |
| Phase V Current | PC1 | ADC1_IN11, shunt amplifier |
| Phase W Current | PC0 | ADC1_IN10, shunt amplifier |
| Bus Voltage | PA1 | ADC1_IN1, voltage divider |
| OCP (Overcurrent) | PA6 | TIM1_BKIN, active low |

**Motor Connection (J2 on IHM08M1)**:

| Terminal | Connection |
|----------|------------|
| U | Motor phase U (blue wire typical) |
| V | Motor phase V (green wire typical) |
| W | Motor phase W (yellow wire typical) |

**Power Supply (J1 on IHM08M1)**:
- VIN: 10-48V DC (24V typical for this motor)
- GND: Power ground

---

### 5. UART Debug Connection

UART2 is connected to ST-LINK virtual COM port (no external wiring needed):

| Function | Pin | Notes |
|----------|-----|-------|
| TX | PA2 | To PC via ST-LINK |
| RX | PA3 | From PC via ST-LINK |

**Baud rate**: 921600 (8N1)

---

### 6. Morpho Connector Pinout Reference

```
        CN7 (Left)                           CN9 (Right)
    ┌───────────────┐                   ┌───────────────┐
  1 │ PC10 (SCK)    │                 1 │ PC9           │
  2 │ PC11 (MISO)   │                 2 │ PC8           │
  3 │ PC12 (MOSI)   │                 3 │ PC6           │
  4 │ VDD           │                 4 │ PB3 (Enc B)   │
    │ ...           │                 5 │ PB5 (Pend B)  │
 17 │ PA15 (Enc A)  │                 6 │ PB4 (Pend A)  │
    │ ...           │                   │ ...           │
 20 │ GND           │                   │               │
    │ ...           │                   │               │
 32 │ PA4 (NSS)     │                   │               │
    └───────────────┘                   └───────────────┘
```

See [docs/pinout_F401RE.png](docs/pinout_F401RE.png) for complete pinout.

---

### 7. Motor Hall Sensors (for FOC commutation)

The motor has built-in Hall sensors for commutation:

**Hall Sensor Connector (Molex 43025-0600, 6-pin)**:

| Pin | Signal | Wire Color | STM32 |
|-----|--------|------------|-------|
| 1 | Hall 1 | Yellow | (via IHM08M1) |
| 2 | Hall 2 | Brown | (via IHM08M1) |
| 3 | Hall 3 | Gray | (via IHM08M1) |
| 4 | GND | Blue | GND |
| 5 | VCC | Green | 2.5-5.5V |
| 6 | NC | - | - |

**Note**: The X-NUCLEO-IHM08M1 has a Hall sensor connector. Check if your configuration uses encoder-based FOC (this project) or Hall-based commutation.

---

### 8. Motor Phase Wiring

**Motor Connector (Molex 39-01-2040, 4-pin)**:

| Pin | Signal | Wire Color |
|-----|--------|------------|
| 1 | Motor 1 (U) | Red |
| 2 | Motor 2 (V) | Black |
| 3 | Motor 3 (W) | White |
| 4 | NC | - |

Connect to IHM08M1 J2 terminals (U, V, W).

---

### Wiring Checklist

- [ ] **Pendulum encoder**: A→PB4(D5), B→PB5(D4), VCC→5V, GND→GND
- [ ] **Motor encoder (differential)**:
  - [ ] Line receiver IC (AM26C32 or similar) installed
  - [ ] Encoder A/A\ → Line receiver → PA15
  - [ ] Encoder B/B\ → Line receiver → PB3(D3)
  - [ ] Encoder VCC→5V, GND→GND
- [ ] **Motor phases**: U(red)→J2-U, V(black)→J2-V, W(white)→J2-W
- [ ] **Hall sensors** (if used): Connect to IHM08M1 Hall connector
- [ ] **RPi SPI**: NSS→PA4, SCK→PC10, MISO→PC11, MOSI→PC12, GND→GND
- [ ] **X-NUCLEO-IHM08M1**: Stacked on Nucleo
- [ ] **Power**: 24V DC to IHM08M1 J1 terminals
- [ ] **Common ground**: Ensure RPi, Nucleo, encoder, and power supply share GND

## SPI Protocol

**Buffer size**: 8 bytes, **Endianness**: Big-endian

### RX from Raspberry Pi
```
Bytes [0-1]: int16_t torque_cmd      // Torque command in milli-Nm
Bytes [2-3]: int16_t reserved1       // Future use
Bytes [4-5]: int16_t reserved2       // Future use
Bytes [6-7]: int16_t reserved3       // Future use
```

### TX to Raspberry Pi
```
Bytes [0-1]: int16_t pendulum_pos    // Encoder counts
Bytes [2-3]: int16_t motor_pos       // Electrical angle
Bytes [4-5]: int16_t motor_vel       // Velocity in SPEED_UNIT
Bytes [6-7]: int16_t measured_torque // Actual motor torque in milli-Nm
```

## State Machine (`StateMachine_Run`)

```
STATE_START (0) → Wait for first SPI transaction
       ↓
STATE_READ (1) → Read encoders, prepare TX buffer
       ↓
STATE_WAIT_SPI (2) → Wait for SPI completion (or timeout)
       ↓
STATE_CONTROL (4) → Apply torque via MC SDK
       ↓
    [loop back to STATE_READ]

STATE_OVERTIME (3) → SPI timeout recovery
STATE_ERROR (50) → Safety stop
STATE_HALT (99) → Halted state
```

## Timing & Interrupts

| Component | Frequency | Priority |
|-----------|-----------|----------|
| System Clock | 84 MHz | - |
| Motor FOC (TIM1) | ~16 kHz | 0 (highest) |
| SPI DMA | RPi-driven (~1 kHz) | 0 |
| Pendulum Control Loop | 1 kHz target | main loop |
| UART Debug | 921600 baud | 3 |

## Critical Constants

```c
// Motor parameters (maxon ECX FLAT 42 M, 24V)
MOTOR_KT_NM_PER_A   = 0.0282f   // Torque constant (28.2 mNm/A from datasheet)
MAX_CURRENT_A       = 7.33f     // Nominal current (continuous) from datasheet
POLE_PAIR_NUM       = 8         // Motor pole pairs

// Motor encoder (ENX 42 MILE)
MOTOR_ENCODER_CPR   = 2048      // Counts per turn (motor encoder)

// Pendulum encoder
PENDULUM_COUNTS_PER_REV = 2400  // Counts per turn (pendulum encoder)

// Timing
T_SAMPLE            = 0.001f    // 1 kHz sample time
SPI_TIMEOUT_MULT    = 1.5f      // Overtime threshold
```

### Motor Specifications (from datasheet)

| Parameter | Value |
|-----------|-------|
| Nominal voltage | 24 V |
| Torque constant | 28.2 mNm/A |
| Speed constant | 339 rpm/V |
| Terminal resistance | 0.233 Ω |
| Terminal inductance | 0.153 mH |
| Nominal torque | 214 mNm |
| Nominal current | 7.33 A |
| No-load speed | 8050 rpm |
| Pole pairs | 8 |

## Development Workflow

### After CubeMX Changes

1. Open `invpend_BLDC.ioc` in STM32CubeMX
2. Make changes (add peripherals, modify pins)
3. Generate code with "Keep User Code" option
4. Verify `USER CODE` sections preserved
5. Rebuild project

### Adding New Functionality

1. Add source files to `Src/`, headers to `Inc/`
2. Update `cmake/stm32cubemx/CMakeLists.txt` if needed
3. Include headers in `main.c` within `USER CODE BEGIN Includes`
4. Initialize in `USER CODE BEGIN 2` section
5. Call from main loop in `USER CODE BEGIN 3` section

### Key Integration Points

```c
// In main.c USER CODE BEGIN 2:
Pendulum_Init();
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
SPI_StartCommunication();
MC_ProgramTorqueRampMotor1_F(0.0f, 0);

// In main.c while loop (USER CODE BEGIN 3):
StateMachine_Run();
```

## Safety Considerations

- **Current limiting**: Hard-coded ±5A in `ApplyTorqueCommand()`
- **OCP**: Hardware overcurrent protection via PA6 (TIM1_BKIN)
- **Error state**: Motor stops on STATE_ERROR
- **Watchdog**: SPI timeout detection (1.5× sample time)

## Dependencies

- **STM32 HAL**: F4 series HAL drivers
- **Motor Control SDK**: v6.4.1 (MCSDK)
- **Power Board**: X-NUCLEO-IHM08M1
- **Motor**: maxon ECX FLAT 42 M (ECXFL42M KL A HTQ 24V)
- **Motor Encoder**: maxon ENX 42 MILE 2048IMP (differential output)
- **Line Receiver**: AM26C32, SN75175, or equivalent (for differential encoder)

## Debugging

- **UART2** (PA2/PA3): 921600 baud for MC Workbench / logging
- **DWT Cycle Counter**: Use `Chrono_*` functions for timing
- **SPI Flags**: Check `spi_txrx_flag`, `spi_err_flag` for comm status
- **Motor State**: Use MC Workbench to monitor FOC variables

## Common Issues

| Issue | Solution |
|-------|----------|
| SPI not working | Check NSS is hardware mode, DMA streams not conflicting |
| Encoder not counting | Verify pull-ups enabled, check TIM3 encoder mode |
| Motor not responding | Ensure `MC_StartMotor1()` called, check motor state |
| Timing drift | Verify 84 MHz clock from HSE, check `Chrono_Init()` |
| No UART output in test mode | Fixed in state machine - test mode now skips SPI wait |
| Torque shows ~47mNm when motor off | Normal - ADC offset not calibrated without motor running |
| User button starts motor | Override `UI_HandleStartStopButton_cb()` (already done) |

## Build and Flash

```bash
# Build
cmake --build build/Debug

# Flash via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_BLDC.elf verify reset exit"

# Or flash via STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_BLDC.elf -v -rst
```
