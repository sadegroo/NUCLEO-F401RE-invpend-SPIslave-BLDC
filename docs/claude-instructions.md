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
| [Inc/app_config.h](Inc/app_config.h) | Test modes, velocity filter alphas, overspeed thresholds |
| [Inc/pendulum_control.h](Inc/pendulum_control.h) | State machine states, SPI protocol (10 bytes), VelocityCalc_t |
| [Inc/torque_control.h](Inc/torque_control.h) | `MOTOR_KT_NM_PER_A = 0.0234`, `MAX_CURRENT_A = 5.0` |
| [Inc/chrono.h](Inc/chrono.h) | `RCC_SYS_CLOCK_FREQ = 84000000` |

### Debug Configuration

Debug and test modes are configured in `Inc/app_config.h`:

```c
// Test mode: disable motor power stage (motor cannot spin)
#define TEST_MODE_NO_MOTOR          1

// Skip waiting for SPI master (run state machine independently)
#define SKIP_SPI_WAIT               0

// Enable pendulum encoder debug output via UART (set to 0 to disable)
#define DEBUG_PENDULUM_ENCODER      1

// Debug print interval in milliseconds
#define DEBUG_PRINT_INTERVAL_MS     100

// Button-triggered torque test: press blue button to apply +20 mNm for 5 seconds
#define TEST_MODE_TORQUE_BUTTON     0

// Motor control state debug output via UART
#define DEBUG_MOTOR                 1

// Auto-start motor on boot (1) or require button press (0)
#define AUTO_MOTOR_INIT             1
```

**TEST_MODE_NO_MOTOR**: When enabled:
- Motor power stage is completely disabled
- All torque commands are ignored (ApplyTorqueCommand returns immediately)
- User button (PC13) is disabled (won't start/stop motor)
- Useful for testing SPI communication without motor risk

**SKIP_SPI_WAIT**: When enabled:
- State machine runs independently without waiting for SPI transactions
- Loops at 1 kHz using internal timing instead of SPI sync
- Useful for testing encoder without Raspberry Pi connected

**DEBUG_PENDULUM_ENCODER**: When enabled:
- Debug info printed to UART2 (921600 baud) at 10 Hz
- Format: `Pend:<encoder> torSP:<commanded> torCV:<measured>`
- Useful for verifying encoder and SPI communication

**TEST_MODE_TORQUE_BUTTON**: When enabled:
- Blue button (PC13) triggers a torque test
- Motor starts and applies +20 mNm for 5 seconds
- Overrides SPI torque commands during test
- Useful for testing motor without Raspberry Pi

**DEBUG_MOTOR**: When enabled:
- Prints motor state, faults, and initialization status to UART2 (921600 baud)
- Output format: `MC:<state> init:<0/1> flt:0x<hex> req:<0/1>`
- Prints every 500ms or on state change

**AUTO_MOTOR_INIT**: When enabled:
- Motor initialization starts automatically after `Pendulum_Init()`
- No button press required to start motor
- Useful when Raspberry Pi controls startup timing

### Motor State Codes (MCI_State_t)

| Value | State | Description |
|-------|-------|-------------|
| 0 | IDLE | Motor stopped, ready to start |
| 4 | ALIGNMENT | Rotor alignment phase |
| 6 | RUN | Motor running normally |
| 10 | FAULT_NOW | Active fault condition |
| 11 | FAULT_OVER | Fault acknowledged, motor stopped |

### Motor Fault Codes (MC_GetOccurredFaultsMotor1)

| Code | Name | Description |
|------|------|-------------|
| 0x0001 | MC_DURATION | FOC execution rate too high |
| 0x0002 | MC_OVER_VOLT | Bus overvoltage |
| 0x0004 | MC_UNDER_VOLT | Bus undervoltage |
| 0x0008 | MC_OVER_TEMP | Overtemperature |
| 0x0010 | MC_START_UP | Startup failure |
| 0x0020 | MC_SPEED_FDBK | Speed feedback error |
| 0x0040 | MC_BREAK_IN | Break input triggered |
| 0x0080 | MC_SW_ERROR | Software error |
| 0x0100 | MC_OVER_CURR | Overcurrent detected |

### Motor Initialization Tracking

The firmware sends `-999` for measured torque when motor cannot deliver torque:
- `motor_initialized = 0`: Motor not initialized
- Motor state is `FAULT_NOW` or `FAULT_OVER`: Motor in fault condition

Real torque values are only sent when motor is initialized AND in RUN state (no faults).

This allows the Raspberry Pi to detect when the motor is ready:
```python
if measured_torque == -999:
    print("Motor not ready - not initialized or in fault state")
else:
    print(f"Motor ready, torque: {measured_torque} mNm")
```

### Button Toggle Behavior

The blue user button (PC13) toggles motor state with 200ms debounce:
- Motor OFF → press → motor starts (via `ProcessMotorStartup()`)
- Motor ON → press → motor stops (via `MC_StopMotor1()`)

### Configuration Combinations

| TEST_MODE_NO_MOTOR | SKIP_SPI_WAIT | TEST_MODE_TORQUE_BUTTON | Use Case |
|--------------------|---------------|-------------------------|----------|
| 1 | 1 | 0 | Encoder test without Pi or motor |
| 1 | 0 | 0 | SPI test with Pi, motor disabled (safe) |
| 0 | 0 | 0 | Normal operation (Pi controls motor) |
| 0 | 0 | 1 | Button torque test with Pi connected |
| 0 | 1 | 1 | Button torque test without Pi |

### Test Mode Behavior

When `SKIP_SPI_WAIT=1`, the state machine skips SPI synchronization:

```
Normal mode (SKIP_SPI_WAIT=0):        Skip SPI mode (SKIP_SPI_WAIT=1):
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

**Buffer size**: 10 bytes, **Endianness**: Big-endian

### RX from Raspberry Pi
```
Bytes [0-1]: int16_t torque_cmd      // Torque command in milli-Nm
Bytes [2-9]: reserved                // Future use
```

### TX to Raspberry Pi
```
Bytes [0-1]: int16_t pendulum_pos    // Pendulum encoder position (counts)
Bytes [2-3]: int16_t pendulum_vel    // Pendulum velocity (counts/sec / DIV)
Bytes [4-5]: int16_t motor_pos       // Motor encoder position (counts)
Bytes [6-7]: int16_t motor_vel       // Motor velocity (counts/sec / DIV)
Bytes [8-9]: int16_t measured_torque // Actual motor torque in milli-Nm
```

**Note**: Velocities are divided by `MOTOR_VEL_RESOLUTION_DIV` (default: 2) before transmission.

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

| Component | Frequency | Priority | Notes |
|-----------|-----------|----------|-------|
| System Clock | 84 MHz | - | |
| TIM1 Update (PWM) | 16 kHz | 0 | Highest priority |
| ADC (FOC calculation) | 16 kHz | 2 | **Must not be preempted** |
| SPI3 DMA (RX/TX) | RPi-driven (~1 kHz) | **3** | Below ADC to avoid MC_DURATION fault |
| USART2 | 921600 baud | 3 | Owned by MC SDK (ASPEP) |
| Pendulum Control Loop | 1 kHz target | main loop | |

### Critical: Interrupt Priority Configuration

**MC_DURATION Fault Prevention**: The SPI DMA interrupts (DMA1_Stream0, DMA1_Stream7) must have **lower priority than ADC** (higher number = lower priority on Cortex-M). If SPI DMA preempts the FOC calculation in ADC_IRQHandler, it causes MC_DURATION fault (0x0001).

```c
// In MX_DMA_Init() - main.c
HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);  // SPI3 RX - priority 3
HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);  // SPI3 TX - priority 3
// ADC_IRQn is at priority 2 - FOC runs here
```

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

### Dual Overspeed Protection

Two independent speed limit windows (configured in `app_config.h`):

| Window | Threshold | Distance | Purpose |
|--------|-----------|----------|---------|
| Window 1 | 300 RPM (40960 cps) | 0.25 rev | Allow brief high-speed bursts |
| Window 2 | 150 RPM (20480 cps) | 0.5 rev | Limit sustained speed |

**Behavior**:
- If speed exceeds threshold for the window distance, `overspeed_fault_active = 1`
- Torque command forced to 0 while fault is active
- Fault clears when Raspberry Pi sends torque command = 0 (acknowledgment)

**Conversion**: RPM = cps × 60 / 8192 (8192 counts per rev)

### MC SDK Speed Measurement Errors

The MC SDK has a speed feedback reliability check:
```c
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3  // In drive_parameters.h
```

This means 3 consecutive speed measurement errors trigger `MC_SPEED_FDBK` fault (0x0020). When motor sits still with zero torque, encoder noise can accumulate errors. Consider increasing this value if motor faults unexpectedly at standstill.

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
| No UART output in test mode | Set `SKIP_SPI_WAIT=1` or connect Pi SPI master |
| Torque shows ~47mNm when motor off | Normal - ADC offset not calibrated without motor running |
| User button starts motor | Override `UI_HandleStartStopButton_cb()` (already done) |
| Simulink display shows 0 | Display block may not be connected to signal - check signal routing |
| MC_DURATION fault (0x0001) | SPI DMA priority too high - set to 3, below ADC (2) |
| UART freezes system | Don't use HAL_UART_Transmit_IT - use blocking with short timeout |
| Motor position saturates quickly | Use `SPD_GetMecAngle()` not `MC_GetElAngledppMotor1()` |
| Motor de-inits unexpectedly | Enable `DEBUG_MOTOR=1` to see fault codes; likely `MC_SPEED_FDBK` (0x0020) - increase `M1_SS_MEAS_ERRORS_BEFORE_FAULTS` in drive_parameters.h |
| Button press triggers twice | Button debounce already 200ms - check for hardware issues |
| Overspeed fault won't clear | Pi must send torque_cmd=0 to acknowledge and clear fault |

### UART Limitation (MC SDK Conflict)

**USART2 is owned by the Motor Control SDK** for ASPEP protocol communication. The USART2_IRQHandler in `stm32_mc_common_it.c` does NOT call `HAL_UART_IRQHandler()`, so **`HAL_UART_Transmit_IT()` will not work** (callbacks never fire, HAL state machine hangs).

**Solution**: Use blocking `HAL_UART_Transmit()` with short timeout (1ms):
```c
HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 1);  // 1ms timeout
```

### Motor Encoder Position

The motor has 8 pole pairs, so electrical angle wraps 8x per mechanical revolution. Use mechanical angle for position tracking:

```c
// Wrong - saturates within ~45° of rotation:
int16_t motor_pos = MC_GetElAngledppMotor1();  // Electrical angle

// Correct - 8192 counts per revolution:
int32_t mec_angle_raw = SPD_GetMecAngle(&ENCODER_M1._Super);  // 65536 counts/rev
int16_t motor_pos = (int16_t)(mec_angle_raw >> 3);            // 8192 counts/rev
```

Requires:
```c
#include "encoder_speed_pos_fdbk.h"
extern ENCODER_Handle_t ENCODER_M1;
```

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

### VS Code Keybindings

Pre-configured in user keybindings:

| Hotkey | Task | Description |
|--------|------|-------------|
| `Ctrl+Shift+B` | Build | Compile Debug build |
| `Ctrl+Shift+F5` | Build and Flash | Compile then flash via OpenOCD |
| `Ctrl+Alt+F` | Flash | Flash only (no rebuild) |

Tasks are defined in `.vscode/tasks.json`. Flash uses relative path with `cwd` set to workspace folder to avoid Windows path escaping issues with OpenOCD.
