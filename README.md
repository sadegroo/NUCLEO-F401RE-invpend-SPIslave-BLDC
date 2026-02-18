# Inverted Pendulum BLDC Motor Control

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/STM32-F401RE-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
[![Motor Control SDK](https://img.shields.io/badge/MC%20SDK-v6.4.1-green)](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)

Real-time inverted pendulum control system using a BLDC motor with Field-Oriented Control (FOC). This project is based on the [STEVAL-EDUKIT01](https://www.st.com/en/evaluation-tools/steval-edukit01.html) educational platform, modified to use a BLDC motor instead of the original stepper motor.

> **Hardware Platform**: See the [STEVAL-EDUKIT01 product page](https://www.st.com/en/evaluation-tools/steval-edukit01.html) for images and specifications of the rotary inverted pendulum kit.

## Overview

This project implements a 1 kHz control loop for stabilizing an inverted pendulum. The STM32 microcontroller acts as a low-level real-time controller, receiving torque commands from a Raspberry Pi via SPI and providing sensor feedback.

### System Architecture

```
┌─────────────────────────────────┐
│      Raspberry Pi               │  ← High-level controller (LQR/MPC)
│      (SPI Master)               │
└───────────────┬─────────────────┘
                │ SPI @ 1 kHz
                │ (torque cmd ↓ / sensor data ↑)
┌───────────────▼─────────────────┐
│      NUCLEO-F401RE              │  ← Real-time low-level control
│      + X-NUCLEO-IHM08M1         │     FOC motor control @ 16 kHz
└───────────────┬─────────────────┘
                │
        ┌───────┴───────┐
        │               │
   ┌────▼────┐    ┌─────▼─────┐
   │  BLDC   │    │ Pendulum  │
   │  Motor  │    │ Encoder   │
   │ +Encoder│    │ (2400 CPR)│
   └─────────┘    └───────────┘
```

## Hardware

### Based on STEVAL-EDUKIT01

This project uses the mechanical structure from the [STEVAL-EDUKIT01](https://www.st.com/en/evaluation-tools/steval-edukit01.html) rotary inverted pendulum kit, which includes:

- Transparent acrylic frame with pendulum arm
- High-precision quadrature encoder (2400 CPR) for pendulum angle
- Sturdy base with motor mount

> **Note**: The original kit uses a stepper motor (L6474). This project replaces it with a BLDC motor for smoother torque control.

### Components Used

| Component | Description |
|-----------|-------------|
| **[NUCLEO-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)** | STM32F401RE development board (84 MHz Cortex-M4) |
| **[X-NUCLEO-IHM08M1](https://www.st.com/en/ecosystems/x-nucleo-ihm08m1.html)** | 3-phase BLDC motor driver (STL220N6F7) |
| **maxon ECX FLAT 42 M** | BLDC motor (24V, 214 mNm, 8 pole pairs) |
| **maxon ENX 42 MILE** | Incremental encoder (2048 CPR, differential output) |
| **Raspberry Pi** | High-level controller (any model with SPI) |

### Pin Mapping

| Function | STM32 Pin | Notes |
|----------|-----------|-------|
| Pendulum Encoder A | PB4 (D5) | TIM3_CH1 |
| Pendulum Encoder B | PB5 (D4) | TIM3_CH2 |
| Motor Encoder A | PA15 | TIM2_CH1 (via line receiver) |
| Motor Encoder B | PB3 (D3) | TIM2_CH2 (via line receiver) |
| SPI3 NSS | PA4 | Hardware slave select |
| SPI3 SCK | PC10 | Clock from RPi |
| SPI3 MISO | PC11 | Data to RPi |
| SPI3 MOSI | PC12 | Data from RPi |

## Software

### Features

- **Field-Oriented Control (FOC)** at 16 kHz for smooth torque control
- **1 kHz control loop** synchronized with Raspberry Pi
- **SPI slave communication** with DMA (circular buffer)
- **Big-endian protocol** for cross-platform compatibility
- **Safety features**: current limiting, overcurrent protection, timeout detection

### SPI Protocol

10-byte bidirectional exchange at 1 kHz (big-endian):

**STM32 → Raspberry Pi:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Pendulum position (encoder counts) |
| 2-3 | int16 | Pendulum velocity (counts/sec / DIV) |
| 4-5 | int16 | Motor position (encoder counts) |
| 6-7 | int16 | Motor velocity (counts/sec / DIV) |
| 8-9 | int16 | Measured motor torque (milli-Nm) |

**Raspberry Pi → STM32:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Torque command (milli-Nm) |
| 2-9 | - | Reserved |

> **Note**: Velocities are divided by `MOTOR_VEL_RESOLUTION_DIV` (default: 2) before transmission. Filter coefficients configurable via `MOTOR_VEL_FILTER_ALPHA` and `PEND_VEL_FILTER_ALPHA` in `app_config.h`.

### Building and Flashing

#### Build Commands

```bash
# Configure and build (Debug)
cmake --preset Debug
cmake --build build/Debug

# Or configure and build (Release)
cmake --preset Release
cmake --build build/Release

# Clean build
cmake --build build/Debug --target clean
```

#### Flash via OpenOCD

```bash
# Flash Debug build
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_BLDC.elf verify reset exit"

# Flash Release build
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Release/invpend_BLDC.elf verify reset exit"
```

#### Flash via STM32CubeProgrammer CLI

```bash
# Flash Debug build
STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_BLDC.elf -v -rst

# Flash Release build
STM32_Programmer_CLI -c port=SWD -w build/Release/invpend_BLDC.elf -v -rst

# Erase and flash
STM32_Programmer_CLI -c port=SWD -e all -w build/Debug/invpend_BLDC.elf -v -rst
```

#### One-liner Build and Flash

```bash
# Build and flash in one command (OpenOCD)
cmake --build build/Debug && openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_BLDC.elf verify reset exit"

# Build and flash in one command (STM32CubeProgrammer)
cmake --build build/Debug && STM32_Programmer_CLI -c port=SWD -w build/Debug/invpend_BLDC.elf -v -rst
```

#### VS Code Keybindings

Pre-configured tasks in `.vscode/tasks.json` with keybindings:

| Hotkey | Action |
|--------|--------|
| `Ctrl+Shift+B` | Build (Debug) |
| `Ctrl+Shift+F5` | Build and Flash |
| `Ctrl+Alt+F` | Flash only |

### Motor Initialization

The motor can be initialized via:
1. **Auto-init on boot** (`AUTO_MOTOR_INIT=1` in `app_config.h`) - motor starts automatically
2. **User button press** - blue button (PC13) toggles motor on/off

**Button behavior** (with 200ms debounce):
- Motor OFF → press button → motor starts
- Motor ON → press button → motor stops

**Motor status indication via SPI**:
- Measured torque = `-999` means motor not ready (not initialized or in fault state)
- Real torque values sent only when motor is in RUN state without faults

### Test Mode

For testing without the Raspberry Pi or motor, edit `Inc/app_config.h`:

```c
#define TEST_MODE_NO_MOTOR      1   // Disable motor (safe for testing)
#define SKIP_SPI_WAIT           1   // Run without Pi (set 0 for SPI test)
#define DEBUG_PENDULUM_ENCODER  1   // Print to UART (921600 baud)
#define DEBUG_MOTOR             1   // Print motor state/faults to UART
#define TEST_MODE_TORQUE_BUTTON 0   // Button-triggered torque test
#define AUTO_MOTOR_INIT         1   // Auto-start motor on boot
```

Configuration options:
- `TEST_MODE_NO_MOTOR=1`: Motor disabled, torque commands ignored
- `SKIP_SPI_WAIT=1`: Run without Pi connected
- `SKIP_SPI_WAIT=0`: Wait for SPI from Pi (use for SPI testing)
- `DEBUG_MOTOR=1`: Print motor state and faults to UART
- `TEST_MODE_TORQUE_BUTTON=1`: Blue button applies +20 mNm for 5 seconds
- `AUTO_MOTOR_INIT=1`: Motor starts automatically on boot
- Debug output format: `Pend:<encoder> torSP:<cmd> torCV:<measured>`

#### Button Torque Test Mode

When `TEST_MODE_TORQUE_BUTTON=1`:
1. Press the blue user button (PC13)
2. Motor starts and applies +20 mNm torque for 5 seconds
3. After 5 seconds, torque returns to zero
4. Useful for testing motor without Raspberry Pi control

### Dependencies

- [STM32CubeF4](https://www.st.com/en/embedded-software/stm32cubef4.html) HAL drivers
- [X-CUBE-MCSDK v6.4.1](https://www.st.com/en/embedded-software/x-cube-mcsdk.html) Motor Control SDK
- ARM GCC toolchain (arm-none-eabi-gcc)
- CMake 3.20+

## Documentation

Detailed documentation is available in the [`docs/`](docs/) folder:

| Document | Description |
|----------|-------------|
| [claude-instructions.md](docs/claude-instructions.md) | Complete wiring guide and technical reference |
| [INTEGRATION_MANUAL_STEPS.md](docs/INTEGRATION_MANUAL_STEPS.md) | CubeMX configuration steps |
| [pinout_F401RE.png](docs/pinout_F401RE.png) | NUCLEO board pinout diagram |

### External Resources

- [STEVAL-EDUKIT01 Product Page](https://www.st.com/en/evaluation-tools/steval-edukit01.html)
- [STEVAL-EDUKIT01 Datasheet (PDF)](https://www.st.com/resource/en/data_brief/steval-edukit01.pdf)
- [STEVAL-EDUKIT01 User Manual (PDF)](https://www.st.com/resource/en/user_manual/um2703-getting-started-with-the-stevaledukit01-rotary-inverted-pendulum-kit-stmicroelectronics.pdf)
- [X-NUCLEO-IHM08M1 User Manual](https://www.st.com/resource/en/user_manual/um1996-getting-started-with-xnucleoihm08m1-lowvoltage-bldc-motor-driver-expansion-board-based-on-stl220n6f7-for-stm32-nucleo-stmicroelectronics.pdf)
- [Motor Control SDK Documentation](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)

## Getting Started

1. **Hardware Setup**
   - Mount the BLDC motor on the STEVAL-EDUKIT01 frame
   - Stack X-NUCLEO-IHM08M1 on NUCLEO-F401RE
   - Connect motor phases (U, V, W) to J2 terminals
   - Connect motor encoder via differential line receiver (AM26C32)
   - Connect pendulum encoder to PB4/PB5
   - Wire SPI to Raspberry Pi (see [wiring guide](docs/claude-instructions.md))

2. **Flash Firmware**
   - Build the project using CMake
   - Flash `invpend_BLDC.elf` to the NUCLEO board

3. **Run Control Algorithm**
   - Implement your controller on the Raspberry Pi
   - Send torque commands via SPI at 1 kHz
   - Read sensor feedback for closed-loop control

## Motor Specifications

| Parameter | Value |
|-----------|-------|
| Motor | maxon ECX FLAT 42 M |
| Nominal Voltage | 24 V |
| Torque Constant | 28.2 mNm/A |
| Nominal Torque | 214 mNm |
| Nominal Current | 7.33 A |
| Pole Pairs | 8 |
| Encoder | 2048 CPR (differential) |

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [STMicroelectronics](https://www.st.com) for the STEVAL-EDUKIT01 platform and Motor Control SDK
- [maxon motor](https://www.maxongroup.com) for motor and encoder specifications

---

*For questions or issues, please open a GitHub issue.*
