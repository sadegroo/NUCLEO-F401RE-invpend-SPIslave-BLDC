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

8-byte bidirectional exchange at 1 kHz:

**STM32 → Raspberry Pi:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Pendulum position (encoder counts) |
| 2-3 | int16 | Motor position (electrical angle) |
| 4-5 | int16 | Motor velocity |
| 6-7 | int16 | Measured motor torque (milli-Nm) |

**Raspberry Pi → STM32:**
| Bytes | Type | Description |
|-------|------|-------------|
| 0-1 | int16 | Torque command (milli-Nm) |
| 2-3 | int16 | Reserved |
| 4-5 | int16 | Reserved |
| 6-7 | int16 | Reserved |

### Building

```bash
# Configure
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug

# Build
cmake --build build/Debug

# Flash via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/invpend_BLDC.elf verify reset exit"
```

### Test Mode

For testing without the Raspberry Pi or motor, edit `Inc/app_config.h`:

```c
#define TEST_MODE_NO_MOTOR      1   // Disable motor (safe for testing)
#define SKIP_SPI_WAIT           1   // Run without Pi (set 0 for SPI test)
#define DEBUG_PENDULUM_ENCODER  1   // Print to UART (921600 baud)
```

Configuration options:
- `TEST_MODE_NO_MOTOR=1`: Motor disabled, torque commands ignored
- `SKIP_SPI_WAIT=1`: Run without Pi connected
- `SKIP_SPI_WAIT=0`: Wait for SPI from Pi (use for SPI testing)
- Debug output format: `Pend:<encoder> torSP:<cmd> torCV:<measured>`

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
