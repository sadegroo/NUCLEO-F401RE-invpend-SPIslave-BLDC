# Integration Manual Steps

This document describes the manual configuration steps required to complete the integration of pendulum control into the BLDC motor control project.

## Step 1: CubeMX Configuration

Open `invpend_BLDC.ioc` in STM32CubeMX and make the following changes:

### 1.1 Add SPI3 Peripheral

1. Navigate to **Connectivity → SPI3**
2. Set Mode: **Full-Duplex Slave**
3. In Configuration:
   - Hardware NSS Signal: **Hardware NSS Input Signal**
   - Frame Format: **Motorola**
   - Data Size: **8 bits**
   - First Bit: **MSB First**
   - Clock Polarity (CPOL): **Low**
   - Clock Phase (CPHA): **1 Edge**

### 1.2 Configure SPI3 DMA

1. In SPI3 Configuration, go to **DMA Settings** tab
2. Click **Add** and configure:
   - **SPI3_RX**:
     - Stream: DMA1 Stream 0
     - Direction: Peripheral to Memory
     - Priority: Very High
     - Mode: **Circular**
     - Data Width: Byte / Byte
   - **SPI3_TX**:
     - Stream: **DMA1 Stream 7** (NOT Stream 5 - that's used by USART2)
     - Direction: Memory to Peripheral
     - Priority: Very High
     - Mode: **Circular**
     - Data Width: Byte / Byte

### 1.3 Configure SPI3 GPIO Pins

Verify these pins are configured (should be automatic):
- **PA4**: SPI3_NSS
- **PC10**: SPI3_SCK
- **PC11**: SPI3_MISO
- **PC12**: SPI3_MOSI

### 1.4 Add TIM3 for Pendulum Encoder

1. Navigate to **Timers → TIM3**
2. Set Combined Channels: **Encoder Mode**
3. In Configuration → Parameter Settings:
   - Encoder Mode: **Encoder Mode TI1 and TI2**
   - Counter Period: **4294967295** (32-bit max)
   - Auto-Reload Preload: Disable
   - Input Filter: 0 (or small value like 4 for noise filtering)

### 1.5 Configure TIM3 GPIO Pins

1. In **Pinout view**, configure:
   - **PB4**: TIM3_CH1 (Encoder A)
   - **PB5**: TIM3_CH2 (Encoder B)
2. For both pins, set:
   - GPIO Pull-up/Pull-down: **Pull-up**

### 1.6 Configure NVIC (Interrupts)

1. Navigate to **System Core → NVIC**
2. Enable these interrupts:
   - **DMA1 Stream0 global interrupt**: Priority 5
   - **DMA1 Stream7 global interrupt**: Priority 5
3. Verify SPI3 global interrupt is enabled if needed (usually DMA handles it)

### 1.7 Generate Code

1. Click **Generate Code** (or press Ctrl+Shift+G)
2. When prompted about USER CODE sections, select **Keep**
3. CubeMX will add:
   - SPI3 initialization in `main.c`
   - DMA initialization
   - TIM3 initialization
   - GPIO configuration
   - HAL_SPI driver to the build

---

## Step 2: Modify main.c

After CubeMX code generation, add the following to `Src/main.c`:

### 2.1 Add Includes (USER CODE BEGIN Includes)

```c
/* USER CODE BEGIN Includes */
#include "pendulum_control.h"
#include "torque_control.h"
#include "chrono.h"
/* USER CODE END Includes */
```

### 2.2 Add Initialization (USER CODE BEGIN 2)

Add this code after all MX_xxx_Init() calls and before the main loop:

```c
/* USER CODE BEGIN 2 */
/* Initialize pendulum control subsystem */
Pendulum_Init();

/* Start TIM3 encoder for pendulum */
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

/* Start SPI DMA circular communication */
SPI_StartCommunication();

/* Start motor in torque mode with zero initial torque */
MC_ProgramTorqueRampMotor1_F(0.0f, 0);

/* Note: Motor should be started when RPi commands it or via button
 * Uncomment next line to auto-start motor:
 * MC_StartMotor1();
 */
/* USER CODE END 2 */
```

### 2.3 Add State Machine to Main Loop (USER CODE BEGIN 3)

```c
/* USER CODE BEGIN 3 */
    /* Run pendulum control state machine (1 kHz target rate) */
    StateMachine_Run();
/* USER CODE END 3 */
```

### 2.4 Complete main.c Example

The while loop should look like this:

```c
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
    /* Run pendulum control state machine */
    StateMachine_Run();
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

---

## Step 3: Verify stm32f4xx_it.c

CubeMX should automatically add DMA interrupt handlers. Verify these exist in `Src/stm32f4xx_it.c`:

```c
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}
```

If not present, add them manually.

---

## Step 4: Build and Test

### 4.1 Build the Project

```bash
cd build/Debug
cmake --build .
```

Or use the VS Code build task.

### 4.2 Verify No Compilation Errors

Common issues:
- Missing `hspi3` or `htim3` handles → CubeMX didn't generate properly
- Missing HAL_SPI functions → Check stm32f4xx_hal_conf.h enables SPI module

### 4.3 Flash and Test

1. Flash the firmware
2. Connect logic analyzer to SPI3 pins (PA4, PC10-12)
3. Connect pendulum encoder to PB4, PB5
4. Test with RPi SPI master

---

## Step 5: Testing Checklist

### Hardware Verification
- [ ] SPI3 NSS (PA4) toggles when RPi sends data
- [ ] SPI3 SCK (PC10) shows clock from RPi
- [ ] SPI3 MISO (PC11) shows data from STM32
- [ ] Pendulum encoder counts change when rotated (check via debugger)
- [ ] Motor encoder still works (MC Workbench shows position)

### Software Verification
- [ ] State machine cycles through states (add breakpoints or GPIO toggle)
- [ ] Torque commands produce motor current (check MC Workbench Iq value)
- [ ] SPI data is correct endianness (verify with RPi)

### Timing Verification
- [ ] Toggle GPIO in STATE_READ to measure loop rate (~1 kHz)
- [ ] Verify no DMA errors (spi_err_flag remains 0)

---

## Troubleshooting

### SPI Not Working
1. Check NSS pin is configured as hardware input (not GPIO)
2. Verify DMA streams are not conflicting (Stream 7 for TX, not Stream 5)
3. Check SPI clock polarity/phase matches RPi settings

### Encoder Not Counting
1. Verify TIM3 is in Encoder Mode TI1 and TI2
2. Check PB4/PB5 are configured with pull-ups
3. Verify encoder cable connections (A→PB4, B→PB5)

### Motor Not Responding to Torque
1. Ensure motor is started: `MC_StartMotor1()`
2. Check motor state in MC Workbench (should be RUN)
3. Verify torque constant (Kt) in torque_control.h

### DMA Errors
1. Check DMA stream assignments (no conflicts)
2. Verify buffer sizes match (6 bytes)
3. Check circular mode is enabled for both RX and TX

---

## Pin Summary

| Function | Pin | Peripheral |
|----------|-----|------------|
| Pendulum Encoder A | PB4 | TIM3_CH1 |
| Pendulum Encoder B | PB5 | TIM3_CH2 |
| SPI NSS (to RPi) | PA4 | SPI3_NSS |
| SPI SCK (from RPi) | PC10 | SPI3_SCK |
| SPI MISO (to RPi) | PC11 | SPI3_MISO |
| SPI MOSI (from RPi) | PC12 | SPI3_MOSI |
| Motor Encoder A | PA15 | TIM2_CH1 (unchanged) |
| Motor Encoder B | PB3 | TIM2_CH2 (unchanged) |
