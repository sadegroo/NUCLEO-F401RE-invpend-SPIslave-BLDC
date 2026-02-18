/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Application configuration and debug settings
  *
  * Central configuration file for compile-time options, debug flags, and
  * test modes. Modify these defines to enable/disable features.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Debug and Test Configuration
 ******************************************************************************/

/**
  * @brief  Test mode - disable motor power stage
  *
  * When set to 1, the motor power stage is completely disabled:
  * - MC_StartMotor1() is never called
  * - All torque commands are ignored (treated as 0)
  * - Motor will NOT spin even if SPI receives a torque command
  *
  * Use this mode when testing encoder and SPI communication without
  * risking unintended motor movement. Set to 0 for normal operation.
  */
#define TEST_MODE_NO_MOTOR          0

/**
  * @brief  Skip waiting for SPI master
  *
  * When set to 1, the state machine runs independently without waiting for
  * SPI transactions from the Raspberry Pi. Useful for testing encoder
  * functionality without the Pi connected.
  *
  * Set to 0 to enable normal SPI-synchronized operation.
  */
#define SKIP_SPI_WAIT               0

/**
  * @brief  Enable pendulum encoder debug output via UART
  *
  * When set to 1, the pendulum encoder count is periodically printed to UART2
  * (ST-Link virtual COM port at 921600 baud). This is useful for testing the
  * encoder connection without needing the Raspberry Pi or motor encoder.
  *
  * Set to 0 for normal operation (no debug output).
  */
#define DEBUG_PENDULUM_ENCODER      0

/**
  * @brief  Debug print interval in milliseconds
  *
  * Controls how often the pendulum encoder count is printed when
  * DEBUG_PENDULUM_ENCODER is enabled. Default is 100ms (10 Hz).
  */
#define DEBUG_PRINT_INTERVAL_MS     100

/**
  * @brief  Enable motor control state debug output via UART
  *
  * When set to 1, motor state, faults, and initialization status are printed
  * to UART2 periodically and on state changes. Useful for diagnosing motor
  * control issues.
  *
  * Output format: MC:<state> init:<0/1> flt:0x<hex> req:<0/1>
  * States: 0=IDLE, 4=ALIGNMENT, 6=RUN, 10=FAULT_NOW, 11=FAULT_OVER
  */
#define DEBUG_MOTOR                 1

/**
  * @brief  Button-triggered torque test mode
  *
  * When set to 1, pressing the blue user button (PC13) applies a fixed
  * torque for a set duration, overriding any SPI torque commands.
  * Useful for testing motor movement without requiring RPi control.
  *
  * Set to 0 for normal SPI-controlled operation.
  */
#define TEST_MODE_TORQUE_BUTTON     0

/**
  * @brief  Automatic motor initialization on boot
  *
  * When set to 1, the motor starts automatically after Pendulum_Init()
  * without requiring user button press. Useful when Pi controls startup.
  *
  * When set to 0, motor requires user button press to start.
  */
#define AUTO_MOTOR_INIT             1

/*******************************************************************************
 * Motor Velocity Calculation
 ******************************************************************************/

/**
  * @brief  Motor velocity filter coefficient (exponential moving average)
  *
  * Range: 0.01 to 1.0
  * - Lower values (0.01-0.1): More filtering, slower response, less noise
  * - Higher values (0.5-1.0): Less filtering, faster response, more noise
  *
  * Default: 0.1 (smooth output, ~10 sample time constant)
  */
#define MOTOR_VEL_FILTER_ALPHA      0.1f

/**
  * @brief  Pendulum velocity filter coefficient (exponential moving average)
  *
  * Same range and behavior as MOTOR_VEL_FILTER_ALPHA.
  */
#define PEND_VEL_FILTER_ALPHA       0.1f

/**
  * @brief  Velocity output resolution divisor
  *
  * The internal velocity (counts/second) is divided by this factor before
  * sending over SPI. This reduces resolution but increases range.
  *
  * Example: With divisor=2, internal velocity of 1000 counts/s outputs as 500
  */
#define MOTOR_VEL_RESOLUTION_DIV    4
#define PEND_VEL_RESOLUTION_DIV     1

/*******************************************************************************
 * Dual Overspeed Protection
 *
 * Two independent windows for speed limiting:
 * - Window 1: Short burst limit (allows brief high-speed bursts)
 * - Window 2: Sustained speed limit (limits average speed over longer distance)
 *
 * Both trigger the same fault behavior (requires Pi to send zero torque to clear).
 * 8192 counts/rev: RPM = cps * 60 / 8192
 ******************************************************************************/

/**
  * @brief  Window 1: Short burst limit (0.25 rev, 300 RPM)
  *
  * Allows brief high-speed bursts. Triggers fault if 300+ RPM sustained
  * for 0.25 revolutions (2048 counts).
  * 300 RPM = 40960 cps
  */
#define OVERSPEED1_THRESHOLD_CPS    40960
#define OVERSPEED1_WINDOW_REVS      0.25f

/**
  * @brief  Window 2: Sustained speed limit (0.5 rev, 150 RPM)
  *
  * Limits average speed over longer distance. Triggers fault if 150+ RPM
  * sustained for 0.5 revolutions (4096 counts).
  * 150 RPM = 20480 cps
  */
#define OVERSPEED2_THRESHOLD_CPS    20480
#define OVERSPEED2_WINDOW_REVS      0.5f

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
