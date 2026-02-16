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
#define DEBUG_PENDULUM_ENCODER      1

/**
  * @brief  Debug print interval in milliseconds
  *
  * Controls how often the pendulum encoder count is printed when
  * DEBUG_PENDULUM_ENCODER is enabled. Default is 100ms (10 Hz).
  */
#define DEBUG_PRINT_INTERVAL_MS     100

/**
  * @brief  Button-triggered torque test mode
  *
  * When set to 1, pressing the blue user button (PC13) applies a fixed
  * torque for a set duration, overriding any SPI torque commands.
  * Useful for testing motor movement without requiring RPi control.
  *
  * Set to 0 for normal SPI-controlled operation.
  */
#define TEST_MODE_TORQUE_BUTTON     1

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
