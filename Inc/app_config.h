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
#define TEST_MODE_NO_MOTOR          1

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

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
