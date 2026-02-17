/**
  ******************************************************************************
  * @file    pendulum_control.h
  * @brief   Inverted pendulum control state machine and encoder interface
  *
  * Implements:
  * - SPI slave communication with Raspberry Pi (10-byte protocol)
  * - Pendulum encoder reading on TIM3
  * - 1 kHz control loop state machine
  ******************************************************************************
  */

#ifndef PENDULUM_CONTROL_H
#define PENDULUM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/*******************************************************************************
 * State Machine Definitions
 ******************************************************************************/

/** @defgroup StateMachine_States State Machine States
  * @{
  */
#define STATE_START     0   /**< Wait for first SPI transaction */
#define STATE_READ      1   /**< Read encoders and prepare TX data */
#define STATE_WAIT_SPI  2   /**< Wait for SPI transaction completion */
#define STATE_OVERTIME  3   /**< Handle SPI timeout */
#define STATE_CONTROL   4   /**< Apply torque command */
#define STATE_ERROR     50  /**< Error state (safety limit exceeded) */
#define STATE_HALT      99  /**< Halted state */
/** @} */

/*******************************************************************************
 * Timing Constants
 ******************************************************************************/

/** Control loop sample time in seconds (1 kHz) */
#define T_SAMPLE        0.001f

/** SPI timeout multiplier (overtime if > 1.5 * T_SAMPLE) */
#define SPI_TIMEOUT_MULT 1.5f

/*******************************************************************************
 * SPI Protocol Definitions
 ******************************************************************************/

/** SPI buffer size (5 x int16_t = 10 bytes) */
#define SPI_BUFFER_SIZE 10

/**
  * SPI Protocol (10 bytes, big-endian):
  *
  * RX from RPi (Master):
  *   [0-1] int16_t torque_cmd     - Torque command in milli-Nm
  *   [2-9] reserved               - Reserved for future use
  *
  * TX to RPi (Master):
  *   [0-1] int16_t pendulum_pos   - Pendulum encoder position (counts)
  *   [2-3] int16_t pendulum_vel   - Pendulum velocity (counts/sec / DIV)
  *   [4-5] int16_t motor_pos      - Motor encoder position (counts)
  *   [6-7] int16_t motor_vel      - Motor velocity (counts/sec / DIV)
  *   [8-9] int16_t measured_torque - Measured motor torque (milli-Nm)
  */

/*******************************************************************************
 * Pendulum Encoder Configuration
 ******************************************************************************/

/** Pendulum encoder counts per revolution */
#define PENDULUM_COUNTS_PER_REV 2400

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
  * @brief  Velocity calculator state structure
  * @note   Used for both motor and pendulum velocity calculation with EMA filter
  */
typedef struct {
    uint32_t enc_prev;          /**< Previous encoder count */
    uint32_t time_prev_cycles;  /**< Previous DWT cycle count */
    float vel_filtered;         /**< Filtered velocity (counts/sec) */
    float filter_alpha;         /**< EMA filter coefficient */
    uint16_t cpr;               /**< Counts per revolution (for wraparound) */
    uint8_t wrap_at_cpr;        /**< 1 if wraps at CPR (motor), 0 if 16-bit wrap (pendulum) */
} VelocityCalc_t;

/**
  * @brief  Pendulum encoder state structure
  * @note   Uses TIM3 which is a 16-bit timer (counter range 0-65535)
  */
typedef struct {
    uint16_t cnt;               /**< Current counter value (16-bit for TIM3) */
    uint16_t previous_cnt;      /**< Previous counter value (16-bit for TIM3) */
    int32_t position_steps;     /**< Accumulated position in encoder steps */
    int32_t position_init;      /**< Initial position offset */
    uint16_t counts_per_turn;   /**< Encoder counts per revolution (2400) */
} Pendulum_Encoder_TypeDef;

/*******************************************************************************
 * External Variables (defined in pendulum_control.c)
 ******************************************************************************/

/** Current state machine state */
extern volatile uint8_t g_state;

/** SPI transaction complete flag */
extern volatile uint8_t spi_txrx_flag;

/** SPI error flag */
extern volatile uint8_t spi_err_flag;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
  * @brief  Initialize pendulum control subsystem
  *         - Initializes chrono timer
  *         - Initializes pendulum encoder structure
  *         - Resets state machine to STATE_START
  */
void Pendulum_Init(void);

/**
  * @brief  Read pendulum encoder position
  * @param  enc: Pointer to pendulum encoder structure
  * @param  htim: Pointer to TIM handle (TIM3)
  */
void Pendulum_Encoder_Read(Pendulum_Encoder_TypeDef *enc, TIM_HandleTypeDef *htim);

/**
  * @brief  Start SPI DMA circular communication
  *         Call this once after initialization
  */
void SPI_StartCommunication(void);

/**
  * @brief  Run state machine iteration
  *         Call this continuously from main loop
  *         Handles: encoder reading, SPI data exchange, torque application
  */
void StateMachine_Run(void);

/**
  * @brief  Get current torque command value
  * @retval Torque command in milli-Nm
  */
int16_t GetTorqueCommand(void);

/**
  * @brief  Get current pendulum position
  * @retval Pendulum position in encoder counts
  */
int32_t GetPendulumPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* PENDULUM_CONTROL_H */
