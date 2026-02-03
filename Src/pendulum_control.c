/**
  ******************************************************************************
  * @file    pendulum_control.c
  * @brief   Inverted pendulum control state machine implementation
  *
  * Implements the 1 kHz control loop state machine that:
  * - Reads pendulum encoder position from TIM3
  * - Reads motor position/velocity from MC SDK
  * - Exchanges data with Raspberry Pi via SPI3 slave
  * - Applies torque commands to BLDC motor
  ******************************************************************************
  */

#include "pendulum_control.h"
#include "torque_control.h"
#include "chrono.h"
#include "mc_api.h"
#include <string.h>

/*******************************************************************************
 * External Handles (defined in main.c by CubeMX)
 ******************************************************************************/
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim3;

/*******************************************************************************
 * Module Variables
 ******************************************************************************/

/** Current state machine state */
volatile uint8_t g_state = STATE_START;

/** SPI transaction complete flag (set in DMA callback) */
volatile uint8_t spi_txrx_flag = 0;

/** SPI error flag */
volatile uint8_t spi_err_flag = 0;

/** SPI receive buffer */
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];

/** SPI transmit buffer */
static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];

/** Current torque command in milli-Nm */
static int16_t torque_cmd_mNm = 0;

/** Pendulum encoder instance */
static Pendulum_Encoder_TypeDef pendulum_enc = {
    .cnt = 0,
    .previous_cnt = 0,
    .position_steps = 0,
    .position_init = 0,
    .counts_per_turn = PENDULUM_COUNTS_PER_REV
};

/** Cycle timer for timing measurements */
static Chrono_TypeDef cycle_timer = {0, 0, 0.0f};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
  * @brief  Swap endianness of 16-bit value (for big-endian SPI protocol)
  * @param  v: Value to swap
  * @retval Byte-swapped value
  */
static inline int16_t swap16(int16_t v)
{
    return (int16_t)(((uint16_t)v >> 8) | ((uint16_t)v << 8));
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
  * @brief  Initialize pendulum control subsystem
  */
void Pendulum_Init(void)
{
    /* Initialize cycle counter */
    Chrono_Init();

    /* Initialize pendulum encoder */
    pendulum_enc.cnt = 0;
    pendulum_enc.previous_cnt = 0;
    pendulum_enc.position_steps = 0;
    pendulum_enc.position_init = 0;
    pendulum_enc.counts_per_turn = PENDULUM_COUNTS_PER_REV;

    /* Clear SPI buffers */
    memset(spi_rx_buf, 0, SPI_BUFFER_SIZE);
    memset(spi_tx_buf, 0, SPI_BUFFER_SIZE);

    /* Reset state machine */
    g_state = STATE_START;
    spi_txrx_flag = 0;
    spi_err_flag = 0;
}

/**
  * @brief  Read pendulum encoder position
  * @param  enc: Pointer to pendulum encoder structure
  * @param  htim: Pointer to TIM handle (TIM3)
  */
void Pendulum_Encoder_Read(Pendulum_Encoder_TypeDef *enc, TIM_HandleTypeDef *htim)
{
    /* Save previous count */
    enc->previous_cnt = enc->cnt;

    /* Read current counter value (TIM3 is 16-bit) */
    enc->cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

    /* Calculate delta with 16-bit overflow handling
     * Signed subtraction of uint16_t values handles wrap-around correctly:
     * e.g., prev=65530, cnt=5 -> (int16_t)(5-65530) = 11 (correct forward delta) */
    int16_t delta = (int16_t)(enc->cnt - enc->previous_cnt);

    /* Accumulate position (int32_t for extended range) */
    enc->position_steps += delta;
}

/**
  * @brief  Start SPI DMA circular communication
  */
void SPI_StartCommunication(void)
{
    /* Start DMA-based SPI transfer in circular mode
     * This will continuously exchange data with RPi master */
    HAL_SPI_TransmitReceive_DMA(&hspi3, spi_tx_buf, spi_rx_buf, SPI_BUFFER_SIZE);
}

/**
  * @brief  Run state machine iteration
  */
void StateMachine_Run(void)
{
    switch (g_state)
    {
    /*------------------------------------------------------------------*/
    case STATE_START:
        /* Wait for first SPI transaction to establish communication */
        if (spi_txrx_flag)
        {
            spi_txrx_flag = 0;
            g_state = STATE_READ;
            Chrono_Mark(&cycle_timer);
        }
        break;

    /*------------------------------------------------------------------*/
    case STATE_READ:
    {
        /* Read pendulum encoder */
        Pendulum_Encoder_Read(&pendulum_enc, &htim3);

        /* Get motor position from MC SDK
         * MC_GetElAngledppMotor1() returns electrical angle in s16 format
         * For mechanical position, divide by pole pairs or use encoder directly */
        int16_t motor_pos = MC_GetElAngledppMotor1();

        /* Get motor velocity from MC SDK (in SPEED_UNIT) */
        int16_t motor_vel = MC_GetMecSpeedAverageMotor1();

        /* Pack TX buffer with big-endian data */
        int16_t pend_be = swap16((int16_t)pendulum_enc.position_steps);
        int16_t mpos_be = swap16(motor_pos);
        int16_t mvel_be = swap16(motor_vel);

        /* Disable interrupts briefly to ensure atomic buffer update */
        __disable_irq();
        memcpy(&spi_tx_buf[0], &pend_be, 2);
        memcpy(&spi_tx_buf[2], &mpos_be, 2);
        memcpy(&spi_tx_buf[4], &mvel_be, 2);
        __enable_irq();

        g_state = STATE_WAIT_SPI;
        break;
    }

    /*------------------------------------------------------------------*/
    case STATE_WAIT_SPI:
        if (spi_txrx_flag)
        {
            spi_txrx_flag = 0;

            /* Parse received torque command (big-endian) */
            int16_t cmd_be;
            memcpy(&cmd_be, &spi_rx_buf[0], 2);
            torque_cmd_mNm = swap16(cmd_be);

            g_state = STATE_CONTROL;
        }
        else if (Chrono_GetDiffNoMark(&cycle_timer) > SPI_TIMEOUT_MULT * T_SAMPLE)
        {
            /* SPI timeout - go to overtime state */
            g_state = STATE_OVERTIME;
        }
        break;

    /*------------------------------------------------------------------*/
    case STATE_OVERTIME:
        /* Keep checking for SPI completion */
        if (spi_txrx_flag)
        {
            spi_txrx_flag = 0;

            /* Parse received torque command */
            int16_t cmd_be;
            memcpy(&cmd_be, &spi_rx_buf[0], 2);
            torque_cmd_mNm = swap16(cmd_be);

            /* Reset timing */
            Chrono_Mark(&cycle_timer);
            g_state = STATE_CONTROL;
        }
        break;

    /*------------------------------------------------------------------*/
    case STATE_CONTROL:
        /* Apply torque command to motor */
        ApplyTorqueCommand(torque_cmd_mNm);

        /* Restart cycle */
        Chrono_Mark(&cycle_timer);
        g_state = STATE_READ;
        break;

    /*------------------------------------------------------------------*/
    case STATE_ERROR:
        /* Error state - stop motor and wait */
        ApplyTorqueCommand(0);
        /* Could add recovery logic here */
        break;

    /*------------------------------------------------------------------*/
    case STATE_HALT:
        /* Halted - do nothing */
        break;

    /*------------------------------------------------------------------*/
    default:
        /* Unknown state - reset to start */
        g_state = STATE_START;
        break;
    }
}

/**
  * @brief  Get current torque command value
  * @retval Torque command in milli-Nm
  */
int16_t GetTorqueCommand(void)
{
    return torque_cmd_mNm;
}

/**
  * @brief  Get current pendulum position
  * @retval Pendulum position in encoder counts
  */
int32_t GetPendulumPosition(void)
{
    return pendulum_enc.position_steps;
}

/*******************************************************************************
 * HAL Callbacks
 ******************************************************************************/

/**
  * @brief  SPI transmit/receive complete callback
  * @param  hspi: SPI handle
  *
  * Called when DMA transfer completes. Sets flag to notify state machine.
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        spi_txrx_flag = 1;
    }
}

/**
  * @brief  SPI error callback
  * @param  hspi: SPI handle
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        spi_err_flag = 1;
        /* Could restart SPI here if needed */
    }
}
