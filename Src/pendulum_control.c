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
#include "mc_interface.h"
#include "encoder_speed_pos_fdbk.h"
#include "app_config.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * External Handles (defined in main.c by CubeMX)
 ******************************************************************************/
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern ENCODER_Handle_t ENCODER_M1;

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

#if TEST_MODE_TORQUE_BUTTON
/** Torque test mode variables */
static volatile uint8_t torque_test_requested = 0;  /* Set by button ISR */
static volatile uint8_t torque_test_active = 0;
static uint32_t torque_test_start_tick = 0;
static uint8_t motor_start_retries = 0;
#define TORQUE_TEST_DURATION_MS  5000
#define TORQUE_TEST_VALUE_MNM    20
#endif

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

/** UART transmit buffer for debug messages */
static char uart_tx_buf[80];

#if DEBUG_PENDULUM_ENCODER
/** Debug print timer */
static Chrono_TypeDef debug_timer = {0, 0, 0.0f};

/** Debug print interval in seconds */
#define DEBUG_PRINT_INTERVAL_S  (DEBUG_PRINT_INTERVAL_MS / 1000.0f)
#endif

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

/**
  * @brief  Quick UART print helper with short timeout
  * @param  msg: Null-terminated string to transmit
  * @note   Uses 1ms timeout to minimize blocking
  */
static void UART_Print_Quick(const char *msg)
{
    if (msg) {
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 1);
    }
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

#if DEBUG_PENDULUM_ENCODER
    /* Initialize debug timer */
    Chrono_Mark(&debug_timer);
#endif
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
#if SKIP_SPI_WAIT
        /* Skip waiting for SPI and go directly to reading encoders */
        g_state = STATE_READ;
        Chrono_Mark(&cycle_timer);
#if DEBUG_PENDULUM_ENCODER
        Chrono_Mark(&debug_timer);
#endif
#else
        /* Wait for first SPI transaction to establish communication */
        if (spi_txrx_flag)
        {
            spi_txrx_flag = 0;
            g_state = STATE_READ;
            Chrono_Mark(&cycle_timer);
#if DEBUG_PENDULUM_ENCODER
            Chrono_Mark(&debug_timer);
#endif
        }
#endif
        break;

    /*------------------------------------------------------------------*/
    case STATE_READ:
    {
        /* Read pendulum encoder */
        Pendulum_Encoder_Read(&pendulum_enc, &htim3);

        /* Get motor mechanical position from encoder
         * SPD_GetMecAngle returns int32 in s16 format (65536 counts/turn)
         * Scale to 8192 counts/turn by dividing by 8 (shift right 3) */
        int32_t mec_angle_raw = SPD_GetMecAngle(&ENCODER_M1._Super);
        int16_t motor_pos = (int16_t)(mec_angle_raw >> 3);

        /* Get motor velocity from MC SDK (in SPEED_UNIT) */
        int16_t motor_vel = MC_GetMecSpeedAverageMotor1();

        /* Get measured motor torque */
        int16_t measured_torque = GetMeasuredTorque();

        /* Pack TX buffer with big-endian data */
        int16_t pend_be = swap16((int16_t)pendulum_enc.position_steps);
        int16_t mpos_be = swap16(motor_pos);
        int16_t mvel_be = swap16(motor_vel);
        int16_t mtorq_be = swap16(measured_torque);

        /* Disable interrupts briefly to ensure atomic buffer update */
        __disable_irq();
        memcpy(&spi_tx_buf[0], &pend_be, 2);
        memcpy(&spi_tx_buf[2], &mpos_be, 2);
        memcpy(&spi_tx_buf[4], &mvel_be, 2);
        memcpy(&spi_tx_buf[6], &mtorq_be, 2);
        __enable_irq();

#if DEBUG_PENDULUM_ENCODER
        /* Periodically print debug info to UART (short timeout) */
        if (Chrono_GetDiffNoMark(&debug_timer) >= DEBUG_PRINT_INTERVAL_S)
        {
            Chrono_Mark(&debug_timer);
            int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                               "Pend:%ld torSP:%d torCV:%d\r\n",
                               (long)pendulum_enc.position_steps,
                               torque_cmd_mNm,
                               measured_torque);
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buf, (uint16_t)len, 1);
        }
#endif

#if SKIP_SPI_WAIT
        /* Without SPI, wait for sample interval then loop back to READ */
        if (Chrono_GetDiffNoMark(&cycle_timer) >= T_SAMPLE)
        {
            Chrono_Mark(&cycle_timer);
            g_state = STATE_READ;
        }
        break;
    }
#else
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
#endif

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
#if TEST_MODE_TORQUE_BUTTON
    {
        MCI_State_t motor_state = MC_GetSTMStateMotor1();

        /* Handle button press request - start motor */
        if (torque_test_requested && !torque_test_active) {
            if (motor_state == RUN) {
                /* Motor already running - start test */
                torque_test_requested = 0;
                torque_test_active = 1;
                torque_test_start_tick = HAL_GetTick();
                UART_Print_Quick(">>> Motor running - torque test started!\r\n");
            } else if (motor_state == FAULT_NOW || motor_state == FAULT_OVER) {
                /* Clear fault and retry */
                MC_AcknowledgeFaultMotor1();
                motor_start_retries++;
                if (motor_start_retries >= 3) {
                    torque_test_requested = 0;
                    UART_Print_Quick(">>> Failed after 3 retries\r\n");
                }
            } else if (motor_state == IDLE) {
                /* Start motor */
                MC_ProgramTorqueRampMotor1_F(0.0f, 0);
                MC_StartMotor1();
                UART_Print_Quick(">>> Starting motor...\r\n");
            }
            /* else: motor is starting up, wait */
        }

        /* Check if torque test is active and handle timing */
        if (torque_test_active) {
            if ((HAL_GetTick() - torque_test_start_tick) < TORQUE_TEST_DURATION_MS) {
                torque_cmd_mNm = TORQUE_TEST_VALUE_MNM;  /* Override SPI command */
            } else {
                torque_cmd_mNm = 0;
                torque_test_active = 0;  /* Test complete */
                UART_Print_Quick(">>> Torque test complete!\r\n");
            }
        }
    }
#endif
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

/**
  * @brief  Override the MC SDK Start/Stop button callback
  *
  * This overrides the __weak function in mc_tasks.c to disable
  * the user button motor control. In our application, motor control
  * is handled via SPI commands from the Raspberry Pi only.
  */
void UI_HandleStartStopButton_cb(void)
{
#if TEST_MODE_TORQUE_BUTTON
    /* Just set flag - actual motor startup happens in main loop */
    if (!torque_test_active && !torque_test_requested) {
        torque_test_requested = 1;
        motor_start_retries = 0;
    }
#else
    /* Do nothing - motor control is via SPI only */
#endif
}

