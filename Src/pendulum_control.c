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
extern TIM_HandleTypeDef htim2;
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

/** Motor start control (button-triggered, always available) */
static volatile uint8_t motor_start_requested = 0;
static uint8_t motor_start_retries = 0;

/** Motor initialization status (1 = motor has reached RUN state after button press) */
static uint8_t motor_initialized = 0;

/** Special value to indicate motor not initialized (999) */
#define TORQUE_NOT_INITIALIZED  (999)

#if TEST_MODE_TORQUE_BUTTON
/** Torque test mode variables (only when enabled) */
static volatile uint8_t torque_test_active = 0;
static uint32_t torque_test_start_tick = 0;
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

/** Encoder counts per revolution (4x quadrature) */
#define MOTOR_ENCODER_CPR  8192

/** Velocity calculators (with separate filter alphas) */
static VelocityCalc_t motor_vel_calc = {
    .enc_prev = 0,
    .time_prev_cycles = 0,
    .vel_filtered = 0.0f,
    .filter_alpha = MOTOR_VEL_FILTER_ALPHA,
    .cpr = MOTOR_ENCODER_CPR,
    .wrap_at_cpr = 1
};

static VelocityCalc_t pend_vel_calc = {
    .enc_prev = 0,
    .time_prev_cycles = 0,
    .vel_filtered = 0.0f,
    .filter_alpha = PEND_VEL_FILTER_ALPHA,
    .cpr = PENDULUM_COUNTS_PER_REV,
    .wrap_at_cpr = 0
};

#if DEBUG_PENDULUM_ENCODER
/** Last raw delta for debugging velocity spikes */
static int32_t motor_vel_last_delta = 0;
#endif

/** Overspeed detection - dual window */
static int32_t overspeed1_accumulator = 0;  /* Short window (0.25 rev, 400 RPM) */
static int32_t overspeed2_accumulator = 0;  /* Long window (0.5 rev, 200 RPM) */
static uint8_t overspeed_fault_active = 0;

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

/**
  * @brief  Calculate velocity with EMA filtering (generic for any encoder)
  *
  * Uses floating-point math and exponential moving average for smooth output.
  * Handles both CPR-wrap (motor) and 16-bit wrap (pendulum) encoder types.
  *
  * @param  vc: Pointer to velocity calculator state
  * @param  cnt: Current encoder count
  * @retval Velocity in counts/second (int32_t for full precision)
  */
static int32_t CalcVelocity(VelocityCalc_t *vc, uint32_t cnt)
{
    /* Measure actual time since last call using DWT cycle counter
     * SystemCoreClock = 84 MHz, so cycles_to_sec = cycles / 84000000 */
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t dt_cycles = now_cycles - vc->time_prev_cycles;
    vc->time_prev_cycles = now_cycles;

    /* Convert to seconds (float for precision) */
    float dt_sec = (float)dt_cycles / (float)SystemCoreClock;

    /* Sanity check: if dt is too small or too large, use nominal 1ms */
    if (dt_sec < 0.0001f || dt_sec > 0.1f) {
        dt_sec = 0.001f;
    }

    /* Calculate encoder delta */
    int32_t delta = (int32_t)cnt - (int32_t)vc->enc_prev;

    /* Handle wraparound based on encoder type */
    if (vc->wrap_at_cpr) {
        /* Motor encoder: wraps at CPR (e.g., 8192)
         * If delta > cpr/2, counter wrapped backward
         * If delta < -cpr/2, counter wrapped forward */
        if (delta > (int32_t)(vc->cpr / 2)) {
            delta -= vc->cpr;
        } else if (delta < -(int32_t)(vc->cpr / 2)) {
            delta += vc->cpr;
        }
    } else {
        /* Pendulum encoder: wraps at 16-bit boundary
         * Signed 16-bit cast handles wrap naturally */
        delta = (int16_t)delta;
    }

    vc->enc_prev = cnt;

    /* Convert to counts/second using measured time delta */
    float vel_counts_per_sec = (float)delta / dt_sec;

    /* Apply exponential moving average filter (instance-specific alpha) */
    vc->vel_filtered = vc->filter_alpha * vel_counts_per_sec
                     + (1.0f - vc->filter_alpha) * vc->vel_filtered;

    /* Return in counts/second (int32 for full precision, scaled later for SPI) */
    return (int32_t)vc->vel_filtered;
}

/**
  * @brief  Check for overspeed condition using dual windows
  *
  * Two independent checks:
  * - Window 1: Short burst (0.25 rev at 400+ RPM)
  * - Window 2: Sustained (0.5 rev at 200+ RPM)
  *
  * @note   Uses motor_vel_calc.vel_filtered for smooth overspeed detection
  * @retval 1 if overspeed fault triggered, 0 otherwise
  */
static uint8_t CheckOverspeed(void)
{
    int32_t vel_cps = (int32_t)motor_vel_calc.vel_filtered;
    int32_t abs_vel = (vel_cps < 0) ? -vel_cps : vel_cps;
    int32_t counts_per_sample = abs_vel / 1000;  /* 1kHz sample rate */

    /* Window 1: Short burst limit (0.25 rev, 400 RPM) */
    if (abs_vel > OVERSPEED1_THRESHOLD_CPS) {
        overspeed1_accumulator += counts_per_sample;
        int32_t threshold1 = (int32_t)(OVERSPEED1_WINDOW_REVS * MOTOR_ENCODER_CPR);
        if (overspeed1_accumulator >= threshold1) {
            overspeed_fault_active = 1;
            return 1;
        }
    } else {
        overspeed1_accumulator = 0;
    }

    /* Window 2: Sustained speed limit (0.5 rev, 200 RPM) */
    if (abs_vel > OVERSPEED2_THRESHOLD_CPS) {
        overspeed2_accumulator += counts_per_sample;
        int32_t threshold2 = (int32_t)(OVERSPEED2_WINDOW_REVS * MOTOR_ENCODER_CPR);
        if (overspeed2_accumulator >= threshold2) {
            overspeed_fault_active = 1;
            return 1;
        }
    } else {
        overspeed2_accumulator = 0;
    }

    return 0;
}

/**
  * @brief  Process motor startup sequence
  *
  * Handles motor state transitions during startup. Can be called from
  * STATE_START (before SPI) or STATE_CONTROL (after SPI sync).
  *
  * @retval 1 if motor reached RUN state, 0 otherwise
  */
static uint8_t ProcessMotorStartup(void)
{
    MCI_State_t motor_state = MC_GetSTMStateMotor1();

    if (motor_state == RUN) {
        /* Motor running - clear request and mark as initialized */
        motor_start_requested = 0;
        motor_start_retries = 0;
        motor_initialized = 1;
        return 1;
    } else if (motor_state == FAULT_NOW || motor_state == FAULT_OVER) {
        /* Clear fault and retry */
        MC_AcknowledgeFaultMotor1();
        motor_start_retries++;
        if (motor_start_retries >= 3) {
            motor_start_requested = 0;
        }
    } else if (motor_state == IDLE) {
        /* Start motor */
        MC_ProgramTorqueRampMotor1_F(0.0f, 0);
        MC_StartMotor1();
    }
    /* else: motor is starting up (ALIGNMENT, etc.), wait */
    return 0;
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

    /* Initialize velocity calculators */
    motor_vel_calc.enc_prev = __HAL_TIM_GET_COUNTER(&htim2);
    motor_vel_calc.time_prev_cycles = DWT->CYCCNT;
    motor_vel_calc.vel_filtered = 0.0f;

    pend_vel_calc.enc_prev = __HAL_TIM_GET_COUNTER(&htim3);
    pend_vel_calc.time_prev_cycles = DWT->CYCCNT;
    pend_vel_calc.vel_filtered = 0.0f;

#if DEBUG_PENDULUM_ENCODER
    /* Initialize debug timer */
    Chrono_Mark(&debug_timer);
#endif

#if AUTO_MOTOR_INIT
    /* Request automatic motor start */
    motor_start_requested = 1;
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
        /* Process motor startup even before SPI sync (button or auto-init) */
        if (motor_start_requested) {
            ProcessMotorStartup();
        }

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

        /* Calculate pendulum velocity using generic velocity calculator */
        int32_t pend_vel_raw = CalcVelocity(&pend_vel_calc, pendulum_enc.cnt);
        int16_t pend_vel = (int16_t)(pend_vel_raw / PEND_VEL_RESOLUTION_DIV);

        /* Get motor mechanical position from encoder
         * SPD_GetMecAngle returns int32 in s16 format (65536 counts/turn)
         * Scale to 8192 counts/turn by dividing by 8 (shift right 3) */
        int32_t mec_angle_raw = SPD_GetMecAngle(&ENCODER_M1._Super);
        int16_t motor_pos = (int16_t)(mec_angle_raw >> 3);

        /* Calculate motor velocity using generic velocity calculator */
        int32_t motor_vel_raw = CalcVelocity(&motor_vel_calc, __HAL_TIM_GET_COUNTER(&htim2));

        /* Check overspeed - sets flag but doesn't interrupt state machine */
        CheckOverspeed();

        /* Scale down and convert to int16 for SPI */
        int16_t motor_vel = (int16_t)(motor_vel_raw / MOTOR_VEL_RESOLUTION_DIV);

        /* Get measured motor torque - use special value if motor not initialized */
        int16_t measured_torque = motor_initialized ? GetMeasuredTorque() : TORQUE_NOT_INITIALIZED;

        /* Pack TX buffer with big-endian data (10 bytes) */
        int16_t ppos_be = swap16((int16_t)pendulum_enc.position_steps);
        int16_t pvel_be = swap16(pend_vel);
        int16_t mpos_be = swap16(motor_pos);
        int16_t mvel_be = swap16(motor_vel);
        int16_t mtorq_be = swap16(measured_torque);

        /* Disable interrupts briefly to ensure atomic buffer update */
        __disable_irq();
        memcpy(&spi_tx_buf[0], &ppos_be, 2);
        memcpy(&spi_tx_buf[2], &pvel_be, 2);
        memcpy(&spi_tx_buf[4], &mpos_be, 2);
        memcpy(&spi_tx_buf[6], &mvel_be, 2);
        memcpy(&spi_tx_buf[8], &mtorq_be, 2);
        __enable_irq();

#if DEBUG_PENDULUM_ENCODER
        /* Periodically print debug info to UART (short timeout) */
        if (Chrono_GetDiffNoMark(&debug_timer) >= DEBUG_PRINT_INTERVAL_S)
        {
            Chrono_Mark(&debug_timer);
            int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                               "P:%ld d:%ld v:%d tS:%d tC:%d\r\n",
                               (long)pendulum_enc.position_steps,
                               (long)motor_vel_last_delta,
                               motor_vel,
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
    {
#if DEBUG_MOTOR
        /* Debug: Monitor motor state and faults */
        static uint32_t last_debug_tick = 0;
        static uint8_t last_motor_init = 0;
        static MCI_State_t last_mc_state = IDLE;
        uint32_t now_tick = HAL_GetTick();
        MCI_State_t mc_state = MC_GetSTMStateMotor1();
        uint16_t faults = MC_GetOccurredFaultsMotor1();

        /* Print on state change or every 500ms */
        if (motor_initialized != last_motor_init ||
            mc_state != last_mc_state ||
            (now_tick - last_debug_tick) >= 500) {

            snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                     "MC:%d init:%d flt:0x%04X req:%d\r\n",
                     (int)mc_state, motor_initialized, faults, motor_start_requested);
            UART_Print_Quick(uart_tx_buf);

            last_debug_tick = now_tick;
            last_motor_init = motor_initialized;
            last_mc_state = mc_state;
        }
#endif

        /* Handle motor startup (button or auto-init) */
        if (motor_start_requested) {
            if (ProcessMotorStartup()) {
#if TEST_MODE_TORQUE_BUTTON
                /* Motor just started - begin torque test */
                torque_test_active = 1;
                torque_test_start_tick = HAL_GetTick();
#endif
            }
        }

        /* Handle overspeed fault - force zero torque, clear when Pi sends zero */
        if (overspeed_fault_active) {
            if (torque_cmd_mNm == 0) {
                /* Pi acknowledged fault by sending zero - clear fault */
                overspeed_fault_active = 0;
                overspeed1_accumulator = 0;
                overspeed2_accumulator = 0;
            } else {
                /* Force zero torque while fault is active */
                torque_cmd_mNm = 0;
            }
        }

#if TEST_MODE_TORQUE_BUTTON
        /* Torque test override (only when enabled) */
        if (torque_test_active) {
            if ((HAL_GetTick() - torque_test_start_tick) < TORQUE_TEST_DURATION_MS) {
                torque_cmd_mNm = TORQUE_TEST_VALUE_MNM;
            } else {
                torque_cmd_mNm = 0;
                torque_test_active = 0;
            }
        }
#endif
    }
        /* Apply torque command to motor */
        ApplyTorqueCommand(torque_cmd_mNm);

        /* Restart cycle */
        Chrono_Mark(&cycle_timer);
        g_state = STATE_READ;
        break;

    /*------------------------------------------------------------------*/
    case STATE_ERROR:
        /* Reserved for future critical errors that require full stop */
        ApplyTorqueCommand(0);
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

/** Button debounce time in ms */
#define BUTTON_DEBOUNCE_MS  200

/**
  * @brief  Override the MC SDK Start/Stop button callback
  *
  * This overrides the __weak function in mc_tasks.c to disable
  * the user button motor control. In our application, motor control
  * is handled via SPI commands from the Raspberry Pi only.
  */
void UI_HandleStartStopButton_cb(void)
{
    static uint32_t last_press_tick = 0;
    uint32_t now = HAL_GetTick();

    /* Debounce: ignore presses within BUTTON_DEBOUNCE_MS of last press */
    if ((now - last_press_tick) < BUTTON_DEBOUNCE_MS) {
        return;
    }
    last_press_tick = now;

    if (motor_initialized) {
        /* Motor is running - stop it */
        MC_StopMotor1();
        motor_initialized = 0;
        motor_start_requested = 0;
#if TEST_MODE_TORQUE_BUTTON
        torque_test_active = 0;
#endif
    } else if (!motor_start_requested) {
        /* Motor not running - request start */
        motor_start_requested = 1;
        motor_start_retries = 0;
    }
}

