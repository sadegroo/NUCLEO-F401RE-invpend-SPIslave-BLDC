/**
  ******************************************************************************
  * @file    torque_control.h
  * @brief   Torque command interface for BLDC motor control
  *
  * Converts torque commands in milli-Nm to motor current (Iq) and applies
  * them via the STM32 Motor Control SDK.
  ******************************************************************************
  */

#ifndef TORQUE_CONTROL_H
#define TORQUE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * Motor Torque Constant (Kt) Calculation:
  *
  * From pmsm_motor_parameters.h:
  *   MOTOR_VOLTAGE_CONSTANT = 3.0 V_rms_ph-ph/kRPM
  *   POLE_PAIR_NUM = 8
  *
  * Ke = MOTOR_VOLTAGE_CONSTANT / (1000 * 2*PI/60) * sqrt(2)/sqrt(3)
  *    = 3.0 / 104.72 * 0.8165
  *    = 0.0234 V_peak_ph/(rad/s)
  *
  * For surface-mount PMSM: Kt ≈ Ke
  * Kt = 0.0234 Nm/A
  *
  * NOTE: This value may need calibration based on actual motor measurements.
  */
#define MOTOR_KT_NM_PER_A   0.0234f

/** Maximum allowed current (from NOMINAL_CURRENT_A in pmsm_motor_parameters.h) */
#define MAX_CURRENT_A       5.0f

/**
  * @brief  Convert torque command in milli-Nm to Iq current in Amperes
  * @param  torque_mNm: Torque command in milli-Newton-meters
  * @retval Iq current in Amperes
  *
  * Formula: Iq = (torque_mNm / 1000) / Kt
  */
static inline float TorqueToIq(int16_t torque_mNm)
{
    return ((float)torque_mNm / 1000.0f) / MOTOR_KT_NM_PER_A;
}

/**
  * @brief  Convert Iq current in Amperes to torque in milli-Nm
  * @param  iq_amps: Iq current in Amperes
  * @retval Torque in milli-Newton-meters
  */
static inline int16_t IqToTorque(float iq_amps)
{
    return (int16_t)(iq_amps * MOTOR_KT_NM_PER_A * 1000.0f);
}

/**
  * @brief  Apply torque command to motor via MC SDK
  * @param  torque_mNm: Torque command in milli-Newton-meters
  *         Positive = forward direction
  *         Negative = reverse direction
  *
  * This function:
  * 1. Converts milli-Nm to Iq current
  * 2. Clamps current to safe limits (±MAX_CURRENT_A)
  * 3. Applies command via MC_ProgramTorqueRampMotor1_F()
  */
void ApplyTorqueCommand(int16_t torque_mNm);

#ifdef __cplusplus
}
#endif

#endif /* TORQUE_CONTROL_H */
