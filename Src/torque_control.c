/**
  ******************************************************************************
  * @file    torque_control.c
  * @brief   Torque command interface implementation
  *
  * Converts torque commands in milli-Nm to motor current and applies them
  * via the STM32 Motor Control SDK.
  ******************************************************************************
  */

#include "torque_control.h"
#include "mc_api.h"
#include "app_config.h"

/**
  * @brief  Apply torque command to motor via MC SDK
  * @param  torque_mNm: Torque command in milli-Newton-meters
  *         Positive = forward direction
  *         Negative = reverse direction
  *
  * Converts torque to Iq current, clamps to safe limits, and applies
  * the command using MC_ProgramTorqueRampMotor1_F with 0ms ramp time
  * for immediate effect.
  */
void ApplyTorqueCommand(int16_t torque_mNm)
{
#if TEST_MODE_NO_MOTOR
    /* Test mode: ignore all torque commands for safety */
    (void)torque_mNm;
    return;
#else
    /* Convert milli-Nm to Iq current (Amperes) */
    float iq_amps = TorqueToIq(torque_mNm);

    /* Clamp to safe current limits */
    if (iq_amps > MAX_CURRENT_A)
    {
        iq_amps = MAX_CURRENT_A;
    }
    else if (iq_amps < -MAX_CURRENT_A)
    {
        iq_amps = -MAX_CURRENT_A;
    }

    /* Apply torque command via MC SDK
     * Duration = 0 means immediate effect (no ramp)
     * MC_ProgramTorqueRampMotor1_F expects current in Amperes */
    MC_ProgramTorqueRampMotor1_F(iq_amps, 0);
#endif
}

/**
  * @brief  Get the measured motor torque from FOC current feedback
  * @retval Measured torque in milli-Newton-meters
  */
int16_t GetMeasuredTorque(void)
{
    /* Get measured Iq current from Motor Control SDK */
    qd_f_t iqd = MC_GetIqdMotor1_F();

    /* Convert Iq current to torque in milli-Nm */
    return IqToTorque(iqd.q);
}
