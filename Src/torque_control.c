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
}
