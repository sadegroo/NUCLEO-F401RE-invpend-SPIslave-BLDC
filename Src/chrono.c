/**
  ******************************************************************************
  * @file    chrono.c
  * @brief   Cycle counter based timing utilities using DWT
  *
  * Uses the ARM Cortex-M4 Data Watchpoint and Trace (DWT) unit's cycle
  * counter for high-resolution timing measurements.
  ******************************************************************************
  */

#include "chrono.h"
#include "stm32f4xx_hal.h"

/**
  * @brief  Initialize the DWT cycle counter
  *
  * Enables the DWT cycle counter (CYCCNT) for timing measurements.
  * The counter runs at the core clock frequency (84 MHz).
  */
void Chrono_Init(void)
{
    /* Enable trace and debug blocks */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  Mark the current time and calculate difference from previous mark
  * @param  chrono: pointer to Chrono structure
  *
  * Reads the current cycle count, calculates the time elapsed since the
  * previous mark, and updates the start_ticks for the next measurement.
  */
void Chrono_Mark(Chrono_TypeDef *chrono)
{
    /* Read current cycle count */
    chrono->end_ticks = DWT->CYCCNT;

    /* Calculate difference (handles overflow automatically due to unsigned arithmetic) */
    uint32_t difference = chrono->end_ticks - chrono->start_ticks;

    /* Update start for next measurement */
    chrono->start_ticks = chrono->end_ticks;

    /* Convert to seconds */
    chrono->t_diff_s = (float)difference / (float)RCC_SYS_CLOCK_FREQ;
}

/**
  * @brief  Get time difference from last mark without updating the mark
  * @param  chrono: pointer to Chrono structure
  * @retval Time difference in seconds
  *
  * Useful for checking elapsed time without resetting the reference point.
  */
float Chrono_GetDiffNoMark(Chrono_TypeDef *chrono)
{
    /* Read current cycle count */
    uint32_t current_ticks = DWT->CYCCNT;

    /* Calculate difference */
    uint32_t difference = current_ticks - chrono->start_ticks;

    /* Convert to seconds */
    return (float)difference / (float)RCC_SYS_CLOCK_FREQ;
}

/**
  * @brief  Get time difference and update the mark
  * @param  chrono: pointer to Chrono structure
  * @retval Time difference in seconds
  *
  * Convenience function that combines GetDiffNoMark and Mark operations.
  */
float Chrono_GetDiffMark(Chrono_TypeDef *chrono)
{
    Chrono_Mark(chrono);
    return chrono->t_diff_s;
}
