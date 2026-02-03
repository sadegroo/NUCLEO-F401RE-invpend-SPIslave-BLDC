/**
  ******************************************************************************
  * @file    chrono.h
  * @brief   Cycle counter based timing utilities using DWT
  ******************************************************************************
  */

#ifndef __CHRONO_H
#define __CHRONO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* System clock frequency - must match actual configuration */
#define RCC_SYS_CLOCK_FREQ 84000000

/* Chrono structure for timing measurements */
typedef struct {
    uint32_t start_ticks;
    uint32_t end_ticks;
    float t_diff_s;
} Chrono_TypeDef;

/**
  * @brief  Initialize the DWT cycle counter for timing measurements
  */
void Chrono_Init(void);

/**
  * @brief  Mark the current time and calculate difference from previous mark
  * @param  chrono: pointer to Chrono structure
  */
void Chrono_Mark(Chrono_TypeDef *chrono);

/**
  * @brief  Get time difference from last mark without updating the mark
  * @param  chrono: pointer to Chrono structure
  * @retval Time difference in seconds
  */
float Chrono_GetDiffNoMark(Chrono_TypeDef *chrono);

/**
  * @brief  Get time difference and update the mark
  * @param  chrono: pointer to Chrono structure
  * @retval Time difference in seconds
  */
float Chrono_GetDiffMark(Chrono_TypeDef *chrono);

#ifdef __cplusplus
}
#endif

#endif /* __CHRONO_H */
