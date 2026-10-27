/**
  ******************************************************************************
  * @file    rgbled.c
  * @brief   Implementation of the RGB LED part driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rgbled.h"

/* Private defines -----------------------------------------------------------*/
/* Number of samples at the tail of a fade transition buffer forced to hold the final target value.
   Holding the last few samples (instead of only the single final one) widens the time the end state
   -- dark when fading to 0% -- remains visible within each playback window. */
#define RGBLED_TRANSITION_TAIL_HOLD        3UL

/* Private functions prototype -----------------------------------------------*/
static rgbled_status_t rgbled_scale_percent_to_compare(const rgbled_tim_ctx_t *pchannel, uint16_t percent_value,
                                                       uint16_t *pcompare_value);
static rgbled_status_t rgbled_limit_color_values(rgbled_obj_t *pobj, const rgbled_color_t *psrc,
                                                 uint16_t *pred_compare, uint16_t *pgreen_compare,
                                                 uint16_t *pblue_compare);
static rgbled_status_t rgbled_apply_channel_output(rgbled_tim_ctx_t *pchannel, uint16_t pulse_value);
static rgbled_status_t rgbled_apply_color_values(rgbled_obj_t *pobj, uint16_t red_compare,
                                                 uint16_t green_compare, uint16_t blue_compare);
static rgbled_status_t rgbled_disable_outputs(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_unique_timers(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_unique_timers(rgbled_obj_t *pobj);
#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
static rgbled_status_t rgbled_stop_transition_timer(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_transition_timer(rgbled_obj_t *pobj, uint32_t period_ms);
static void rgbled_fill_channel_transition_pattern(uint16_t *pdst, uint16_t start_percent,
                                                   uint16_t end_percent, const rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_start_dma_pattern(rgbled_obj_t *pobj, uint32_t period_ms);
static rgbled_status_t rgbled_prepare_limited_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern);
static rgbled_status_t rgbled_stop_channel_dma_pattern(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_stop_dma_patterns(rgbled_obj_t *pobj);
#endif /* RGBLED_PATTERN_PLAYBACK */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Convert one user percentage into the nearest effective timer compare value.
  * @param pchannel pointer to the PWM channel context.
  * @param percent_value user brightness percentage.
  * @param pcompare_value pointer receiving the effective compare value.
  * @retval error status.
  */
static rgbled_status_t rgbled_scale_percent_to_compare(const rgbled_tim_ctx_t *pchannel, uint16_t percent_value,
                                                       uint16_t *pcompare_value)
{
  uint16_t clamped_percent;
  uint32_t limited_percent;
  uint32_t compare_value;

  /* Validate the channel context and output pointer before converting. */
  if ((pchannel == NULL) || (pcompare_value == NULL) || (pchannel->phtim == NULL)
      || (pchannel->autoreload == 0U))
  {
    return RGBLED_ERROR;
  }

  /* Clamp the requested percentage to the maximum allowed brightness. */
  clamped_percent = (percent_value > RGBLED_AUTORELOAD_PERCENTAGE) ? RGBLED_AUTORELOAD_PERCENTAGE : percent_value;

  /* Apply the per-channel maximum duty-cycle cap (in percent) so the requested brightness can never
     exceed the buck-converter current limit, rounding to nearest. Capping the requested percentage
     keeps every channel reaching zero together during a fade. */
  limited_percent = (((uint32_t)clamped_percent * pchannel->max_duty_cycle) + (RGBLED_MAX_DUTY / 2U))
                    / RGBLED_MAX_DUTY;

  /* Scale the clamped percentage into a timer compare value with rounding to nearest. */
  compare_value = ((pchannel->autoreload * limited_percent) + (RGBLED_AUTORELOAD_PERCENTAGE / 2U))
                  / RGBLED_AUTORELOAD_PERCENTAGE;

  /* Minimum-on floor: any nonzero brightness request maps to at least one timer tick, while 0%
    remains fully off. */
  if ((clamped_percent > 0U) && (compare_value == 0U))
  {
    compare_value = 1U;
  }

  *pcompare_value = (uint16_t)compare_value;
  return RGBLED_OK;
}

/**
  * @brief Apply the per-channel and global RGB safety limits and convert them to compare values.
  * @param pobj pointer to the RGB LED object.
  * @param psrc pointer to the requested RGB color.
  * @param pred_compare pointer receiving the limited red compare value.
  * @param pgreen_compare pointer receiving the limited green compare value.
  * @param pblue_compare pointer receiving the limited blue compare value.
  * @retval error status.
  */
static rgbled_status_t rgbled_limit_color_values(rgbled_obj_t *pobj, const rgbled_color_t *psrc,
                                                 uint16_t *pred_compare, uint16_t *pgreen_compare,
                                                 uint16_t *pblue_compare)
{
  uint16_t channel_value;

  /* Check pointers before applying the RGB safety rules. */
  if ((pobj == NULL) || (psrc == NULL) || (pred_compare == NULL)
      || (pgreen_compare == NULL) || (pblue_compare == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Convert each user percentage into the corresponding clamped compare value. */
  if (rgbled_scale_percent_to_compare(&pobj->red, psrc->red_color_value, &channel_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  *pred_compare = channel_value;

  if (rgbled_scale_percent_to_compare(&pobj->green, psrc->green_color_value, &channel_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  *pgreen_compare = channel_value;

  if (rgbled_scale_percent_to_compare(&pobj->blue, psrc->blue_color_value, &channel_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  *pblue_compare = channel_value;

  return RGBLED_OK;
}

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
/**
  * @brief Stop the DMA transfer on one RGB channel if it is active.
  * @param pchannel pointer to the channel context.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_channel_dma_pattern(rgbled_tim_ctx_t *pchannel)
{
  /* Validate the channel resources before touching the DMA path. */
  if ((pchannel == NULL) || (pchannel->phtim == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Stop the DMA stream only when this channel is currently active. */
  if (pchannel->dma_running == 1U)
  {
    if (HAL_OK != HAL_TIM_OC_StopChannel_DMA(pchannel->phtim, pchannel->pwm_channel))
    {
      return RGBLED_ERROR;
    }

    pchannel->dma_running = 0U;
  }

  return RGBLED_OK;
}

/**
  * @brief Stop all active DMA resources used by the RGB LED object.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_dma_patterns(rgbled_obj_t *pobj)
{
  /* Stop the three channel transfers before stopping the pacing timer. */
  if (rgbled_stop_channel_dma_pattern(&pobj->red) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_stop_channel_dma_pattern(&pobj->green) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_stop_channel_dma_pattern(&pobj->blue) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_stop_transition_timer(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Stop the timer used to generate DMA pacing events.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_transition_timer(rgbled_obj_t *pobj)
{
  /* Nothing to stop when the pattern timer was not configured by the board layer. */
  if (pobj->pattern_timer.phtim == NULL)
  {
    return RGBLED_OK;
  }

  if (pobj->pattern_timer.running == 0U)
  {
    return RGBLED_OK;
  }

  /* Stop the pacing timer used to emit DMA trigger events. */
  if (HAL_OK != HAL_TIM_Stop(pobj->pattern_timer.phtim))
  {
    return RGBLED_ERROR;
  }

  pobj->pattern_timer.running = 0U;

  return RGBLED_OK;
}

/**
  * @brief Configure and start the pacing timer at a given trigger-output frequency.
  * @param pobj pointer to the RGB LED object.
  * @param period_ms requested TRGO update period in milliseconds.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_transition_timer(rgbled_obj_t *pobj, uint32_t period_ms)
{
  hal_tim_handle_t *ptimer;
  uint32_t timer_clock_hz;
  uint32_t step_frequency_hz;
  uint32_t total_ticks;
  uint32_t prescaler_div;
  uint32_t prescaler;
  uint32_t period_ticks;

  /* Check that the transition timer exists and the requested period is valid. */
  if ((pobj == NULL) || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0UL))
  {
    return RGBLED_ERROR;
  }

  /* Convert the requested transition duration into a per-sample pacing frequency. */
  step_frequency_hz = (1000UL * RGBLED_PATTERN_SIZE) / period_ms;

  if (step_frequency_hz == 0UL)
  {
    return RGBLED_ERROR;
  }

  /* Derive timer prescaler and period so the pacing timer stays within 16-bit range. */
  ptimer = pobj->pattern_timer.phtim;
  timer_clock_hz = HAL_TIM_GetClockFreq(ptimer);
  if (timer_clock_hz == 0UL)
  {
    return RGBLED_ERROR;
  }

  if (pobj->pattern_timer.running != 0U)
  {
    if (HAL_OK != HAL_TIM_Stop(ptimer))
    {
      return RGBLED_ERROR;
    }
    pobj->pattern_timer.running = 0U;
  }

  /* Compute how many timer input ticks correspond to one trigger-output period. */
  total_ticks = timer_clock_hz / step_frequency_hz;
  if (total_ticks == 0UL)
  {
    /* Keep a minimum period of one timer tick when the requested rate is very high. */
    total_ticks = 1UL;
  }

  /* Expand the prescaler only when needed so the auto-reload value fits in 16 bits. */
  prescaler_div = (total_ticks + (RGBLED_TIM_PERIOD_TICK_LIMIT - 1UL)) / RGBLED_TIM_PERIOD_TICK_LIMIT;
  if (prescaler_div == 0UL)
  {
    prescaler_div = 1UL;
  }

  prescaler = prescaler_div - 1UL;

  /* Convert the total tick budget into the counter period after prescaling. */
  period_ticks = total_ticks / prescaler_div;
  if (period_ticks == 0UL)
  {
    /* Keep a valid non-zero timer period even after integer division rounding. */
    period_ticks = 1UL;
  }

  /* Program the timer base so one timer overflow matches one pattern sample step. */
  if (HAL_OK != HAL_TIM_SetPrescaler(ptimer, prescaler))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_TIM_SetPeriod(ptimer, period_ticks - 1UL))
  {
    return RGBLED_ERROR;
  }

  /* Publish one TRGO pulse on each update event so the DMA nodes advance one sample per pacing period. */
  if (HAL_OK != HAL_TIM_SetTriggerOutput(ptimer, HAL_TIM_TRGO_UPDATE))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_TIM_Start(ptimer))
  {
    return RGBLED_ERROR;
  }

  pobj->pattern_timer.running = 1U;

  return RGBLED_OK;
}
#endif /* RGBLED_PATTERN_PLAYBACK */

/**
  * @brief Apply one PWM brightness value to one channel.
  * @param pchannel pointer to the channel context.
  * @param pulse_value brightness value to program.
  * @retval error status.
  */
static rgbled_status_t rgbled_apply_channel_output(rgbled_tim_ctx_t *pchannel, uint16_t pulse_value)
{
  hal_tim_oc_compare_unit_config_t oc_compare_unit_config;
  hal_tim_oc_compare_unit_t compare_unit;

  /* Validate the channel resources before updating the PWM duty cycle. */
  if ((pchannel == NULL) || (pchannel->phtim == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Program PWM mode by default, then force the output inactive for a zero brightness value. */
  compare_unit = hal_tim_oc_channel_to_compare_unit(pchannel->pwm_channel);
  oc_compare_unit_config.mode = HAL_TIM_OC_PWM1;
  oc_compare_unit_config.pulse = pulse_value;

  if (pulse_value == 0U)
  {
    oc_compare_unit_config.mode = HAL_TIM_OC_FORCED_INACTIVE;
  }

  /* Apply the compare configuration and restart the output channel. */
  if (HAL_OK != HAL_TIM_OC_SetConfigCompareUnit(pchannel->phtim, compare_unit, &oc_compare_unit_config))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_TIM_OC_StartChannel(pchannel->phtim, pchannel->pwm_channel))
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Apply one compare-unit RGB color to the hardware outputs.
  * @param pobj pointer to the RGB LED object.
  * @param red_compare requested red compare value.
  * @param green_compare requested green compare value.
  * @param blue_compare requested blue compare value.
  * @retval error status.
  */
static rgbled_status_t rgbled_apply_color_values(rgbled_obj_t *pobj, uint16_t red_compare,
                                                 uint16_t green_compare, uint16_t blue_compare)
{
  /* Update the three PWM channels with the caller-provided effective color. */
  if (rgbled_apply_channel_output(&pobj->red, red_compare) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_apply_channel_output(&pobj->green, green_compare) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_apply_channel_output(&pobj->blue, blue_compare) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Disable the three RGB outputs.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_disable_outputs(rgbled_obj_t *pobj)
{
  /* Force each hardware channel to its inactive state. */
  if (rgbled_apply_channel_output(&pobj->red, 0U) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_apply_channel_output(&pobj->green, 0U) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_apply_channel_output(&pobj->blue, 0U) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Start each unique TIM instance used by the RGB channels.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_unique_timers(rgbled_obj_t *pobj)
{
  hal_tim_handle_t *pstarted_timers[RGBLED_CHANNEL_COUNT] = {NULL, NULL, NULL};
  rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;
  uint32_t started_count = 0U;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    uint32_t previous_index;
    uint8_t already_started = 0U;
    uint32_t timer_clock_hz;
    uint32_t autoreload_ticks;
    uint32_t achieved_frequency_hz;

    /* Validate the timer handle and detect whether it was already configured. */
    if (pchannels[channel_index]->phtim == NULL)
    {
      return RGBLED_ERROR;
    }

    timer_clock_hz = HAL_TIM_GetClockFreq(pchannels[channel_index]->phtim);

    if (timer_clock_hz < RGBLED_MIN_TIM_CLK_FREQUENCY)
    {
      return RGBLED_ERROR;
    }

    /*
     * Derive the auto-reload value for the target PWM carrier. This assumes an
     * up-counting (edge-aligned) timer configured with repetition counter of 0,
     * so that f_pwm = f_clk / autoreload with ARR = autoreload - 1.
     * Boards binding a center-aligned timer or a non-zero
     * repetition counter are not supported by this driver.
     */
    autoreload_ticks = timer_clock_hz / RGBLED_PWM_FREQUENCY_TARGET;

    /* Reject values that cannot be programmed into the 16-bit auto-reload register. */
    if ((autoreload_ticks != 0UL) && (autoreload_ticks < (RGBLED_TIM_PERIOD_TICK_LIMIT + 1UL)))
    {
      /* Reject clocks that cannot reach the target carrier within the accepted tolerance. */
      achieved_frequency_hz = timer_clock_hz / autoreload_ticks;
    }
    else
    {
      return RGBLED_ERROR;
    }

    if ((achieved_frequency_hz < (RGBLED_PWM_FREQUENCY_TARGET - RGBLED_PWM_FREQUENCY_TOLERANCE))
        || (achieved_frequency_hz > (RGBLED_PWM_FREQUENCY_TARGET + RGBLED_PWM_FREQUENCY_TOLERANCE)))
    {
      return RGBLED_ERROR;
    }

    pchannels[channel_index]->autoreload = (uint16_t)autoreload_ticks;

    /* Some boards map red, green, and blue to the same TIM instance, so start each timer only once. */
    for (previous_index = 0U; previous_index < started_count; previous_index++)
    {
      if (pstarted_timers[previous_index] == pchannels[channel_index]->phtim)
      {
        already_started = 1U;
        break;
      }
    }

    if (already_started == 0U)
    {
      /* Configure the timer base for the RGB PWM carrier and start it once. */
      if (HAL_OK != HAL_TIM_SetPrescaler(pchannels[channel_index]->phtim, 0U))
      {
        return RGBLED_ERROR;
      }

      if (HAL_OK != HAL_TIM_SetPeriod(pchannels[channel_index]->phtim, pchannels[channel_index]->autoreload - 1UL))
      {
        return RGBLED_ERROR;
      }

      if (HAL_OK != HAL_TIM_Start(pchannels[channel_index]->phtim))
      {
        return RGBLED_ERROR;
      }

      pstarted_timers[started_count] = pchannels[channel_index]->phtim;
      started_count++;
    }
  }

  return RGBLED_OK;
}

/**
  * @brief Stop each unique TIM instance used by the RGB channels.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_unique_timers(rgbled_obj_t *pobj)
{
  hal_tim_handle_t *pstopped_timers[RGBLED_CHANNEL_COUNT] = {NULL, NULL, NULL};
  const rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;
  uint32_t previous_index;
  uint32_t stopped_count;
  uint8_t already_stopped;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;

  stopped_count = 0U;

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    /* A missing timer handle is a configuration issue. */
    if (pchannels[channel_index]->phtim == NULL)
    {
      return RGBLED_ERROR;
    }

    already_stopped = 0U;

    /* Stop each unique timer instance only once, since several channels can share the same TIM. */
    for (previous_index = 0U; previous_index < stopped_count; previous_index++)
    {
      if (pstopped_timers[previous_index] == pchannels[channel_index]->phtim)
      {
        already_stopped = 1U;
        break;
      }
    }

    if (already_stopped == 0U)
    {
      if (HAL_OK != HAL_TIM_Stop(pchannels[channel_index]->phtim))
      {
        return RGBLED_ERROR;
      }

      pstopped_timers[stopped_count] = pchannels[channel_index]->phtim;
      stopped_count++;
    }
  }

  return RGBLED_OK;
}

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
/**
  * @brief Limit one user-provided RGB pattern and copy it into the private workspace.
  * @param pobj pointer to the RGB LED object.
  * @param ppattern pointer to the user pattern.
  * @retval error status.
  */
static rgbled_status_t rgbled_prepare_limited_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern)
{
  rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  const uint16_t *pred_src;
  const uint16_t *pgreen_src;
  const uint16_t *pblue_src;
  uint32_t index;

  /* The input pattern is still provided as contiguous R, G and B planes. */
  pred_src = &ppattern[0];
  pgreen_src = &ppattern[RGBLED_PATTERN_SIZE];
  pblue_src = &ppattern[2U * RGBLED_PATTERN_SIZE];

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;

  /* Limit each RGB sample triplet before storing it in the active workspace. */
  for (index = 0U; index < RGBLED_PATTERN_SIZE; index++)
  {
    if (rgbled_scale_percent_to_compare(pchannels[0], pred_src[index], &pobj->red_pattern_buffer[index]) \
        != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }

    if (rgbled_scale_percent_to_compare(pchannels[1], pgreen_src[index], &pobj->green_pattern_buffer[index]) \
        != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }

    if (rgbled_scale_percent_to_compare(pchannels[2], pblue_src[index], &pobj->blue_pattern_buffer[index]) \
        != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
  }

  return RGBLED_OK;
}

/**
  * @brief Generate one synchronized brightness ramp in a per-channel DMA pattern buffer.
  * @param pdst pointer to the destination buffer.
  * @param start_percent initial brightness percentage.
  * @param end_percent final brightness percentage.
  * @param pchannel pointer to the channel context (for per-channel max_duty scaling).
  *
  * The ramp is interpolated linearly in brightness-percentage space, then duty-capped and
  * minimum-on floored through rgbled_scale_percent_to_compare. Percentage-domain interpolation
  * keeps channel transitions synchronized across different max_duty caps.
  */
static void rgbled_fill_channel_transition_pattern(uint16_t *pdst, uint16_t start_percent,
                                                   uint16_t end_percent, const rgbled_tim_ctx_t *pchannel)
{
  uint16_t index;
  uint16_t pattern_size;
  uint16_t pattern_last;
  uint16_t tail_start;
  int16_t  delta_percent;

  /* Validate the destination and channel before filling the ramp. */
  if ((pdst == NULL) || (pchannel == NULL))
  {
    return;
  }

  pattern_size = (uint16_t)RGBLED_PATTERN_SIZE;
  pattern_last = (uint16_t)(pattern_size - 1U);
  tail_start = (uint16_t)(pattern_size - (uint16_t)RGBLED_TRANSITION_TAIL_HOLD);

  /* Signed distance to cover in brightness-percentage space. */
  delta_percent = (int16_t)end_percent - (int16_t)start_percent;

  for (index = 0U; index < pattern_size; index++)
  {
    uint16_t interp_percent;
    uint16_t compare_value = 0U;

    /* Linear interpolation in percentage domain: one integer-percent step at a time. */
    int16_t interp = (int16_t)start_percent +
                     ((delta_percent * (int16_t)index) / (int16_t)pattern_last);

    interp_percent = (uint16_t)interp;

    /* Duty-cap and minimum-on floor the interpolated percentage into a compare value. */
    (void)rgbled_scale_percent_to_compare(pchannel, interp_percent, &compare_value);
    pdst[index] = compare_value;
  }

  /* Hold the final target across the last RGBLED_TRANSITION_TAIL_HOLD samples so the end of the
     transition -- dark when fading to 0% -- stays visible for a wider slice of the playback window
     rather than a single sample. The final sample already holds the target, so only the samples
     immediately before it are overwritten. */
  for (index = tail_start; index < pattern_last; index++)
  {
    pdst[index] = pdst[pattern_last];
  }
}

/**
  * @brief Start the three channel DMA transfers from the active per-channel patterns.
  * @param pobj pointer to the RGB LED object.
  * @param period_ms total playback duration in milliseconds.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_dma_pattern(rgbled_obj_t *pobj, uint32_t period_ms)
{
  rgbled_status_t ret = RGBLED_OK;
  rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  const uint16_t *ppatterns[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;
  ppatterns[0] = pobj->red_pattern_buffer;
  ppatterns[1] = pobj->green_pattern_buffer;
  ppatterns[2] = pobj->blue_pattern_buffer;

  /* Stop any ongoing transfer before arming the new RGB pattern. */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    const hal_tim_oc_compare_unit_t compare_unit =
      hal_tim_oc_channel_to_compare_unit(pchannels[channel_index]->pwm_channel);
    hal_tim_oc_compare_unit_config_t oc_compare_unit_config;

    /* Resolve the channel slice and preload the first PWM sample before DMA start. */
    if (pchannels[channel_index]->phtim == NULL)
    {
      ret = RGBLED_ERROR;
    }
    else
    {
      oc_compare_unit_config.mode = HAL_TIM_OC_PWM1;
      oc_compare_unit_config.pulse = ppatterns[channel_index][0];

      if (HAL_OK != HAL_TIM_OC_SetConfigCompareUnit(pchannels[channel_index]->phtim,
                                                    compare_unit,
                                                    &oc_compare_unit_config))
      {
        ret = RGBLED_ERROR;
      }
      else
      {
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        SCB_CleanDCache_by_Addr((uint8_t *)ppatterns[channel_index],
                                RGBLED_PATTERN_SIZE * sizeof(ppatterns[channel_index][0]));
#endif /* __DCACHE_PRESENT */
        /* Start the DMA stream for the current color plane and mark it as active. */
        if (HAL_OK != HAL_TIM_OC_StartChannel_DMA(pchannels[channel_index]->phtim,
                                                  pchannels[channel_index]->pwm_channel,
                                                  (const uint8_t *)ppatterns[channel_index],
                                                  RGBLED_PATTERN_SIZE * sizeof(ppatterns[channel_index][0])))
        {
          ret = RGBLED_ERROR;
        }
        else
        {
          pchannels[channel_index]->dma_running = 1U;
        }
      }
    }

    if (ret != RGBLED_OK)
    {
      (void)rgbled_stop_dma_patterns(pobj);
      (void)rgbled_disable_outputs(pobj);
      return ret;
    }
  }

  /* Start the pacing timer so each sample is consumed at the requested rate. */
  if (rgbled_start_transition_timer(pobj, period_ms) != RGBLED_OK)
  {
    (void)rgbled_stop_dma_patterns(pobj);
    (void)rgbled_disable_outputs(pobj);
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}
#endif /* RGBLED_PATTERN_PLAYBACK */

/* Exported functions --------------------------------------------------------*/
/**
  * @brief Initialize the RGB LED driver.
  * @param pobj pointer to the RGB LED object to initialize.
  * @param id configuration identifier forwarded to the IO layer.
  * @retval error status.
  */
rgbled_status_t rgbled_init(rgbled_obj_t *pobj, uint32_t id)
{
  /* Check the object pointer and reject reinitialization of an active instance. */
  if (pobj == NULL)
  {
    return RGBLED_INVALID_PARAM;
  }

  if (pobj->state == 1U)
  {
    return RGBLED_ERROR;
  }

  /* Reset the software state before the board layer binds the hardware resources. */
  pobj->id = id;
  pobj->color.red_color_value = 0U;
  pobj->color.green_color_value = 0U;
  pobj->color.blue_color_value = 0U;
  /* Default every channel duty-cycle cap to full scale. The generated IO layer overrides this with
     the buck-converter current-limited cap. */
  pobj->red.max_duty_cycle = RGBLED_RED_MAX_DUTY;
  pobj->green.max_duty_cycle = RGBLED_GREEN_MAX_DUTY;
  pobj->blue.max_duty_cycle = RGBLED_BLUE_MAX_DUTY;
  pobj->red.dma_running = 0U;
  pobj->green.dma_running = 0U;
  pobj->blue.dma_running = 0U;

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  pobj->pattern_timer.running = 0U;
#endif /* RGBLED_PATTERN_PLAYBACK */

  /* Initialize the board-specific IO layer and start the timers used by the channels. */
  if (rgbled_io_init(pobj) != 0)
  {
    return RGBLED_ERROR;
  }

  /* Force all outputs inactive before exposing the instance as initialized. */
  if (rgbled_disable_outputs(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_start_unique_timers(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  pobj->state = 1U;

  return RGBLED_OK;
}

/**
  * @brief Deinitialize the RGB LED driver.
  * @param pobj pointer to the RGB LED object to deinitialize.
  * @retval error status.
  */
rgbled_status_t rgbled_deinit(rgbled_obj_t *pobj)
{
  /* Deinitialization is allowed only on a valid initialized object. */
  if ((pobj == NULL) || (pobj->state != 1U))
  {
    return RGBLED_ERROR;
  }

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  /* Stop any active PWM-duty DMA playback. */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
#endif /* RGBLED_PATTERN_PLAYBACK */

  /* Turn the outputs off and mark the instance as reset. */
  if (rgbled_disable_outputs(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Stop each unique PWM timer started by the driver. */
  if (rgbled_stop_unique_timers(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  pobj->state = 0U;
  return RGBLED_OK;
}

/**
  * @brief Set the per-channel maximum duty-cycle caps.
  * @param pobj pointer to the RGB LED object.
  * @param max_redduty maximum red duty-cycle percentage.
  * @param max_greenduty maximum green duty-cycle percentage.
  * @param max_blueduty maximum blue duty-cycle percentage.
  * @retval error status.
  */
rgbled_status_t rgbled_set_max_duty_cycle(rgbled_obj_t *pobj, uint8_t max_redduty, uint8_t max_greenduty,
                                          uint8_t max_blueduty)
{
  /* Validate the instance and reject caps above the full-scale duty cycle. */
  if ((pobj == NULL) || (pobj->state != 1U))
  {
    return RGBLED_INVALID_PARAM;
  }

  if ((max_redduty > RGBLED_MAX_DUTY) || (max_greenduty > RGBLED_MAX_DUTY) || (max_blueduty > RGBLED_MAX_DUTY))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Store the new per-channel caps applied by the brightness converter. */
  pobj->red.max_duty_cycle = max_redduty;
  pobj->green.max_duty_cycle = max_greenduty;
  pobj->blue.max_duty_cycle = max_blueduty;

  return RGBLED_OK;
}

/**
  * @brief Apply one static RGB color.
  * @param pobj pointer to the RGB LED object.
  * @param pcolor pointer to the requested color.
  * @retval error status.
  */
rgbled_status_t rgbled_set_color(rgbled_obj_t *pobj, const rgbled_color_t *pcolor)
{
  uint16_t red_compare;
  uint16_t green_compare;
  uint16_t blue_compare;
  rgbled_color_t clamped_color;

  /* Validate the instance and the requested RGB color. */
  if ((pobj == NULL) || (pobj->state != 1U) || (pcolor == NULL))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Clamp the requested color before applying it to the hardware. */
  if (rgbled_limit_color_values(pobj, pcolor, &red_compare, &green_compare, &blue_compare) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Keep the clamped percentages so the public getter echoes back the requested values. */
  clamped_color.red_color_value =
    (pcolor->red_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : pcolor->red_color_value;
  clamped_color.green_color_value =
    (pcolor->green_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : pcolor->green_color_value;
  clamped_color.blue_color_value =
    (pcolor->blue_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : pcolor->blue_color_value;

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  /* Stop any running pattern playback because a static color is about to be applied. */

  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
#endif /* RGBLED_PATTERN_PLAYBACK */

  /* Program the three channels and store the effective color in the object. */
  if (rgbled_apply_color_values(pobj, red_compare, green_compare, blue_compare) != RGBLED_OK)
  {
    /* A channel update failed mid-way; fail safe to a coherent dark state. */
    (void)rgbled_disable_outputs(pobj);
    return RGBLED_ERROR;
  }

  pobj->color = clamped_color;

  return RGBLED_OK;
}

/**
  * @brief Return the current RGB color stored by the driver.
  * @param pobj pointer to the RGB LED object.
  * @param pcolor pointer to the structure receiving the current color.
  * @retval error status.
  */
rgbled_status_t rgbled_get_color(rgbled_obj_t *pobj, rgbled_color_t *pcolor)
{
  /* Return the latest color value stored by the driver for the active instance. */
  if ((pobj == NULL) || (pobj->state != 1U) || (pcolor == NULL))
  {
    return RGBLED_INVALID_PARAM;
  }

  pcolor->red_color_value = pobj->color.red_color_value;
  pcolor->green_color_value = pobj->color.green_color_value;
  pcolor->blue_color_value = pobj->color.blue_color_value;
  return RGBLED_OK;
}

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
/**
  * @brief Fade from the current color to a target color using DMA.
  * @param pobj pointer to the RGB LED object.
  * @param ptarget_color pointer to the requested target color.
  * @param period_ms duration of one fade ramp in milliseconds.
  * @retval error status.
  */
rgbled_status_t rgbled_fade_to_color(rgbled_obj_t *pobj, const rgbled_color_t *ptarget_color, uint16_t period_ms)
{
  rgbled_color_t clamped_target_color;

  /* Validate the instance and the requested target color. */
  if ((pobj == NULL) || (pobj->state != 1U) || (ptarget_color == NULL)
      || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0U))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Keep the clamped percentages so the public getter echoes back the requested values. */
  clamped_target_color.red_color_value =
    (ptarget_color->red_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : ptarget_color->red_color_value;
  clamped_target_color.green_color_value =
    (ptarget_color->green_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : ptarget_color->green_color_value;
  clamped_target_color.blue_color_value =
    (ptarget_color->blue_color_value > RGBLED_MAX_DUTY) ? RGBLED_MAX_DUTY : ptarget_color->blue_color_value;

  /* Fill per-channel transition patterns by interpolating from the current brightness percentage to
     the target percentage, each sample is duty-capped and minimum-on floored, keeping every channel
     synchronized to reach its target (and dark) at the same time. */
  rgbled_fill_channel_transition_pattern(pobj->red_pattern_buffer,
                                         pobj->color.red_color_value,
                                         clamped_target_color.red_color_value,
                                         &pobj->red);

  rgbled_fill_channel_transition_pattern(pobj->green_pattern_buffer,
                                         pobj->color.green_color_value,
                                         clamped_target_color.green_color_value,
                                         &pobj->green);

  rgbled_fill_channel_transition_pattern(pobj->blue_pattern_buffer,
                                         pobj->color.blue_color_value,
                                         clamped_target_color.blue_color_value,
                                         &pobj->blue);

  /* Launch DMA playback from the per-channel pattern buffers. */
  if (rgbled_start_dma_pattern(pobj, period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  pobj->color = clamped_target_color;

  return RGBLED_OK;
}

/**
  * @brief Play one full RGB pattern using DMA.
  * @param pobj pointer to the RGB LED object.
  * @param ppattern pointer to the input pattern laid out as red, green, then blue blocks.
  * @param period_ms duration of one pass through the pattern in milliseconds.
  * @retval error status.
  */
rgbled_status_t rgbled_play_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern, uint16_t period_ms)
{
  rgbled_color_t final_color;

  /* Validate the instance and the user pattern before preparing playback. */
  if ((pobj == NULL) || (pobj->state != 1U) || (ppattern == NULL)
      || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0U))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Copy the pattern into the private workspace while enforcing brightness limits. */
  if (rgbled_prepare_limited_pattern(pobj, ppattern) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Start the DMA sequence from the per-channel pattern buffers. */
  if (rgbled_start_dma_pattern(pobj, period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Cache the last user pattern sample (clamped), matching the set/fade getter semantics. */
  final_color.red_color_value = (uint8_t)((ppattern[RGBLED_PATTERN_SIZE - 1UL] > RGBLED_MAX_DUTY)
                                          ? RGBLED_MAX_DUTY : ppattern[RGBLED_PATTERN_SIZE - 1UL]);
  final_color.green_color_value = (uint8_t)((ppattern[(2UL * RGBLED_PATTERN_SIZE) - 1UL] > RGBLED_MAX_DUTY)
                                            ? RGBLED_MAX_DUTY : ppattern[(2UL * RGBLED_PATTERN_SIZE) - 1UL]);
  final_color.blue_color_value = (uint8_t)((ppattern[(3UL * RGBLED_PATTERN_SIZE) - 1UL] > RGBLED_MAX_DUTY)
                                           ? RGBLED_MAX_DUTY : ppattern[(3UL * RGBLED_PATTERN_SIZE) - 1UL]);
  pobj->color = final_color;

  return RGBLED_OK;
}
#endif /* RGBLED_PATTERN_PLAYBACK */

/**
  * @brief Initialize the IO layer.
  * @param pobj RGB LED object to be bound to board resources.
  * @note This function must be overridden with a proper implementation.
  * @retval error status: 0 means OK, anything else is an error.
  */
__WEAK int32_t rgbled_io_init(rgbled_obj_t *pobj)
{
  /* Default weak implementation: board code must override this hook. */
  (void)(pobj);
  return -1;
}
