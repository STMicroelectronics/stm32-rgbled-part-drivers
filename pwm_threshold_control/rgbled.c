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
/*!< Free-running pacing-timer trigger frequency (Hz) so the externally triggered DAC reloads its
     current threshold (DHR -> DOR) and a static color set with rgbled_set_color is applied
     continuously. Kept well below the PWM carrier, the exact value only bounds how quickly a newly
     written threshold becomes effective. */
#define RGBLED_ADVANCED_DAC_REFRESH_FREQUENCY   10000UL

/* Private functions prototype -----------------------------------------------*/
#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
static rgbled_status_t rgbled_stop_transition_timer(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_transition_timer(rgbled_obj_t *pobj, uint32_t period_ms);
static rgbled_status_t rgbled_start_pacing_timer(rgbled_obj_t *pobj, uint32_t step_frequency_hz);
#endif /* RGBLED_PATTERN_PLAYBACK */
static rgbled_status_t rgbled_apply_channel_output(rgbled_tim_ctx_t *pchannel, uint16_t pulse_value);
static rgbled_status_t rgbled_disable_outputs(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_unique_timers(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_unique_timers(rgbled_obj_t *pobj);
static uint16_t rgbled_brightness_to_permille(uint16_t percent_value);
static rgbled_status_t rgbled_current_percent_to_dac(uint16_t percent_value, uint16_t *pcode);
static rgbled_status_t rgbled_scale_percent_to_compare(const rgbled_tim_ctx_t *pchannel, uint16_t percent_value,
                                                       uint16_t *pcompare_value);
static rgbled_status_t rgbled_apply_channel_current(rgbled_tim_ctx_t *pchannel, uint16_t code, uint16_t percent_value);
static rgbled_status_t rgbled_apply_current_values(rgbled_obj_t *pobj, const rgbled_color_t *ppercent);
static rgbled_status_t rgbled_validate_advanced_topology(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_setup_advanced_channel(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_start_advanced_control(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_advanced_channel(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_stop_advanced_control(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_set_color_advanced(rgbled_obj_t *pobj, const rgbled_color_t *pcolor);
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
static void rgbled_fill_channel_transition_pattern(uint16_t *pcurrent_dst, uint16_t *pduty_dst,
                                                   uint16_t start_percent, uint16_t end_percent,
                                                   const rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_prepare_limited_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern);
static rgbled_status_t rgbled_stop_channel_dma(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_stop_dma_patterns(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_dma_pattern(rgbled_obj_t *pobj, uint32_t period_ms);
#endif /* USE_HAL_DAC_DMA */

/* Private functions ---------------------------------------------------------*/
#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
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
  * @param step_frequency_hz requested TRGO update frequency in hertz.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_pacing_timer(rgbled_obj_t *pobj, uint32_t step_frequency_hz)
{
  hal_tim_handle_t *ptimer;
  uint32_t timer_clock_hz;
  uint32_t total_ticks;
  uint32_t prescaler_div;
  uint32_t prescaler;
  uint32_t period_ticks;

  /* Check that the pacing timer exists and the requested frequency is valid. */
  if ((pobj == NULL) || (pobj->pattern_timer.phtim == NULL) || (step_frequency_hz == 0UL))
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

  /* Program the timer base so one timer overflow matches one trigger-output period. */
  if (HAL_OK != HAL_TIM_SetPrescaler(ptimer, prescaler))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_TIM_SetPeriod(ptimer, period_ticks - 1UL))
  {
    return RGBLED_ERROR;
  }

  /* Publish one TRGO pulse on each update event. */
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

/**
  * @brief Configure and start the timer that paces a DMA transition.
  * @param pobj pointer to the RGB LED object.
  * @param period_ms total transition duration in milliseconds.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_transition_timer(rgbled_obj_t *pobj, uint32_t period_ms)
{
  uint32_t step_frequency_hz;

  /* Check that the transition timer exists and the requested period is valid. */
  if ((pobj == NULL) || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0UL))
  {
    return RGBLED_ERROR;
  }

  /* Convert the requested transition duration into a per-sample pacing frequency so that one
     timer overflow (TRGO) advances the DMA nodes by one pattern sample. */
  step_frequency_hz = (1000UL * RGBLED_PATTERN_SIZE) / period_ms;

  if (rgbled_start_pacing_timer(pobj, step_frequency_hz) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}
#endif /* RGBLED_PATTERN_PLAYBACK */

/**
  * @brief Drive one channel carrier at full duty or force it inactive.
  * @param pchannel pointer to the channel context.
  * @param pulse_value carrier compare value to program.
  * @retval error status.
  */
static rgbled_status_t rgbled_apply_channel_output(rgbled_tim_ctx_t *pchannel, uint16_t pulse_value)
{
  hal_tim_oc_compare_unit_config_t oc_compare_unit_config;
  hal_tim_oc_compare_unit_t compare_unit;

  /* Validate the channel resources before updating the carrier duty cycle. */
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

/**
  * @brief Map a brightness percentage to a perceived-light fraction in per-mille.
  * @param percent_value user brightness percentage.
  * @retval gamma-corrected brightness in per-mille (0..1000).
  * @note Used to shape the PWM duty cycle (brightness). With gamma correction disabled the mapping
  *       is a plain linear scaling, so callers can treat the result as a 0..1000 light fraction
  *       regardless of the build option.
  */
static uint16_t rgbled_brightness_to_permille(uint16_t percent_value)
{
#if (RGBLED_GAMMA_CORRECTION == 1)
  /* Brightness gamma table: maps each percentage (index 0..100) to a per-mille duty fraction (0..1000)
     as round(1000 * (percent / 100) ^ 0.87). The sub-unity exponent lifts the dim end so low
     percentages stay visible and evenly spaced despite the coarse PWM duty resolution. It shapes the
     PWM duty cycle only (brightness), it never touches the DAC current threshold. */
  static const uint16_t rgbled_gamma_permille[RGBLED_AUTORELOAD_PERCENTAGE + 1U] =
  {
    0U,   18U,   33U,   47U,   60U,   73U,   86U,   98U,  110U,  122U,
    134U,  146U,  157U,  168U,  180U,  191U,  202U,  213U,  224U,  235U,
    245U,  256U,  267U,  277U,  288U,  298U,  308U,  319U,  329U,  340U,
    350U,  360U,  370U,  380U,  390U,  400U,  410U,  420U,  430U,  440U,
    450U,  459U,  469U,  479U,  488U,  498U,  508U,  517U,  527U,  536U,
    546U,  555U,  565U,  575U,  584U,  593U,  603U,  612U,  621U,  631U,
    640U,  650U,  659U,  668U,  677U,  686U,  696U,  705U,  714U,  723U,
    733U,  742U,  750U,  760U,  769U,  778U,  788U,  796U,  805U,  814U,
    823U,  831U,  841U,  850U,  859U,  868U,  876U,  885U,  894U,  903U,
    912U,  921U,  930U,  939U,  947U,  956U,  965U,  973U,  982U,  991U,
    1000U
  };
#endif /* RGBLED_GAMMA_CORRECTION */
  uint16_t limited_percent;

  limited_percent = (uint16_t)((percent_value > RGBLED_AUTORELOAD_PERCENTAGE)
                               ? RGBLED_AUTORELOAD_PERCENTAGE : percent_value);
#if (RGBLED_GAMMA_CORRECTION == 1)
  return rgbled_gamma_permille[limited_percent];
#else
  return (uint16_t)((limited_percent * 1000UL) / RGBLED_AUTORELOAD_PERCENTAGE);
#endif /* RGBLED_GAMMA_CORRECTION */
}

/**
  * @brief Convert one brightness percentage into the matching DAC current-threshold code.
  * @param percent_value user brightness percentage (0..100).
  * @param pcode pointer receiving the DAC code.
  * @retval error status.
  * @note The current threshold is mapped LINEARLY onto the safe operating window
  *       [RGBLED_DAC_CODE_FLOOR .. RGBLED_DAC_CODE_MAX] = [50 mA .. 100 mA]: a 0 % request is fully
  *       off, while any nonzero request stays at or above the current floor so the comparator keeps
  *       regulating out of the low-current dead zone. This mapping is independent from the perceptual
  *       gamma curve, which shapes the PWM duty cycle only. Peak current can never exceed
  *       RGBLED_DAC_CODE_MAX, preserving the overcurrent protection.
  */
static rgbled_status_t rgbled_current_percent_to_dac(uint16_t percent_value, uint16_t *pcode)
{
  uint16_t limited_percent;
  uint16_t dac_code;

  if (pcode == NULL)
  {
    return RGBLED_ERROR;
  }

  /* A 0 % request is fully off; the carrier duty cycle is forced inactive for this channel too. */
  if (percent_value == 0U)
  {
    *pcode = 0U;
    return RGBLED_OK;
  }

  /* Clamp to full scale, then map linearly onto the [floor .. max] current window, rounding to
     nearest: code = floor + (max - floor) * percent / 100. limited_percent is bounded to
     RGBLED_AUTORELOAD_PERCENTAGE, so the result never exceeds RGBLED_DAC_CODE_MAX and fits a
     16-bit code. */
  limited_percent = (percent_value > RGBLED_AUTORELOAD_PERCENTAGE) ? RGBLED_AUTORELOAD_PERCENTAGE : percent_value;
  dac_code = RGBLED_DAC_CODE_FLOOR + ((((RGBLED_DAC_CODE_MAX - RGBLED_DAC_CODE_FLOOR) * limited_percent)
                                       + (RGBLED_AUTORELOAD_PERCENTAGE / 2U)) / RGBLED_AUTORELOAD_PERCENTAGE);

  *pcode = dac_code;
  return RGBLED_OK;
}

/**
  * @brief Apply one current-threshold code to one channel DAC with variable duty-cycle.
  * @param pchannel pointer to the channel context.
  * @param code DAC code to program.
  * @param percent_value brightness percentage for duty-cycle modulation.
  * @retval error status.
  */
static rgbled_status_t rgbled_apply_channel_current(rgbled_tim_ctx_t *pchannel, uint16_t code, uint16_t percent_value)
{
  uint16_t compare_value;

  if ((pchannel == NULL) || (pchannel->phdac == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Program the comparator threshold that caps this channel forward current. */
  if (HAL_OK != HAL_DAC_SetChannelData(pchannel->phdac, pchannel->dac_channel, (uint32_t)code))
  {
    return RGBLED_ERROR;
  }

  /* Convert brightness percentage to carrier duty-cycle compare value (gamma correction via LUT). */
  if (rgbled_scale_percent_to_compare(pchannel, percent_value, &compare_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Drive the carrier at the calculated duty-cycle, force inactive for zero current. */
  if (rgbled_apply_channel_output(pchannel, compare_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Convert one brightness percentage into the nearest effective timer compare (duty) value.
  * @param pchannel pointer to the PWM channel context.
  * @param percent_value user brightness percentage (0..100).
  * @param pcompare_value pointer receiving the effective compare value.
  * @retval error status.
  * @note Perceived brightness is carried entirely by the PWM duty cycle: the percentage is first
  *       gamma-corrected and then scaled by the per-channel maximum duty cycle, so equal percentage
  *       steps look like equal brightness steps. This is independent from the DAC current threshold,
  *       which stays in its safe 50..100 mA window. A 0 % request yields a 0 compare (output off).
  */
static rgbled_status_t rgbled_scale_percent_to_compare(const rgbled_tim_ctx_t *pchannel, uint16_t percent_value,
                                                       uint16_t *pcompare_value)
{
  uint16_t brightness_permille;
  uint32_t compare_value;

  /* Validate the channel context and output pointer before converting. */
  if ((pchannel == NULL) || (pcompare_value == NULL) || (pchannel->phtim == NULL)
      || (pchannel->autoreload == 0U))
  {
    return RGBLED_ERROR;
  }

  /* Gamma-correct the request to a perceived-light fraction (per-mille, 0..1000). */
  brightness_permille = rgbled_brightness_to_permille(percent_value);

  /* Scale by the per-channel maximum duty-cycle cap and the timer period, rounding to nearest:
     compare = autoreload * (permille / 1000) * (max_duty / RGBLED_MAX_DUTY). */
  compare_value = (((uint32_t)pchannel->autoreload * brightness_permille * pchannel->max_duty_cycle)
                   + ((1000UL * RGBLED_MAX_DUTY) / 2UL))
                  / (1000UL * RGBLED_MAX_DUTY);

  *pcompare_value = (uint16_t)compare_value;
  return RGBLED_OK;
}
/**
  * @brief Convert and apply one static RGB request as per-channel current thresholds and duty cycles.
  * @param pobj pointer to the RGB LED object.
  * @param ppercent pointer to the requested per-channel percentages.
  * @retval error status.
  * @note Each channel gets a linear current threshold (50..100 mA, overcurrent + dead-zone
  *       protection) and a gamma-corrected PWM duty cycle (perceived brightness), the two are set
  *       independently by rgbled_apply_channel_current.
  */
static rgbled_status_t rgbled_apply_current_values(rgbled_obj_t *pobj, const rgbled_color_t *ppercent)
{
  uint16_t channel_code;

  if ((pobj == NULL) || (ppercent == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Red channel: linear current threshold + gamma-corrected duty cycle. */
  if (rgbled_current_percent_to_dac(ppercent->red_color_value, &channel_code) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_apply_channel_current(&pobj->red, channel_code, ppercent->red_color_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Green channel: linear current threshold + gamma-corrected duty cycle. */
  if (rgbled_current_percent_to_dac(ppercent->green_color_value, &channel_code) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_apply_channel_current(&pobj->green, channel_code, ppercent->green_color_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Blue channel: linear current threshold + gamma-corrected duty cycle. */
  if (rgbled_current_percent_to_dac(ppercent->blue_color_value, &channel_code) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_apply_channel_current(&pobj->blue, channel_code, ppercent->blue_color_value) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Reject topologies that share a comparator or DAC channel between two colors.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  * @note Independent per-channel current regulation requires a dedicated comparator and a
  *       dedicated DAC channel per color. Sharing either would let one channel overwrite the
  *       current threshold or the regulation of another channel.
  */
static rgbled_status_t rgbled_validate_advanced_topology(rgbled_obj_t *pobj)
{
  const rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;
  uint32_t other_index;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    for (other_index = channel_index + 1U; other_index < RGBLED_CHANNEL_COUNT; other_index++)
    {
      /* Two channels cannot regulate their current from the same comparator. */
      if ((pchannels[channel_index]->phcomp != NULL)
          && (pchannels[channel_index]->phcomp == pchannels[other_index]->phcomp))
      {
        return RGBLED_ERROR;
      }

      /* Two channels cannot derive independent thresholds from the same DAC channel. */
      if ((pchannels[channel_index]->phdac != NULL)
          && (pchannels[channel_index]->phdac == pchannels[other_index]->phdac)
          && (pchannels[channel_index]->dac_channel == pchannels[other_index]->dac_channel))
      {
        return RGBLED_ERROR;
      }
    }
  }

  return RGBLED_OK;
}

/**
  * @brief Wire one channel for comparator-based current-mode regulation.
  * @param pchannel pointer to the channel context.
  * @retval error status.
  * @note The channel output is left forced inactive (dark) until a non-zero current is
  *       requested. When a color is applied the carrier is driven at full duty and the
  *       comparator output clears the timer OCREF every cycle as soon as the sensed current
  *       reaches the DAC threshold. The OCREF-clear source and enable are configured by the
  *       board in the TIM peripheral.
  */
static rgbled_status_t rgbled_setup_advanced_channel(rgbled_tim_ctx_t *pchannel)
{
  /* Validate every resource used by the current-mode path. */
  if ((pchannel == NULL) || (pchannel->phtim == NULL) || (pchannel->phcomp == NULL)
      || (pchannel->phdac == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Start the DAC and program the current threshold to zero so the channel stays dark until a color
     is applied. */
  if (HAL_OK != HAL_DAC_StartChannel(pchannel->phdac, pchannel->dac_channel))
  {
    return RGBLED_ERROR;
  }
  if (HAL_OK != HAL_DAC_SetChannelData(pchannel->phdac, pchannel->dac_channel, 0U))
  {
    return RGBLED_ERROR;
  }

  /* Enable the comparator sensing the LED current. */
  if (HAL_OK != HAL_COMP_Start(pchannel->phcomp))
  {
    return RGBLED_ERROR;
  }

  /* Keep the output forced inactive until a non-zero current is requested. With a zero threshold,
    the carrier/comparator path can still produce a short pulse before OCREF clear. */
  if (rgbled_apply_channel_output(pchannel, 0U) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Start comparator and DAC current-mode regulation on all three channels.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_start_advanced_control(rgbled_obj_t *pobj)
{
  /* Reject topologies that share a comparator or DAC channel between two colors. */
  if (rgbled_validate_advanced_topology(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_setup_advanced_channel(&pobj->red) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_setup_advanced_channel(&pobj->green) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_setup_advanced_channel(&pobj->blue) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  /* The externally triggered DAC loads its threshold (DHR -> DOR) only on a pacing-timer
     trigger, so free-run that timer. Static colors set with rgbled_set_color are then converted
     continuously, the DMA fade/pattern paths temporarily re-pace the same timer. */
  if (pobj->pattern_timer.phtim != NULL)
  {
    if (rgbled_start_pacing_timer(pobj, RGBLED_ADVANCED_DAC_REFRESH_FREQUENCY) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
  }
#endif /* RGBLED_PATTERN_PLAYBACK */

  return RGBLED_OK;
}

/**
  * @brief Disconnect the current-mode regulation path on one channel.
  * @param pchannel pointer to the channel context.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_advanced_channel(rgbled_tim_ctx_t *pchannel)
{
  if ((pchannel == NULL) || (pchannel->phtim == NULL) || (pchannel->phcomp == NULL)
      || (pchannel->phdac == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Force the current threshold to zero before tearing down the regulation path. */
  if (HAL_OK != HAL_DAC_SetChannelData(pchannel->phdac, pchannel->dac_channel, 0U))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_COMP_Stop(pchannel->phcomp))
  {
    return RGBLED_ERROR;
  }

  if (HAL_OK != HAL_DAC_StopChannel(pchannel->phdac, pchannel->dac_channel))
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Stop comparator and DAC current-mode regulation on all three channels.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_advanced_control(rgbled_obj_t *pobj)
{
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
  /* Stop any running fade or pattern playback before tearing down the channels
     (this also stops the free-running pacing timer). */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
#elif defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  /* Stop the free-running pacing timer started for the static current-mode path. */
  if (rgbled_stop_transition_timer(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
#endif /* USE_HAL_DAC_DMA */

  if (rgbled_stop_advanced_channel(&pobj->red) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_stop_advanced_channel(&pobj->green) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  if (rgbled_stop_advanced_channel(&pobj->blue) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}

/**
  * @brief Apply one static RGB color as per-channel current thresholds.
  * @param pobj pointer to the RGB LED object.
  * @param pcolor pointer to the requested color (percentage of the maximum LED current).
  * @retval error status.
  */
static rgbled_status_t rgbled_set_color_advanced(rgbled_obj_t *pobj, const rgbled_color_t *pcolor)
{
  rgbled_color_t requested_color;

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
  /* A static color supersedes any running fade or pattern playback. */
  if ((pobj->red.dma_running == 1U) || (pobj->green.dma_running == 1U) || (pobj->blue.dma_running == 1U))
  {
    if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }

    /* Re-enable the DAC channels disabled by the asynchronous DMA stop before writing static data. */
    if (HAL_OK != HAL_DAC_StartChannel(pobj->red.phdac, pobj->red.dac_channel))
    {
      return RGBLED_ERROR;
    }
    if (HAL_OK != HAL_DAC_StartChannel(pobj->green.phdac, pobj->green.dac_channel))
    {
      return RGBLED_ERROR;
    }
    if (HAL_OK != HAL_DAC_StartChannel(pobj->blue.phdac, pobj->blue.dac_channel))
    {
      return RGBLED_ERROR;
    }

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
    /* Resume the free-running pacing trigger stopped together with the DMA so the new static
       threshold is converted (DHR -> DOR) by the externally triggered DAC. */
    if (pobj->pattern_timer.phtim != NULL)
    {
      if (rgbled_start_pacing_timer(pobj, RGBLED_ADVANCED_DAC_REFRESH_FREQUENCY) != RGBLED_OK)
      {
        return RGBLED_ERROR;
      }
    }
#endif /* RGBLED_PATTERN_PLAYBACK */
  }
#endif /* USE_HAL_DAC_DMA */

  /* Clamp each channel to the maximum current cap before mapping it to a DAC code. */
  requested_color.red_color_value = (uint16_t)((pcolor->red_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                               ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->red_color_value);
  requested_color.green_color_value = (uint16_t)((pcolor->green_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                                 ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->green_color_value);
  requested_color.blue_color_value = (uint16_t)((pcolor->blue_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                                ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->blue_color_value);

  /* Program the per-channel current thresholds through the DAC. */
  if (rgbled_apply_current_values(pobj, &requested_color) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Cache the clamped percentages so rgbled_get_color reports the effective color. */
  pobj->color = requested_color;
  return RGBLED_OK;
}

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief Fill one channel current + duty ramp by interpolating in brightness-percentage space.
  * @param pcurrent_dst pointer to the destination DAC current-code buffer.
  * @param pduty_dst pointer to the destination PWM duty-compare buffer.
  * @param start_percent initial brightness percentage.
  * @param end_percent final brightness percentage.
  * @param pchannel pointer to the channel context (for the per-channel duty scaling).
  * @note Interpolating in percentage (not in compare/code units) keeps the current threshold and the
  *       gamma-corrected duty in step and lets every channel reach its target together. Each sample
  *       yields the same (current, duty) pair rgbled_set_color would apply for that percentage.
  */
static void rgbled_fill_channel_transition_pattern(uint16_t *pcurrent_dst, uint16_t *pduty_dst,
                                                   uint16_t start_percent, uint16_t end_percent,
                                                   const rgbled_tim_ctx_t *pchannel)
{
  uint16_t index;
  int16_t delta_percent;

  if ((pcurrent_dst == NULL) || (pduty_dst == NULL) || (pchannel == NULL))
  {
    return;
  }

  /* Signed distance to cover in brightness-percentage space. */
  delta_percent = (int16_t)end_percent - (int16_t)start_percent;

  const int16_t pattern_last = (int16_t)(RGBLED_PATTERN_SIZE - 1U);

  for (index = 0U; index < RGBLED_PATTERN_SIZE; index++)
  {
    int16_t interp = (int16_t)start_percent +
                     ((delta_percent * (int16_t)index) /
                      pattern_last);

    const uint16_t interp_percent = (uint16_t)interp;

    /* Linear current threshold (50..100 mA) and gamma-corrected duty for this interpolated percent. */
    (void)rgbled_current_percent_to_dac(interp_percent, &pcurrent_dst[index]);
    (void)rgbled_scale_percent_to_compare(pchannel, interp_percent, &pduty_dst[index]);
  }
}

/**
  * @brief Convert one user RGB pattern into the private current + duty DMA workspaces.
  * @param pobj pointer to the RGB LED object.
  * @param ppattern pointer to the user pattern (brightness percentages).
  * @retval error status.
  */
static rgbled_status_t rgbled_prepare_limited_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern)
{
  const uint16_t *pred_src = &ppattern[0];
  const uint16_t *pgreen_src = &ppattern[RGBLED_PATTERN_SIZE];
  const uint16_t *pblue_src = &ppattern[2U * RGBLED_PATTERN_SIZE];
  uint32_t index;

  /* Convert each RGB sample into a DAC current-threshold code (50..100 mA window) and a matching
     gamma-corrected PWM duty compare value, filling both per-channel DMA workspaces. */
  for (index = 0U; index < RGBLED_PATTERN_SIZE; index++)
  {
    if (rgbled_current_percent_to_dac(pred_src[index], &pobj->red_pattern_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    if (rgbled_current_percent_to_dac(pgreen_src[index], &pobj->green_pattern_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    if (rgbled_current_percent_to_dac(pblue_src[index], &pobj->blue_pattern_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }

    if (rgbled_scale_percent_to_compare(&pobj->red, pred_src[index], &pobj->red_duty_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    if (rgbled_scale_percent_to_compare(&pobj->green, pgreen_src[index], &pobj->green_duty_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    if (rgbled_scale_percent_to_compare(&pobj->blue, pblue_src[index], &pobj->blue_duty_buffer[index]) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
  }

  return RGBLED_OK;
}

/**
  * @brief Stop both DMA transfers (current threshold + duty cycle) on one channel if active.
  * @param pchannel pointer to the channel context.
  * @retval error status.
  * @note The DAC and TIM OC DMA aborts are asynchronous, the channel requires time to reach the
  *       idle state required before a new DMA transfer can be armed.
  */
static rgbled_status_t rgbled_stop_channel_dma(rgbled_tim_ctx_t *pchannel)
{
  /* Validate the channel resources before touching the DMA paths. */
  if ((pchannel == NULL) || (pchannel->phdac == NULL) || (pchannel->phtim == NULL))
  {
    return RGBLED_ERROR;
  }

  /* Stop both DMA streams only when this channel is currently active. */
  if (pchannel->dma_running == 1U)
  {
    if (HAL_OK != HAL_DAC_StopChannel_DMA(pchannel->phdac, pchannel->dac_channel))
    {
      return RGBLED_ERROR;
    }

    if (HAL_OK != HAL_TIM_OC_StopChannel_DMA(pchannel->phtim, pchannel->pwm_channel))
    {
      return RGBLED_ERROR;
    }

    pchannel->dma_running = 0U;
  }

  return RGBLED_OK;
}

/**
  * @brief Stop all active DMA resources (current + duty) used by the RGB LED object.
  * @param pobj pointer to the RGB LED object.
  * @retval error status.
  */
static rgbled_status_t rgbled_stop_dma_patterns(rgbled_obj_t *pobj)
{
  /* Stop the three channel transfers before stopping the pacing timer. */
  if (rgbled_stop_channel_dma(&pobj->red) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_stop_channel_dma(&pobj->green) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  if (rgbled_stop_channel_dma(&pobj->blue) != RGBLED_OK)
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
  * @brief Start the per-channel dual DMA transfers (current threshold + duty cycle).
  * @param pobj pointer to the RGB LED object.
  * @param period_ms total playback duration in milliseconds.
  * @retval error status.
  * @note Each channel streams two synchronized DMA buffers paced by the same trigger timer: the DAC
  *       reloads its current threshold from the current-code buffer, and the TIM output compare
  *       reloads its duty cycle from the duty-compare buffer. Both advance one sample per pacing
  *       trigger, so current and brightness stay in step for the whole playback. Each DAC channel is
  *       returned to the idle state before its DMA stream is armed.
  */
static rgbled_status_t rgbled_start_dma_pattern(rgbled_obj_t *pobj, uint32_t period_ms)
{
  rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  const uint16_t *pcurrent_patterns[RGBLED_CHANNEL_COUNT];
  const uint16_t *pduty_patterns[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;
  uint8_t dac_dma_started = 0U;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;
  pcurrent_patterns[0] = pobj->red_pattern_buffer;
  pcurrent_patterns[1] = pobj->green_pattern_buffer;
  pcurrent_patterns[2] = pobj->blue_pattern_buffer;
  pduty_patterns[0] = pobj->red_duty_buffer;
  pduty_patterns[1] = pobj->green_duty_buffer;
  pduty_patterns[2] = pobj->blue_duty_buffer;

  /* Stop the pacing timer; each channel is re-armed individually below. */
  if (rgbled_stop_transition_timer(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    rgbled_tim_ctx_t *pchannel = pchannels[channel_index];
    hal_tim_oc_compare_unit_t compare_unit;
    hal_tim_oc_compare_unit_config_t oc_compare_unit_config;

    if ((pchannel->phdac == NULL) || (pchannel->phtim == NULL))
    {
      (void)rgbled_stop_dma_patterns(pobj);
      return RGBLED_ERROR;
    }

    /* Return the current (DAC) and duty (TIM OC) paths to the idle state required by the DMA start. */
    if (pchannel->dma_running == 1U)
    {
      if (HAL_OK != HAL_DAC_StopChannel_DMA(pchannel->phdac, pchannel->dac_channel))
      {
        (void)rgbled_stop_dma_patterns(pobj);
        return RGBLED_ERROR;
      }
      if (HAL_OK != HAL_TIM_OC_StopChannel_DMA(pchannel->phtim, pchannel->pwm_channel))
      {
        (void)rgbled_stop_dma_patterns(pobj);
        return RGBLED_ERROR;
      }
      pchannel->dma_running = 0U;
    }
    else
    {
      if (HAL_OK != HAL_DAC_StopChannel(pchannel->phdac, pchannel->dac_channel))
      {
        (void)rgbled_stop_dma_patterns(pobj);
        return RGBLED_ERROR;
      }
    }

    /* Arm the DAC DMA feeding this channel current threshold from its current-code buffer. */
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint8_t *)pcurrent_patterns[channel_index],
                            RGBLED_PATTERN_SIZE * sizeof(pcurrent_patterns[channel_index][0]));
#endif /* __DCACHE_PRESENT */
    if (HAL_OK != HAL_DAC_StartChannel_DMA(pchannel->phdac, pchannel->dac_channel,
                                           (const void *)pcurrent_patterns[channel_index],
                                           RGBLED_PATTERN_SIZE * sizeof(pcurrent_patterns[channel_index][0])))
    {
      (void)rgbled_stop_dma_patterns(pobj);
      return RGBLED_ERROR;
    }

    dac_dma_started = 1U;

    /* Preload the first duty sample, then arm the TIM OC DMA feeding this channel duty cycle. */
    compare_unit = hal_tim_oc_channel_to_compare_unit(pchannel->pwm_channel);
    oc_compare_unit_config.mode = HAL_TIM_OC_PWM1;
    oc_compare_unit_config.pulse = pduty_patterns[channel_index][0];
    if (HAL_OK != HAL_TIM_OC_SetConfigCompareUnit(pchannel->phtim, compare_unit, &oc_compare_unit_config))
    {
      (void)rgbled_stop_dma_patterns(pobj);
      return RGBLED_ERROR;
    }
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint8_t *)pduty_patterns[channel_index],
                            RGBLED_PATTERN_SIZE * sizeof(pduty_patterns[channel_index][0]));
#endif /* __DCACHE_PRESENT */
    if (HAL_OK != HAL_TIM_OC_StartChannel_DMA(pchannel->phtim, pchannel->pwm_channel,
                                              (const uint8_t *)pduty_patterns[channel_index],
                                              RGBLED_PATTERN_SIZE * sizeof(pduty_patterns[channel_index][0])))
    {
      if (dac_dma_started != 0U)
      {
        (void)HAL_DAC_StopChannel_DMA(pchannel->phdac, pchannel->dac_channel);
      }

      (void)rgbled_stop_dma_patterns(pobj);
      (void)rgbled_disable_outputs(pobj);
      return RGBLED_ERROR;
    }

    pchannel->dma_running = 1U;
  }

  /* Start the pacing timer so each sample (DAC current + TIM duty) is consumed at the requested rate. */
  if (rgbled_start_transition_timer(pobj, period_ms) != RGBLED_OK)
  {
    (void)rgbled_stop_dma_patterns(pobj);
    return RGBLED_ERROR;
  }

  return RGBLED_OK;
}
#endif /* USE_HAL_DAC_DMA */

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
  pobj->red.dma_running = 0U;
  pobj->green.dma_running = 0U;
  pobj->blue.dma_running = 0U;

  pobj->red.max_duty_cycle = RGBLED_RED_MAX_DUTY;
  pobj->green.max_duty_cycle = RGBLED_GREEN_MAX_DUTY;
  pobj->blue.max_duty_cycle = RGBLED_BLUE_MAX_DUTY;

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

  /* Bring up the comparator/DAC current-mode regulation on top of the PWM carriers. */
  if (rgbled_start_advanced_control(pobj) != RGBLED_OK)
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

  /* Stop the comparator/DAC current-mode regulation before disabling the outputs. */
  if (rgbled_stop_advanced_control(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

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
  * @brief Apply one static RGB color.
  * @param pobj pointer to the RGB LED object.
  * @param pcolor pointer to the requested color.
  * @retval error status.
  */
rgbled_status_t rgbled_set_color(rgbled_obj_t *pobj, const rgbled_color_t *pcolor)
{
  /* Validate the instance and the requested RGB color. */
  if ((pobj == NULL) || (pobj->state != 1U) || (pcolor == NULL))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Current-mode regulation actuates the per-channel current threshold instead of the PWM duty. */
  if (rgbled_set_color_advanced(pobj, pcolor) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

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
  * @brief Fade from the current color to a target color, repeating until stopped.
  * @param pobj pointer to the RGB LED object.
  * @param ptarget_color pointer to the requested target color.
  * @param period_ms duration of one fade ramp in milliseconds.
  * @retval error status.
  * @note The fade repeats continuously: it ramps the per-channel current threshold and PWM duty
  *       cycle from the current color to the target over period_ms, then restarts from the current
  *       color. A subsequent rgbled_set_color, rgbled_fade_to_color, rgbled_play_pattern or
  *       rgbled_deinit stops the fade.
  */
rgbled_status_t rgbled_fade_to_color(rgbled_obj_t *pobj, const rgbled_color_t *ptarget_color, uint16_t period_ms)
{
  /* Validate the instance and the requested target color. */
  if ((pobj == NULL) || (pobj->state != 1U) || (ptarget_color == NULL)
      || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0U))
  {
    return RGBLED_INVALID_PARAM;
  }

  rgbled_color_t clamped_target;

  /* Clamp the target percentages that the getter will echo back and that anchor the ramp. */
  clamped_target.red_color_value = (ptarget_color->red_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                   ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->red_color_value;
  clamped_target.green_color_value = (ptarget_color->green_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                     ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->green_color_value;
  clamped_target.blue_color_value = (ptarget_color->blue_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                    ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->blue_color_value;

  /* Interpolate from the current color to the target in brightness-percentage space, filling both the
     current-threshold and duty-cycle ramps per channel so both stream in step during the fade. */
  rgbled_fill_channel_transition_pattern(pobj->red_pattern_buffer, pobj->red_duty_buffer,
                                         pobj->color.red_color_value, clamped_target.red_color_value,
                                         &pobj->red);
  rgbled_fill_channel_transition_pattern(pobj->green_pattern_buffer, pobj->green_duty_buffer,
                                         pobj->color.green_color_value, clamped_target.green_color_value,
                                         &pobj->green);
  rgbled_fill_channel_transition_pattern(pobj->blue_pattern_buffer, pobj->blue_duty_buffer,
                                         pobj->color.blue_color_value, clamped_target.blue_color_value,
                                         &pobj->blue);

  /* Launch the synchronized current + duty DMA playback from the per-channel buffers. */
  if (rgbled_start_dma_pattern(pobj, period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Cache the final clamped percentages so rgbled_get_color reports the effective color. */
  pobj->color = clamped_target;

  return RGBLED_OK;
}

/**
  * @brief Play one full RGB pattern, repeating until stopped.
  * @param pobj pointer to the RGB LED object.
  * @param ppattern pointer to the input pattern laid out as red, green, then blue blocks.
  * @param period_ms duration of one pass through the pattern in milliseconds.
  * @retval error status.
  * @note The pattern loops continuously: the current + duty DMA run circularly, replaying the
  *       per-channel current-threshold and duty-cycle sequence until the next rgbled_set_color,
  *       rgbled_fade_to_color, rgbled_play_pattern or rgbled_deinit stops it.
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

  /* Convert the pattern into the private current + duty DMA workspaces. */
  if (rgbled_prepare_limited_pattern(pobj, ppattern) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Launch the synchronized current + duty DMA playback from the per-channel buffers. */
  if (rgbled_start_dma_pattern(pobj, period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Report the last requested percentages (clamped) through rgbled_get_color. */
  final_color.red_color_value = ppattern[RGBLED_PATTERN_SIZE - 1UL];
  final_color.green_color_value = ppattern[(2UL * RGBLED_PATTERN_SIZE) - 1UL];
  final_color.blue_color_value = ppattern[(3UL * RGBLED_PATTERN_SIZE) - 1UL];
  pobj->color.red_color_value = (uint16_t)((final_color.red_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                           ? RGBLED_AUTORELOAD_PERCENTAGE : final_color.red_color_value);
  pobj->color.green_color_value = (uint16_t)((final_color.green_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                             ? RGBLED_AUTORELOAD_PERCENTAGE : final_color.green_color_value);
  pobj->color.blue_color_value = (uint16_t)((final_color.blue_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                            ? RGBLED_AUTORELOAD_PERCENTAGE : final_color.blue_color_value);

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
