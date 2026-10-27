/**
  ******************************************************************************
  * @file    rgbled.c
  * @brief   Implementation of the RGB LED part driver (burst regulation mode).
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

/* Private functions prototype -----------------------------------------------*/
static rgbled_status_t rgbled_apply_channel_output(rgbled_tim_ctx_t *pchannel, uint16_t pulse_value);
static rgbled_status_t rgbled_disable_outputs(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_unique_timers(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_unique_timers(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_transition_timer(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_transition_timer(rgbled_obj_t *pobj, uint32_t period_ms);
static rgbled_status_t rgbled_start_pacing_timer(rgbled_obj_t *pobj, uint32_t step_frequency_hz);
static rgbled_status_t rgbled_stop_channel_dma_pattern(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_stop_dma_patterns(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_start_dma_pattern(rgbled_obj_t *pobj, uint32_t period_ms);
static uint8_t rgbled_percent_to_burst_density(uint16_t percent_value);
static rgbled_status_t rgbled_setup_advanced_channel(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_start_advanced_control(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_stop_advanced_channel(rgbled_tim_ctx_t *pchannel);
static rgbled_status_t rgbled_stop_advanced_control(rgbled_obj_t *pobj);
static rgbled_status_t rgbled_set_color_advanced(rgbled_obj_t *pobj, const rgbled_color_t *pcolor);
static void rgbled_expand_density(uint16_t *phalf, uint16_t on_value, uint8_t density);
static uint32_t rgbled_anim_window_index(uint32_t window, uint32_t total_windows);
static void rgbled_build_fade_keyframes(uint8_t *pkeyframes, uint8_t start_density, uint8_t end_density);
static void rgbled_burst_window_isr(rgbled_obj_t *pobj, uint32_t half);
static rgbled_status_t rgbled_start_burst_anim(rgbled_obj_t *pobj, uint32_t total_windows);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Apply one carrier duty value to one channel.
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

/**
  * @brief Configure and start the timer that paces a burst window.
  * @param pobj pointer to the RGB LED object.
  * @param period_ms total playback duration in milliseconds.
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

/**
  * @brief Stop the carrier-compare DMA transfer on one RGB channel if it is active.
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
  pobj->anim_active = 0U;
  pobj->anim_window = 0U;
  pobj->anim_total_windows = 0U;

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

  pobj->burst_running = 0U;

  return RGBLED_OK;
}

/**
  * @brief Map a brightness percentage to a count of gated-on carrier slots in one burst window.
  * @param percent_value requested brightness as a percentage of full scale.
  * @retval gated-on slot count in the range 0..RGBLED_PATTERN_SIZE (the burst density).
  * @note Scales the percentage to a light fraction, maps it to the window slot count rounded to
  *       nearest, then applies a minimum-on floor so a nonzero request lights at least one slot
  *       (low percentages that round toward zero stay visible). A full-scale request gates every
  *       slot on, a zero request gates every slot off.
  */
static uint8_t rgbled_percent_to_burst_density(uint16_t percent_value)
{
  uint16_t limited_percent;
  uint16_t brightness_permille;
  uint32_t on_slots;

  /* Clamp to full scale and scale linearly to a 0..1000 light fraction. */
  limited_percent = (percent_value > RGBLED_AUTORELOAD_PERCENTAGE)
                    ? RGBLED_AUTORELOAD_PERCENTAGE : percent_value;
  brightness_permille = (uint16_t)((limited_percent * 1000UL) / RGBLED_AUTORELOAD_PERCENTAGE);

  on_slots = ((brightness_permille * RGBLED_PATTERN_SIZE) + 500UL) / 1000UL;

  if ((percent_value > 0U) && (on_slots == 0U))
  {
    on_slots = 1U;
  }

  return (uint8_t)on_slots;
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

  /* Start the DAC and pin the current threshold at the 100 mA cap so every gated-on carrier period
     regulates at constant peak current and brightness is set by burst density (the carrier is
     gated, never the threshold). The output stays forced inactive below, so the channel is dark
     until a color is applied even with the threshold pinned high. */
  if (HAL_OK != HAL_DAC_StartChannel(pchannel->phdac, pchannel->dac_channel))
  {
    return RGBLED_ERROR;
  }
  if (HAL_OK != HAL_DAC_SetChannelData(pchannel->phdac, pchannel->dac_channel, RGBLED_DAC_CODE_MAX))
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
  /* A burst-dimmed static color drives the carrier compare register through DMA. Stop those three
     transfers and the pacing timer, then force the outputs inactive, before the comparators are
     stopped below, so no channel is left driving a full-duty carrier without current regulation. */
  if (pobj->burst_running == 1U)
  {
    if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    if (rgbled_disable_outputs(pobj) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
    pobj->burst_running = 0U;
  }
  else
  {
    /* No burst is active, ensure the pacing timer is stopped. */
    if (rgbled_stop_transition_timer(pobj) != RGBLED_OK)
    {
      return RGBLED_ERROR;
    }
  }

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
  * @brief Apply one static RGB color as per-channel carrier burst density.
  * @param pobj pointer to the RGB LED object.
  * @param pcolor pointer to the requested color (percentage of full brightness).
  * @retval error status.
  */
static rgbled_status_t rgbled_set_color_advanced(rgbled_obj_t *pobj, const rgbled_color_t *pcolor)
{
  rgbled_color_t requested_color;
  rgbled_status_t ret = RGBLED_OK;

  /* Clamp each channel to full scale before mapping it to a burst density. */
  requested_color.red_color_value = (pcolor->red_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                    ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->red_color_value;
  requested_color.green_color_value = (pcolor->green_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                      ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->green_color_value;
  requested_color.blue_color_value = (pcolor->blue_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                     ? RGBLED_AUTORELOAD_PERCENTAGE : pcolor->blue_color_value;

  /* Stop any running burst animation; this also clears the animation state and burst_running. */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Pin every channel current threshold at the 100 mA cap so each gated-on carrier period
    regulates at constant peak current. Brightness is then controlled by burst density. */
  if (HAL_OK != HAL_DAC_SetChannelData(pobj->red.phdac, pobj->red.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else if (HAL_OK != HAL_DAC_SetChannelData(pobj->green.phdac, pobj->green.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else if (HAL_OK != HAL_DAC_SetChannelData(pobj->blue.phdac, pobj->blue.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else
  {
    ret = RGBLED_OK;
  }

  if (ret != RGBLED_OK)
  {
    /* The prior burst DMA is already stopped, force the outputs inactive so a failed
       reconfiguration leaves the channels dark instead of statically driven. */
    (void)rgbled_disable_outputs(pobj);
    return RGBLED_ERROR;
  }

  /* Precompute one full burst window per channel (a single contiguous on-burst sized by brightness
     followed by an off-burst). Each slot is a carrier compare sample: the channel auto-reload gates
     the period fully on, zero gates it off. */
  rgbled_expand_density(pobj->red_pattern_buffer, pobj->red.autoreload,
                        rgbled_percent_to_burst_density(requested_color.red_color_value));
  rgbled_expand_density(pobj->green_pattern_buffer, pobj->green.autoreload,
                        rgbled_percent_to_burst_density(requested_color.green_color_value));
  rgbled_expand_density(pobj->blue_pattern_buffer, pobj->blue.autoreload,
                        rgbled_percent_to_burst_density(requested_color.blue_color_value));

  /* Stream the windows circularly to the carrier compare registers. The GPDMA linked list loops the
     burst forever in hardware with no interrupt, the playback timer paces one full window per burst
     period, so a static color needs no carrier compare-match callback and no DMA restart. */
  if (rgbled_start_dma_pattern(pobj, RGBLED_BURST_WINDOW_PERIOD_MS) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }
  pobj->burst_running = 1U;

  /* Cache the clamped percentages so rgbled_get_color reports the effective color. */
  pobj->color = requested_color;
  return RGBLED_OK;
}

/**
  * @brief Start the three channel DMA transfers from the active per-channel burst windows.
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

  /* Stop any ongoing transfer before arming the new RGB window. */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    const hal_tim_oc_compare_unit_t compare_unit =
      hal_tim_oc_channel_to_compare_unit(pchannels[channel_index]->pwm_channel);
    hal_tim_oc_compare_unit_config_t oc_compare_unit_config;

    /* Resolve the channel slice and preload the first carrier sample before DMA start. */
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
        if (HAL_OK != HAL_TIM_OC_StartChannel_DMA_Opt(pchannels[channel_index]->phtim,
                                                      pchannels[channel_index]->pwm_channel,
                                                      (const uint8_t *)ppatterns[channel_index],
                                                      RGBLED_PATTERN_SIZE * sizeof(ppatterns[channel_index][0]),
                                                      HAL_TIM_OPT_DMA_IT_SILENT))
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

/* Animated burst dimming (fade and pattern playback).
   A fade/pattern is a stream of burst windows, each window's gated-on slot count (density) sets the
   brightness while the DAC threshold stays pinned at the cap (constant peak current). The three
   carrier compare DMAs run circularly over a two-window double buffer and share the pacing trigger,
   so they advance in lock-step. Only the red carrier requests the half/transfer-complete interrupt
   (green/blue are armed silent), red's interrupt alone refills the just-freed half with the next
   window for all three channels. The animation advances one window per RGBLED_BURST_WINDOW_PERIOD_MS
   and never stops or re-arms the running DMA, at the end it wraps back to the first keyframe, so the
   same refill interrupt replays the fade/pattern indefinitely.

   The carrier compare-match callbacks recover their owning object from the red carrier timer-handle
   user data (set in rgbled_init), so several independent RGB LED objects can coexist. */

/**
  * @brief Expand one burst-density value into one carrier-buffer half (one window).
  * @param phalf pointer to the first slot of the window to fill (RGBLED_PATTERN_SIZE slots).
  * @param on_value compare value gating a slot fully on (the channel full-duty auto-reload).
  * @param density number of leading slots to gate on (0..RGBLED_PATTERN_SIZE).
  */
static void rgbled_expand_density(uint16_t *phalf, uint16_t on_value, uint8_t density)
{
  uint32_t index;

  /* A contiguous on-burst of `density` slots at full duty, then off for the rest of the window. */
  for (index = 0U; index < RGBLED_PATTERN_SIZE; index++)
  {
    phalf[index] = (index < density) ? on_value : 0U;
  }
}

/**
  * @brief Map a window position in a running animation to its keyframe index.
  * @param window zero-based index of the window being rendered.
  * @param total_windows total number of windows in the animation (>= 1).
  * @retval keyframe index in the range 0..RGBLED_PATTERN_SIZE-1.
  * @note The first window maps to keyframe 0 and the last window to the final keyframe, so the
  *       animation reaches its end value exactly, positions past the end hold the final keyframe.
  */
static uint32_t rgbled_anim_window_index(uint32_t window, uint32_t total_windows)
{
  uint32_t clamped_window;
  uint32_t index;

  /* Hold the final keyframe once the animation has run its course. */
  clamped_window = (window >= total_windows) ? (total_windows - 1U) : window;

  if (total_windows <= 1U)
  {
    index = RGBLED_PATTERN_SIZE - 1U;
  }
  else
  {
    index = (clamped_window * (RGBLED_PATTERN_SIZE - 1U)) / (total_windows - 1U);
    if (index >= RGBLED_PATTERN_SIZE)
    {
      index = RGBLED_PATTERN_SIZE - 1U;
    }
  }

  return index;
}

/**
  * @brief Build one channel keyframe table as a linear ramp of burst density.
  * @param pkeyframes pointer to the channel keyframe table (RGBLED_PATTERN_SIZE entries).
  * @param start_density burst density displayed at the first keyframe.
  * @param end_density burst density reached at the last keyframe.
  */
static void rgbled_build_fade_keyframes(uint8_t *pkeyframes, uint8_t start_density, uint8_t end_density)
{
  int32_t delta;
  uint32_t index;

  /* Linear ramp of burst density from the current level to the target across the keyframes. */
  delta = (int32_t)end_density - (int32_t)start_density;
  for (index = 0U; index < RGBLED_PATTERN_SIZE; index++)
  {
    const int32_t value = (int32_t)start_density
                          + ((delta * (int32_t)index) / (int32_t)(RGBLED_PATTERN_SIZE - 1UL));
    pkeyframes[index] = (uint8_t)value;
  }
}

/**
  * @brief Refill one freed carrier-buffer half with the next animation window.
  * @param pobj pointer to the RGB LED object.
  * @param half index of the freed half (0 = first window, 1 = second window).
  * @note Runs in the red carrier DMA half/transfer-complete interrupt. It only writes the idle half,
  *       never the half the DMA is currently streaming, so the running transfer is never disturbed.
  */
static void rgbled_burst_window_isr(rgbled_obj_t *pobj, uint32_t half)
{
  uint32_t base;
  uint32_t window;
  uint32_t index;

  /* Nothing to refill for a static burst color (no animation active). */
  if (pobj->anim_active == 0U)
  {
    return;
  }

  if (pobj->anim_total_windows == 0U)
  {
    return;
  }

  /* Render the window the just-freed half will display next, for all three channels (their carrier
     DMAs share the pacing trigger, so the freed half spans the same slot range on each). */
  window = pobj->anim_window;
  index = rgbled_anim_window_index(window, pobj->anim_total_windows);
  base = half * RGBLED_PATTERN_SIZE;
  rgbled_expand_density(&pobj->red_pattern_buffer[base], pobj->red.autoreload, pobj->red_anim_density[index]);
  rgbled_expand_density(&pobj->green_pattern_buffer[base], pobj->green.autoreload, pobj->green_anim_density[index]);
  rgbled_expand_density(&pobj->blue_pattern_buffer[base], pobj->blue.autoreload, pobj->blue_anim_density[index]);

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  /* Push the just-refilled half out of the data cache so the carrier DMA streams the new window
     instead of stale cache lines when the transfer wraps to this half. */
  SCB_CleanDCache_by_Addr((uint8_t *)&pobj->red_pattern_buffer[base],
                          RGBLED_PATTERN_SIZE * sizeof(pobj->red_pattern_buffer[0]));
  SCB_CleanDCache_by_Addr((uint8_t *)&pobj->green_pattern_buffer[base],
                          RGBLED_PATTERN_SIZE * sizeof(pobj->green_pattern_buffer[0]));
  SCB_CleanDCache_by_Addr((uint8_t *)&pobj->blue_pattern_buffer[base],
                          RGBLED_PATTERN_SIZE * sizeof(pobj->blue_pattern_buffer[0]));
#endif /* __DCACHE_PRESENT */

  /* Advance to the next window, wrapping back to the first keyframe at the end so the fade/pattern
     replays indefinitely. The carrier DMA never stops or re-arms; this refill interrupt alone
     re-feeds the keyframe walk. */
  pobj->anim_window = window + 1U;
  if ((window + 1U) >= pobj->anim_total_windows)
  {
    pobj->anim_window = 0U;
  }
}

/**
  * @brief Start the burst refill ring for a fade or pattern from the per-channel keyframe tables.
  * @param pobj pointer to the RGB LED object.
  * @param total_windows number of burst windows in one pass of the animation.
  * @retval error status.
  * @note The caller fills the three *_anim_density keyframe tables before calling this function.
  *       One window lasts RGBLED_BURST_WINDOW_PERIOD_MS, so total_windows equals the animation
  *       duration in milliseconds. The animation replays indefinitely until the next color/animation
  *       command or rgbled_deinit stops it.
  */
static rgbled_status_t rgbled_start_burst_anim(rgbled_obj_t *pobj, uint32_t total_windows)
{
  rgbled_status_t ret = RGBLED_OK;
  rgbled_tim_ctx_t *pchannels[RGBLED_CHANNEL_COUNT];
  uint16_t *pburst_pattern[RGBLED_CHANNEL_COUNT];
  const uint8_t *pdensity[RGBLED_CHANNEL_COUNT];
  uint32_t channel_index;
  uint32_t index0;
  uint32_t index1;

  pchannels[0] = &pobj->red;
  pchannels[1] = &pobj->green;
  pchannels[2] = &pobj->blue;
  pburst_pattern[0] = pobj->red_pattern_buffer;
  pburst_pattern[1] = pobj->green_pattern_buffer;
  pburst_pattern[2] = pobj->blue_pattern_buffer;
  pdensity[0] = pobj->red_anim_density;
  pdensity[1] = pobj->green_anim_density;
  pdensity[2] = pobj->blue_anim_density;

  /* Begin from a clean slate: stop any running carrier burst and the pacing timer. */
  if (rgbled_stop_dma_patterns(pobj) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Pin every channel current threshold at the cap so each gated-on carrier period regulates at the
     constant peak current, brightness animates purely as carrier burst density. */
  if (HAL_OK != HAL_DAC_SetChannelData(pobj->red.phdac, pobj->red.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else if (HAL_OK != HAL_DAC_SetChannelData(pobj->green.phdac, pobj->green.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else if (HAL_OK != HAL_DAC_SetChannelData(pobj->blue.phdac, pobj->blue.dac_channel, RGBLED_DAC_CODE_MAX))
  {
    ret = RGBLED_ERROR;
  }

  else
  {
    ret = RGBLED_OK;
  }

  if (ret != RGBLED_OK)
  {
    /* The prior burst DMA is already stopped, force the outputs inactive so a failed
       reconfiguration leaves the channels dark instead of statically driven. */
    (void)rgbled_disable_outputs(pobj);
    return ret;
  }

  /* One window spans RGBLED_PATTERN_SIZE carrier slots paced at the burst-window rate, so each
     window lasts RGBLED_BURST_WINDOW_PERIOD_MS and the window count equals the duration in ms. The
     callers reject a zero period, so total_windows is always at least one. */
  pobj->anim_total_windows = total_windows;

  /* Pre-render the first two windows: the carrier DMA streams window 0 from the first half and
     window 1 from the second half, then the refill interrupt loads window 2 onward. */
  index0 = rgbled_anim_window_index(0U, pobj->anim_total_windows);
  index1 = rgbled_anim_window_index(1U, pobj->anim_total_windows);
  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    rgbled_expand_density(&pburst_pattern[channel_index][0],
                          pchannels[channel_index]->autoreload,
                          pdensity[channel_index][index0]);
    rgbled_expand_density(&pburst_pattern[channel_index][RGBLED_PATTERN_SIZE],
                          pchannels[channel_index]->autoreload,
                          pdensity[channel_index][index1]);
  }
  pobj->anim_window = 2U;
  pobj->anim_active = 1U;

  /* Arm the three carrier compare DMAs over the full double buffer. Only the red carrier requests
     the half/transfer-complete interrupt that drives the refill, the green and blue carriers are
     armed silent (no DMA interrupt). The three transfers share the pacing trigger so they advance in
     lock-step, so the red interrupt alone marks when every channel's just-freed half can be refilled. */
  for (channel_index = 0U; channel_index < RGBLED_CHANNEL_COUNT; channel_index++)
  {
    const hal_tim_oc_compare_unit_t compare_unit =
      hal_tim_oc_channel_to_compare_unit(pchannels[channel_index]->pwm_channel);
    hal_tim_oc_compare_unit_config_t oc_compare_unit_config;
    /* Red (channel 0) carries the refill interrupt, green and blue are armed silent so they raise
       no DMA interrupt and advance in lock-step under the shared pacing trigger. */
    const uint32_t dma_it_opt =
      (channel_index == 0U) ? (uint32_t)HAL_TIM_OPT_DMA_IT_HT : (uint32_t)HAL_TIM_OPT_DMA_IT_SILENT;

    if (pchannels[channel_index]->phtim == NULL)
    {
      ret = RGBLED_ERROR;
    }
    else
    {
      oc_compare_unit_config.mode = HAL_TIM_OC_PWM1;
      oc_compare_unit_config.pulse = pburst_pattern[channel_index][0];

      if (HAL_OK != HAL_TIM_OC_SetConfigCompareUnit(pchannels[channel_index]->phtim,
                                                    compare_unit, &oc_compare_unit_config))
      {
        ret = RGBLED_ERROR;
      }
      else
      {
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        SCB_CleanDCache_by_Addr((uint8_t *)pburst_pattern[channel_index],
                                (2U * RGBLED_PATTERN_SIZE) * sizeof(pburst_pattern[channel_index][0]));
#endif /* __DCACHE_PRESENT */
        /* Start the DMA stream for the current color plane and mark it as active. */
        if (HAL_OK != HAL_TIM_OC_StartChannel_DMA_Opt(pchannels[channel_index]->phtim,
                                                      pchannels[channel_index]->pwm_channel,
                                                      (const uint8_t *)pburst_pattern[channel_index],
                                                      (2U * RGBLED_PATTERN_SIZE)
                                                      * sizeof(pburst_pattern[channel_index][0]),
                                                      dma_it_opt))
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

  /* Start the pacing timer at the burst-window rate so each carrier slot advances once per trigger
     and one full window is consumed every RGBLED_BURST_WINDOW_PERIOD_MS. */
  if (rgbled_start_transition_timer(pobj, RGBLED_BURST_WINDOW_PERIOD_MS) != RGBLED_OK)
  {
    (void)rgbled_stop_dma_patterns(pobj);
    (void)rgbled_disable_outputs(pobj);
    return RGBLED_ERROR;
  }

  pobj->burst_running = 1U;
  return RGBLED_OK;
}

/**
  * @brief Red carrier DMA half-transfer interrupt: refill the first window half.
  * @param htim TIM handle that raised the interrupt.
  * @param channel TIM channel that raised the interrupt.
  */
void HAL_TIM_CompareMatchHalfCpltCallback(hal_tim_handle_t *htim, hal_tim_channel_t channel)
{
  rgbled_obj_t *pobj = (rgbled_obj_t *)HAL_TIM_GetUserData(htim);

  /* The red carrier is the window clock. A half-transfer means the first window was consumed and is
     now free to refill while the DMA streams the second window. */
  if ((pobj != NULL) && (htim == pobj->red.phtim) && (channel == pobj->red.pwm_channel))
  {
    rgbled_burst_window_isr(pobj, 0U);
  }
}

/**
  * @brief Red carrier DMA transfer-complete interrupt: refill the second window half.
  * @param htim TIM handle that raised the interrupt.
  * @param channel TIM channel that raised the interrupt.
  */
void HAL_TIM_CompareMatchCallback(hal_tim_handle_t *htim, hal_tim_channel_t channel)
{
  rgbled_obj_t *pobj = (rgbled_obj_t *)HAL_TIM_GetUserData(htim);

  /* Transfer-complete means the second window was consumed and the transfer wrapped to the first
     window, so the second window is now free to refill. */
  if ((pobj != NULL) && (htim == pobj->red.phtim) && (channel == pobj->red.pwm_channel))
  {
    rgbled_burst_window_isr(pobj, 1U);
  }
}

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
  pobj->pattern_timer.running = 0U;
  pobj->burst_running = 0U;
  /* Start with the animation idle, the carrier compare-match callbacks are routed to this object
     through the red timer handle user data once the timers are brought up below. */
  pobj->anim_active = 0U;
  pobj->anim_window = 0U;
  pobj->anim_total_windows = 0U;

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
  /* Route red carrier compare-match callbacks to this object through timer-handle user data.
    The red timer handle is valid here and burst DMA is armed later by color/animation APIs. */
  HAL_TIM_SetUserData(pobj->red.phtim, pobj);

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

  /* Stop the burst refill ring first so the carrier compare-match callbacks no longer touch this
     object while its DMA streams are being torn down. */
  pobj->anim_active = 0U;
  pobj->anim_window = 0U;
  pobj->anim_total_windows = 0U;
  HAL_TIM_SetUserData(pobj->red.phtim, NULL);

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

  /* Burst regulation actuates the per-channel carrier burst density with the current threshold
     pinned at the cap. */
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

/**
  * @brief Fade from the current color to a target color, repeating until stopped.
  * @param pobj pointer to the RGB LED object.
  * @param ptarget_color pointer to the requested target color.
  * @param period_ms duration of one fade ramp in milliseconds.
  * @retval error status.
  * @note The fade repeats continuously: it ramps from the current color to the target over
  *       period_ms, then restarts from the current color. A subsequent rgbled_set_color,
  *       rgbled_fade_to_color, rgbled_play_pattern or rgbled_deinit stops the fade.
  */
rgbled_status_t rgbled_fade_to_color(rgbled_obj_t *pobj, const rgbled_color_t *ptarget_color, uint16_t period_ms)
{
  rgbled_color_t clamped_target;

  /* Validate the instance and the requested target color. */
  if ((pobj == NULL) || (pobj->state != 1U) || (ptarget_color == NULL)
      || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0U))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Clamp the requested target percentages to full scale. */
  clamped_target.red_color_value = (ptarget_color->red_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                   ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->red_color_value;
  clamped_target.green_color_value = (ptarget_color->green_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                     ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->green_color_value;
  clamped_target.blue_color_value = (ptarget_color->blue_color_value > RGBLED_AUTORELOAD_PERCENTAGE)
                                    ? RGBLED_AUTORELOAD_PERCENTAGE : ptarget_color->blue_color_value;

  /* Build one per-channel burst-density ramp from the present color to the clamped target. The
     DAC current threshold stays pinned at the cap, so brightness fades purely as carrier burst
     density at constant peak current. */
  rgbled_build_fade_keyframes(pobj->red_anim_density,
                              rgbled_percent_to_burst_density(pobj->color.red_color_value),
                              rgbled_percent_to_burst_density(clamped_target.red_color_value));
  rgbled_build_fade_keyframes(pobj->green_anim_density,
                              rgbled_percent_to_burst_density(pobj->color.green_color_value),
                              rgbled_percent_to_burst_density(clamped_target.green_color_value));
  rgbled_build_fade_keyframes(pobj->blue_anim_density,
                              rgbled_percent_to_burst_density(pobj->color.blue_color_value),
                              rgbled_percent_to_burst_density(clamped_target.blue_color_value));

  /* One burst window per millisecond, so the requested duration maps directly to the window count.
     The ramp loops so the fade repeats continuously until the next color/animation command or
     rgbled_deinit stops it. */
  if (rgbled_start_burst_anim(pobj, (uint32_t)period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  pobj->color = clamped_target;
  return RGBLED_OK;
}

/**
  * @brief Play one full RGB pattern, repeating until stopped.
  * @param pobj pointer to the RGB LED object.
  * @param ppattern pointer to the input pattern laid out as red, green, then blue blocks.
  * @param period_ms duration of one pass through the pattern in milliseconds.
  * @retval error status.
  * @note The pattern loops continuously: the carrier DMA runs circularly and the refill interrupt
  *       replays the keyframe sequence with no software replay loop. A subsequent rgbled_set_color,
  *       rgbled_fade_to_color, rgbled_play_pattern or rgbled_deinit stops the loop.
  */
rgbled_status_t rgbled_play_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern, uint16_t period_ms)
{
  uint32_t sample_index;
  uint16_t last_red;
  uint16_t last_green;
  uint16_t last_blue;

  /* Validate the instance and the user pattern before preparing playback. */
  if ((pobj == NULL) || (pobj->state != 1U) || (ppattern == NULL)
      || (pobj->pattern_timer.phtim == NULL) || (period_ms == 0U))
  {
    return RGBLED_INVALID_PARAM;
  }

  /* Convert each pattern plane sample (a brightness percentage) into a burst-density keyframe. The
     DAC current threshold stays pinned at the cap, so the pattern plays as carrier burst density
     at constant peak current. The planes are laid out as red, then green, then blue. */
  for (sample_index = 0U; sample_index < RGBLED_PATTERN_SIZE; sample_index++)
  {
    pobj->red_anim_density[sample_index] =
      rgbled_percent_to_burst_density(ppattern[sample_index]);
    pobj->green_anim_density[sample_index] =
      rgbled_percent_to_burst_density(ppattern[RGBLED_PATTERN_SIZE + sample_index]);
    pobj->blue_anim_density[sample_index] =
      rgbled_percent_to_burst_density(ppattern[(2UL * RGBLED_PATTERN_SIZE) + sample_index]);
  }

  /* One burst window per millisecond, so the requested duration maps directly to the window count.
     The sequence loops indefinitely until the next color/animation command or rgbled_deinit stops it. */
  if (rgbled_start_burst_anim(pobj, (uint32_t)period_ms) != RGBLED_OK)
  {
    return RGBLED_ERROR;
  }

  /* Report the last requested percentages (clamped) through rgbled_get_color. */
  last_red = ppattern[RGBLED_PATTERN_SIZE - 1UL];
  last_green = ppattern[(2UL * RGBLED_PATTERN_SIZE) - 1UL];
  last_blue = ppattern[(3UL * RGBLED_PATTERN_SIZE) - 1UL];
  pobj->color.red_color_value = (uint8_t)((last_red > RGBLED_AUTORELOAD_PERCENTAGE)
                                          ? RGBLED_AUTORELOAD_PERCENTAGE : last_red);
  pobj->color.green_color_value = (uint8_t)((last_green > RGBLED_AUTORELOAD_PERCENTAGE)
                                            ? RGBLED_AUTORELOAD_PERCENTAGE : last_green);
  pobj->color.blue_color_value = (uint8_t)((last_blue > RGBLED_AUTORELOAD_PERCENTAGE)
                                           ? RGBLED_AUTORELOAD_PERCENTAGE : last_blue);
  return RGBLED_OK;
}

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
