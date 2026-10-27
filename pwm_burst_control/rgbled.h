/**
  ******************************************************************************
  * @file    rgbled.h
  * @brief   Public interface of the RGB LED part driver (burst regulation mode).
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RGBLED_H
#define RGBLED_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

/* Exported macros -----------------------------------------------------------*/
#define RGBLED_CHANNEL_COUNT               3U    /*!< Number of RGB channels. */
#define RGBLED_PATTERN_SIZE                128UL /*!< Number of samples per channel in a DMA pattern. */
/* Active carrier-compare DMA buffer length per channel. The burst refill ring streams a double
   buffer of two windows so the CPU can refill the idle window from the half/transfer complete
   interrupt while the DMA streams the other. */
#define RGBLED_CARRIER_BUFFER_SIZE         (2UL * RGBLED_PATTERN_SIZE) /*!< Two-window burst double buffer. */
#define RGBLED_AUTORELOAD_PERCENTAGE       100U  /*!< User-visible full-scale brightness percentage. */
#define RGBLED_PWM_FREQUENCY_TARGET        250000UL /*!< Target PWM carrier frequency in hertz. */
#define RGBLED_PWM_FREQUENCY_TOLERANCE     5000UL /*!< Accepted deviation around the target PWM carrier in hertz. */
#define RGBLED_MIN_TIM_CLK_FREQUENCY       25000000UL /*!< Minimum TIM kernel clock frequency in hertz. */
#define RGBLED_TIM_PERIOD_TICK_LIMIT       0x10000UL /*!< Maximum 16-bit timer period in input clock ticks
                                                          equal to the maximum ARR value plus one. */

/* Current-mode control parameters (DAC + comparator). */
#define RGBLED_LED_CURRENT_MAX_MA          100U  /*!< Hard safety cap of the per-channel LED forward current (mA). */
#define RGBLED_SENSE_RESISTOR_MILLIOHM     3010U /*!< Current-sense shunt value (3.01 ohm expressed in milliohm). */
#define RGBLED_DAC_VREF_MV                 3300U /*!< DAC reference voltage in millivolt. */
#define RGBLED_DAC_FULL_SCALE_CODE         4095U /*!< DAC full-scale code (12-bit resolution). */
/*!< Sense voltage at the maximum current cap: I(mA) * R(milliohm) / 1000 = 100 * 3010 / 1000 = 301 mV. */
#define RGBLED_SENSE_VOLTAGE_MAX_MV        ((RGBLED_LED_CURRENT_MAX_MA * RGBLED_SENSE_RESISTOR_MILLIOHM) / 1000U)
/*!< DAC code matching the current cap, rounded to nearest: (301 * 4095 + 1650) / 3300 = 374. */
#define RGBLED_DAC_CODE_MAX                (((RGBLED_SENSE_VOLTAGE_MAX_MV * RGBLED_DAC_FULL_SCALE_CODE) \
                                             + (RGBLED_DAC_VREF_MV / 2U)) / RGBLED_DAC_VREF_MV)

/* Burst dimming parameters.
   Brightness is set by the fraction of carrier periods gated on inside a fixed burst window while
   the DAC current threshold is pinned at the 100 mA cap, so every gated-on period is regulated at
   constant peak current. One window spans RGBLED_PATTERN_SIZE carrier slots streamed circularly to
   the carrier compare register and paced by the playback timer. */
#define RGBLED_BURST_WINDOW_FREQUENCY      1000UL /*!< Burst-window repetition rate in hertz (>= ~1 kHz is flicker-free). */
/*!< Playback-timer period (ms) that consumes one full burst window of RGBLED_PATTERN_SIZE slots. */
#define RGBLED_BURST_WINDOW_PERIOD_MS      (1000UL / RGBLED_BURST_WINDOW_FREQUENCY)

/* Exported types ------------------------------------------------------------*/
/**
  * @brief Enumeration defining the part status.
  */
typedef enum
{
  RGBLED_OK = 0,          /*!< Operation completed successfully. */
  RGBLED_ERROR = -1,      /*!< Operation failed. */
  RGBLED_INVALID_PARAM = -2 /*!< One or more input parameters are invalid. */
} rgbled_status_t;

/**
  * @brief RGB color values expressed as brightness samples.
  */
typedef struct
{
  uint8_t red_color_value;   /*!< Red channel brightness percentage. */
  uint8_t green_color_value; /*!< Green channel brightness percentage. */
  uint8_t blue_color_value;  /*!< Blue channel brightness percentage. */
} rgbled_color_t;

/**
  * @brief TIM resources used to pace DMA-driven color transitions.
  */
typedef struct
{
  hal_tim_handle_t *phtim;    /*!< TIM handle used to generate DMA pacing triggers. */
  uint8_t running;            /*!< Boolean: pacing timer has been started by this driver. */
} rgbled_pattern_timer_t;

/**
  * @brief TIM context used by one carrier output.
  */
typedef struct
{
  hal_tim_handle_t *phtim;       /*!< TIM handle bound to this context. */
  hal_tim_channel_t pwm_channel; /*!< TIM channel used by this context. */
  uint16_t autoreload;           /*!< Timer auto-reload value for this context. */
  uint8_t dma_running;           /*!< Boolean: DMA operation ongoing on this context. */
  hal_comp_handle_t *phcomp;     /*!< Comparator sensing this channel forward current. */
  hal_dac_handle_t *phdac;       /*!< DAC providing this channel current threshold. */
  hal_dac_channel_t dac_channel; /*!< DAC channel feeding the comparator inverting input. */
} rgbled_tim_ctx_t;

/**
  * @brief RGB LED object.
  */
typedef struct
{
  uint32_t id;                   /*!< Configuration ID. */
  rgbled_tim_ctx_t red;          /*!< Red channel TIM context. */
  rgbled_tim_ctx_t green;        /*!< Green channel TIM context. */
  rgbled_tim_ctx_t blue;         /*!< Blue channel TIM context. */
  rgbled_color_t color;          /*!< Current effective RGB color percentages exposed by the public API. */
  uint8_t burst_running;         /*!< Boolean: carrier-compare burst DMA active for current-mode dimming. */
  rgbled_pattern_timer_t pattern_timer; /*!< TIM context used to pace DMA buffer consumption. */
  /* Carrier-compare DMA buffers, RGBLED_CARRIER_BUFFER_SIZE samples each. The carrier DMA streams
     both windows circularly while the CPU refills the idle half (one window) from the half/transfer
     complete interrupt, animating burst density at constant peak current. */
  uint16_t red_pattern_buffer[RGBLED_CARRIER_BUFFER_SIZE];   /*!< Active red carrier-compare DMA buffer. */
  uint16_t green_pattern_buffer[RGBLED_CARRIER_BUFFER_SIZE]; /*!< Active green carrier-compare DMA buffer. */
  uint16_t blue_pattern_buffer[RGBLED_CARRIER_BUFFER_SIZE];  /*!< Active blue carrier-compare DMA buffer. */
  /* Per-channel burst-density keyframes (0..RGBLED_PATTERN_SIZE gated-on slots) for a running fade
     or pattern. The refill interrupt expands the keyframe selected for the current window into the
     idle half of the carrier buffer above. */
  uint8_t red_anim_density[RGBLED_PATTERN_SIZE];   /*!< Red burst-density keyframes. */
  uint8_t green_anim_density[RGBLED_PATTERN_SIZE]; /*!< Green burst-density keyframes. */
  uint8_t blue_anim_density[RGBLED_PATTERN_SIZE];  /*!< Blue burst-density keyframes. */
  volatile uint32_t anim_window;        /*!< Index of the next burst window the refill interrupt will render. */
  volatile uint32_t anim_total_windows; /*!< Total burst windows in the running fade/pattern (one per millisecond). */
  volatile uint8_t anim_active;         /*!< Boolean: a fade/pattern burst refill ring is currently animating. */
  uint8_t state;                 /*!< Driver state: 0=uninitialized, 1=initialized. */
} rgbled_obj_t;

/* Exported functions --------------------------------------------------------*/
int32_t rgbled_io_init(rgbled_obj_t *pobj);
rgbled_status_t rgbled_init(rgbled_obj_t *pobj, uint32_t id);
rgbled_status_t rgbled_deinit(rgbled_obj_t *pobj);
rgbled_status_t rgbled_set_color(rgbled_obj_t *pobj, const rgbled_color_t *pcolor);
rgbled_status_t rgbled_get_color(rgbled_obj_t *pobj, rgbled_color_t *pcolor);

rgbled_status_t rgbled_fade_to_color(rgbled_obj_t *pobj, const rgbled_color_t *ptarget_color, uint16_t period_ms);
rgbled_status_t rgbled_play_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern, uint16_t period_ms);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* RGBLED_H */
