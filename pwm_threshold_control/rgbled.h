/**
  ******************************************************************************
  * @file    rgbled.h
  * @brief   Public interface of the RGB LED part driver.
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

/* Animated transitions (fade and pattern playback) stream per-channel DAC current-threshold codes
   over a DMA-paced buffer. The board layer binds the pacing timer (pobj->pattern_timer), when it is
   left unbound the playback APIs return RGBLED_INVALID_PARAM. */
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1) \
 && defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
#define RGBLED_PATTERN_PLAYBACK 1
#else
#define RGBLED_PATTERN_PLAYBACK 0
#endif /* RGBLED_PATTERN_PLAYBACK */
/* Exported macros -----------------------------------------------------------*/
#define RGBLED_CHANNEL_COUNT               3U    /*!< Number of RGB channels. */
#define RGBLED_PATTERN_SIZE                128UL /*!< Number of samples per channel in a DMA pattern. */
#define RGBLED_AUTORELOAD_PERCENTAGE       100U  /*!< User-visible full-scale brightness percentage. */
#define RGBLED_PWM_FREQUENCY_TARGET        250000UL /*!< Target PWM carrier frequency in hertz. */
#define RGBLED_PWM_FREQUENCY_TOLERANCE     5000UL /*!< Accepted deviation around the target PWM carrier in hertz. */
#define RGBLED_MIN_TIM_CLK_FREQUENCY       25000000UL /*!< Minimum TIM input clock frequency in hertz. */
#define RGBLED_TIM_PERIOD_TICK_LIMIT       0x10000UL /*!< Maximum 16-bit timer period in input clock ticks
                                                          equal to the maximum ARR value plus one. */
#define RGBLED_MAX_DUTY                    100U /*!< Maximum PWM duty-mode brightness percentage for one color channel. */
#define RGBLED_RED_MAX_DUTY                50U  /*!< Maximum allowed duty cycle for the red channel, in percent. */
#define RGBLED_GREEN_MAX_DUTY              70U  /*!< Maximum allowed duty cycle for the green channel, in percent. */
#define RGBLED_BLUE_MAX_DUTY               70U  /*!< Maximum allowed duty cycle for the blue channel, in percent. */

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
/*!< Current floor of the regulation window (mA). Below roughly this level the current-mode loop is
     unreliable and the LED color/brightness is not trustworthy, so any nonzero brightness request
     is mapped to at least this current. Brightness lower than this level is produced by reducing
     the PWM duty cycle, not the current, only a 0 % request is fully off. */
#define RGBLED_LED_CURRENT_FLOOR_MA        50U
/*!< Sense voltage at the current floor: 50 * 3010 / 1000 = 150 mV. */
#define RGBLED_SENSE_VOLTAGE_FLOOR_MV      ((RGBLED_LED_CURRENT_FLOOR_MA * RGBLED_SENSE_RESISTOR_MILLIOHM) / 1000U)
/*!< DAC code matching the current floor, rounded to nearest: (150 * 4095 + 1650) / 3300 = 186. */
#define RGBLED_DAC_CODE_FLOOR              (((RGBLED_SENSE_VOLTAGE_FLOOR_MV * RGBLED_DAC_FULL_SCALE_CODE) \
                                             + (RGBLED_DAC_VREF_MV / 2U)) / RGBLED_DAC_VREF_MV)

/* When enabled, each requested percentage is remapped through a power-law curve
   (round(1000 * (percent / 100) ^ 0.87), see rgbled_gamma_permille) before it becomes a PWM
   duty-cycle compare value. The sub-unity exponent lifts the dim end so low percentages stay
   visible and evenly spaced given the coarse PWM duty resolution. It shapes the duty cycle only, it
   never changes the DAC current threshold (which stays a linear map onto the 50..100 mA window).
   Define this to 0 to keep a linear percentage-to-duty mapping. */
#ifndef RGBLED_GAMMA_CORRECTION
#define RGBLED_GAMMA_CORRECTION            1
#endif /* RGBLED_GAMMA_CORRECTION */

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
  uint16_t red_color_value;   /*!< Red channel value. */
  uint16_t green_color_value; /*!< Green channel value. */
  uint16_t blue_color_value;  /*!< Blue channel value. */
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
  uint8_t max_duty_cycle;        /*!< Maximum duty cycle. */
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
#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
  rgbled_pattern_timer_t pattern_timer; /*!< TIM context used to pace DMA buffer consumption. */
  uint16_t red_pattern_buffer[RGBLED_PATTERN_SIZE];   /*!< Active red DAC current-threshold DMA buffer. */
  uint16_t green_pattern_buffer[RGBLED_PATTERN_SIZE]; /*!< Active green DAC current-threshold DMA buffer. */
  uint16_t blue_pattern_buffer[RGBLED_PATTERN_SIZE];  /*!< Active blue DAC current-threshold DMA buffer. */
  uint16_t red_duty_buffer[RGBLED_PATTERN_SIZE];      /*!< Active red PWM duty-compare DMA buffer. */
  uint16_t green_duty_buffer[RGBLED_PATTERN_SIZE];    /*!< Active green PWM duty-compare DMA buffer. */
  uint16_t blue_duty_buffer[RGBLED_PATTERN_SIZE];     /*!< Active blue PWM duty-compare DMA buffer. */
#endif /* RGBLED_PATTERN_PLAYBACK */
  uint8_t state;                 /*!< Driver state: 0=uninitialized, 1=initialized. */
} rgbled_obj_t;

/* Exported functions --------------------------------------------------------*/
int32_t rgbled_io_init(rgbled_obj_t *pobj);
rgbled_status_t rgbled_init(rgbled_obj_t *pobj, uint32_t id);
rgbled_status_t rgbled_deinit(rgbled_obj_t *pobj);
rgbled_status_t rgbled_set_color(rgbled_obj_t *pobj, const rgbled_color_t *pcolor);
rgbled_status_t rgbled_get_color(rgbled_obj_t *pobj, rgbled_color_t *pcolor);

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
rgbled_status_t rgbled_fade_to_color(rgbled_obj_t *pobj, const rgbled_color_t *ptarget_color, uint16_t period_ms);
rgbled_status_t rgbled_play_pattern(rgbled_obj_t *pobj, const uint16_t *ppattern, uint16_t period_ms);
#endif /* RGBLED_PATTERN_PLAYBACK */

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* RGBLED_H */
