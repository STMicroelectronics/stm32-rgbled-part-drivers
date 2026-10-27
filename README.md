# __RGBLED Part Drivers__

![tag](https://img.shields.io/badge/tag-2.0.0-brightgreen.svg)
[![release note](https://img.shields.io/badge/release_note-view_html-gold.svg)](https://htmlpreview.github.io/?https://github.com/STMicroelectronics/stm32-rgbled-part-drivers/blob/main/Release_Notes.html)

## Overview

The RGBLED part driver controls an RGB LED whose red, green, and blue channels are driven by STM32 HAL TIM PWM outputs. It is delivered as an OpenCMSIS pack with three interchangeable driver variants that share the same object model and the same core API but differ in how each channel's brightness and LED current are regulated:

| Variant (pack component) | Source folder | Brightness control | LED current control | HAL dependencies |
| --- | --- | --- | --- | --- |
| **PWM No Control** | `pwm_no_control` | PWM duty cycle | none in the driver — bounded only by a per-channel duty-cycle cap | TIM (+ TIM DMA for animations) |
| **PWM Threshold Control** | `pwm_threshold_control` | gamma-corrected PWM duty cycle | hardware current-mode loop, linear 50–100 mA threshold | TIM + DAC + COMP (+ TIM & DAC DMA for animations) |
| **PWM Burst Control** | `pwm_burst_control` | carrier burst density | hardware current-mode loop, current pinned at the 100 mA cap | TIM + DAC + COMP |

All three variants:

- Expose the same object type `rgbled_obj_t` and the same core functions `rgbled_init()`, `rgbled_deinit()`, `rgbled_set_color()`, and `rgbled_get_color()`.
- Take colors as per-channel brightness percentages in the range 0–100.
- Generate the PWM carrier from HAL TIM at `RGBLED_PWM_FREQUENCY_TARGET` (250 kHz, ±`RGBLED_PWM_FREQUENCY_TOLERANCE`) and require a TIM kernel clock of at least `RGBLED_MIN_TIM_CLK_FREQUENCY` (25 MHz).
- Assume an up-counting (edge-aligned) timer with a repetition counter of 0, center-aligned timers or non-zero repetition counters are not supported.
- Bind board resources through the weak `rgbled_io_init()` hook and force all outputs off until a color or animation is applied.
- Offer DMA-paced animations, `rgbled_fade_to_color()` and `rgbled_play_pattern()`, that stream `RGBLED_PATTERN_SIZE` (128) samples per channel and loop until the next command.

  Only one variant is compiled into an application: pick the source folder (pack component) that matches the board's hardware.

## Description

## Peripherals initialization

The driver assumes that every peripheral it uses is initialized by the application (or by STM32CubeMX2-generated code) before `rgbled_init()` is called. At minimum the TIM instances used for the red, green, and blue PWM outputs must be available. Depending on the variant, additional resources are mandatory — see each variant's *Required configuration* below.

The binding between the driver object and the board resources is done by `rgbled_io_init()`, which is declared weak inside the driver and must be overridden by the application or generated code. The application selects the board configuration through the `.id` field of `rgbled_obj_t`.

All public APIs require a valid `rgbled_obj_t` object. `rgbled_init()` rejects a null object and re-initialization of an already initialized instance. `rgbled_set_color()` and `rgbled_get_color()` require an initialized object, and the animation APIs additionally require a configured `pattern_timer.phtim`, a non-null pattern or target color, and a nonzero transition duration.

## Common resource model

Each RGB channel owns one `hal_tim_handle_t` and one TIM channel selector. Boards may map red, green, and blue to the same TIM instance, the driver starts each distinct RGB PWM timer only once. Animations keep their transition workspace inside each `rgbled_obj_t` instance (per-channel buffers), not in a driver-wide shared buffer, and are paced by a dedicated timer bound through `pattern_timer.phtim`.

The core carrier and animation parameters are defined by the public macros `RGBLED_PWM_FREQUENCY_TARGET`, `RGBLED_PWM_FREQUENCY_TOLERANCE`, `RGBLED_MIN_TIM_CLK_FREQUENCY`, `RGBLED_PATTERN_SIZE`, and `RGBLED_CHANNEL_COUNT`.

## PWM No Control (`pwm_no_control`)

### Principle

Brightness is the PWM duty cycle of each channel: the requested percentage is scaled by a per-channel maximum duty-cycle cap and programmed directly into the timer compare register. A minimum-on floor guarantees that any nonzero request lights at least one timer tick, so a channel is dark only at 0 %. There is no current sensing or regulation in this variant.

### Required configuration

- Three PWM-capable TIM channels (which may share TIM instances).
- For animations only: `USE_HAL_TIM_DMA == 1`, plus a fourth TIM instance bound to `pattern_timer.phtim` to pace the DMA.
- No DAC or comparator is used.

### Safety — user-chosen duty cap

This variant has **no hardware current limiting**. The only bound on the LED current is the per-channel duty-cycle cap (`RGBLED_RED_MAX_DUTY` = 50 %, `RGBLED_GREEN_MAX_DUTY` = 70 %, `RGBLED_BLUE_MAX_DUTY` = 70 % by default), which the generated IO layer sets to match the board's buck-converter current limit. The application can override these caps at runtime with `rgbled_set_max_duty_cycle()`.

Raising a cap lets the channel draw more current: **it is the user's responsibility to keep the caps within the board's safe operating area.** Lowering the caps is always safe. Because nothing in hardware enforces the limit, this variant must only be used on boards whose LED path is otherwise current-limited across the duty range the application allows.

### API

`rgbled_init`, `rgbled_deinit`, `rgbled_set_color`, `rgbled_get_color`, `rgbled_set_max_duty_cycle`, and (when TIM DMA is enabled) `rgbled_fade_to_color`, `rgbled_play_pattern`.

## PWM Threshold Control (`pwm_threshold_control`)

### Principle

Each channel runs a hardware current-mode loop. A DAC sets a per-channel current threshold and a comparator clears the timer OCREF on every carrier period as soon as the sensed LED current reaches that threshold. Brightness and current are controlled on two independent axes:

- **Current threshold** — mapped **linearly** onto the safe operating window 50–100 mA (`RGBLED_DAC_CODE_FLOOR`…`RGBLED_DAC_CODE_MAX`).
- **PWM duty cycle** — gamma-corrected (`round(1000·(p/100)^0.87)`, enabled by `RGBLED_GAMMA_CORRECTION`, on by default) and scaled by the per-channel duty caps, so equal percentage steps look like equal brightness steps.

### Required configuration

- Three PWM TIM channels **plus a dedicated comparator and a dedicated DAC channel per color**. Sharing a comparator or a DAC channel between two colors is rejected by the driver's topology check.
- The board must route the timer's **OCREF-clear** input from the comparator output and enable it, so the comparator gates the carrier each period.
- The DAC is **externally triggered**: a pacing/trigger timer must be bound to `pattern_timer.phtim`. The driver free-runs it (~10 kHz) so the DAC reloads its threshold (DHR→DOR) even for static colors.
- HAL dependencies: TIM + DAC + COMP. Animations additionally require `USE_HAL_TIM_DMA == 1` **and** `USE_HAL_DAC_DMA == 1` (`RGBLED_PATTERN_PLAYBACK`).

### Safety — hardware current cap

The peak LED current is bounded in hardware: the DAC threshold can never exceed `RGBLED_DAC_CODE_MAX`, which corresponds to `RGBLED_LED_CURRENT_MAX_MA` (100 mA), whatever brightness or duty is requested. A current floor (`RGBLED_LED_CURRENT_FLOOR_MA`, 50 mA) keeps the loop inside its reliable regulation band, brightness below that floor is produced by lowering the duty cycle, not the current. The caps are fixed in the driver and cannot be raised by the application.

### API

`rgbled_init`, `rgbled_deinit`, `rgbled_set_color`, `rgbled_get_color`, and (when TIM + DAC DMA are enabled) `rgbled_fade_to_color`, `rgbled_play_pattern`.

## PWM Burst Control (`pwm_burst_control`)

### Principle

Each channel runs the same hardware current-mode loop, but the DAC current threshold is **pinned at the 100 mA cap** so every gated-on carrier period is regulated at the same peak current. Brightness is set by **burst density** — the fraction of carrier periods gated on inside a fixed burst window (linear map with a minimum-on floor). Holding the current constant removes the discontinuous-conduction nonlinearity, so this variant stays linear and color-accurate down to very low levels.

### Required configuration

- Three PWM TIM channels **plus a dedicated comparator and a dedicated DAC channel per color** (same topology rule as threshold control).
- The board must route and enable the timer's **OCREF-clear** input from the comparator output.
- A pacing timer must be bound to `pattern_timer.phtim`, it consumes one burst window (`RGBLED_PATTERN_SIZE` slots) per `RGBLED_BURST_WINDOW_PERIOD_MS`, at the `RGBLED_BURST_WINDOW_FREQUENCY` (1 kHz) window rate that keeps dimming flicker-free.
- Static colors and animations both stream the carrier compare register over DMA, animations use the **red** carrier's DMA half/transfer-complete interrupt to refill a double buffer, so the red carrier timer must keep its compare-DMA interrupt available (the driver wires the callback through the red timer's user-data pointer).
- HAL dependencies: TIM + DAC + COMP.

### Safety — hardware current cap

Overcurrent protection is the same hardware cap as threshold control (comparator + DAC at `RGBLED_LED_CURRENT_MAX_MA`, 100 mA), here the current is simply held at that regulated peak while brightness is carried entirely by the on/off ratio of whole carrier periods. It cannot be raised by the application.

### API

`rgbled_init`, `rgbled_deinit`, `rgbled_set_color`, `rgbled_get_color`, `rgbled_fade_to_color`, `rgbled_play_pattern` (animations are always available).

## Choosing a variant

- **PWM No Control** — simplest hardware (TIM only). Use when the LED path is already current-limited and you accept responsibility for the duty-cycle caps. It is the only variant whose caps are user-adjustable, and therefore the only one that can be driven outside a safe current range if misconfigured.
- **PWM Threshold Control** — hardware-safe current-mode dimming. It dims by moving the regulated current within the safe 50–100 mA window and trimming the duty cycle, with a perceptual gamma curve on brightness.
- **PWM Burst Control** — hardware-safe current-mode dimming that holds the current at the safe peak and dims by burst density, giving the most stable, color-accurate low-brightness behaviour.

Both threshold and burst control provide the same hardware overcurrent guarantee, the difference is *how* they dim: threshold control varies the current level (plus duty), burst control holds the current and varies the on/off ratio of carrier periods.

## How to use it?

Define one `rgbled_obj_t` object and provide an implementation of `rgbled_io_init()` that binds the resources required by the chosen variant. `rgbled_init()` links the two and must be called after all MCU peripherals have been initialized. After a successful init the driver forces the three outputs off until a color or animation is applied.

Typical usage:

1. Initialize the HAL, clocks, GPIOs, and TIM peripherals — and, for the threshold/burst variants, the per-channel DAC and comparator, the OCREF-clear wiring, and the pacing timer, enable DMA if animations are used.
2. Create one `rgbled_obj_t` object.
3. Override `rgbled_io_init()` to bind the red, green, blue, pacing, and (for threshold/burst) comparator and DAC resources for the object's `.id`.
4. Call `rgbled_init(&rgbled_obj, RGBLED_ID)`.
5. Use `rgbled_set_color()` for static colors, and `rgbled_get_color()` to read back the effective color.
6. Where available, use `rgbled_fade_to_color()` or `rgbled_play_pattern()` for animated transitions.
7. (PWM No Control only, optional) call `rgbled_set_max_duty_cycle()` to change the per-channel caps — within the board's safe current range.
8. Call `rgbled_deinit()` when the instance is no longer needed.

Calling `rgbled_set_color()` while an animation is running stops the active playback first, then applies the requested static color. `rgbled_deinit()` also stops any active playback before forcing the outputs off.

### Example — PWM No Control binding

```c
static rgbled_obj_t status_led;

int32_t rgbled_io_init(rgbled_obj_t *pobj)
{
  if (pobj == NULL)
  {
    return -1;
  }

  switch (pobj->id)
  {
    case 0U:
      pobj->red.phtim         = &htim1;
      pobj->red.pwm_channel   = HAL_TIM_CHANNEL_1;
      pobj->green.phtim       = &htim1;
      pobj->green.pwm_channel = HAL_TIM_CHANNEL_2;
      pobj->blue.phtim        = &htim1;
      pobj->blue.pwm_channel  = HAL_TIM_CHANNEL_3;

#if defined(RGBLED_PATTERN_PLAYBACK) && (RGBLED_PATTERN_PLAYBACK == 1)
      pobj->pattern_timer.phtim = &htim2;   /* pacing timer for animations */
#endif /* RGBLED_PATTERN_PLAYBACK */
      return 0;

    default:
      return -1;
  }
}

void app_init(void)
{
  const rgbled_color_t warm_white = {40U, 25U, 10U};

  (void)rgbled_init(&status_led, 0U);
  (void)rgbled_set_color(&status_led, &warm_white);
}
```

### Example — PWM Threshold / Burst Control binding (per channel)

The current-mode variants additionally bind one comparator and one DAC channel to each color (a dedicated pair per channel), plus the pacing timer:

```c
    case 0U:
      pobj->red.phtim           = &htim3;
      pobj->red.pwm_channel     = HAL_TIM_CHANNEL_1;
      pobj->green.phtim         = &htim4;
      pobj->green.pwm_channel   = HAL_TIM_CHANNEL_2;
      pobj->blue.phtim          = &htim2;
      pobj->blue.pwm_channel    = HAL_TIM_CHANNEL_1;
      pobj->pattern_timer.phtim = &htim6;         /* DAC trigger / pacing timer */

      pobj->red.phcomp        = &hcomp_red;
      pobj->red.phdac         = &hdac_red;
      pobj->red.dac_channel   = HAL_DAC_CHANNEL_1;
      pobj->green.phcomp      = &hcomp_green;
      pobj->green.phdac       = &hdac_green;
      pobj->green.dac_channel = HAL_DAC_CHANNEL_1;
      pobj->blue.phcomp       = &hcomp_blue;
      pobj->blue.phdac        = &hdac_blue;
      pobj->blue.dac_channel  = HAL_DAC_CHANNEL_1;
      break;
```

Once the object is initialized successfully, all other APIs of the selected variant can be used freely. `rgbled_get_color()` returns the color cached by the driver after the last successful static update, fade, or pattern launch.