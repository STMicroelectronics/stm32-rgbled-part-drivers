# origin-pack: STMicroelectronics::rgbled_part_drivers@2.0.0
# file-format: 1.0.0
project(STMicroelectronics_rgbled_part_drivers_2_0_0)
cmake_minimum_required(VERSION 3.30)
add_library(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE)

# List of all CMSIS properties that influence conditions for this package


# Device specific defined symbols








# Enable all components in this package
if(CMSIS_ENTIRE_STMicroelectronics_rgbled_part_drivers_2_0_0)
  list(APPEND CMSIS_COMPONENTS_LIST "Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM NO CONTROL#Cversion:1.0.0")
  list(APPEND CMSIS_COMPONENTS_LIST "Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM THRESHOLD CONTROL#Cversion:1.0.0")
  list(APPEND CMSIS_COMPONENTS_LIST "Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM BURST CONTROL#Cversion:1.0.0")
endif()

# All conditions used by this package

# condition: STMicroelectronics.rgbled_part_drivers.2.0.0:RGBLED Advanced Condition
# description: STMicroelectronics RGBLED current-mode (DAC + COMP) Driver
set(STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition "$<AND:$<NOT:$<STREQUAL:$<FILTER:${CMSIS_COMPONENTS_LIST},INCLUDE,.*Cclass:Device#.*Cgroup:STM32 HAL#.*Csub:TIM(#.*|$)>,>>,$<NOT:$<STREQUAL:$<FILTER:${CMSIS_COMPONENTS_LIST},INCLUDE,.*Cclass:Device#.*Cgroup:STM32 HAL#.*Csub:DAC(#.*|$)>,>>,$<NOT:$<STREQUAL:$<FILTER:${CMSIS_COMPONENTS_LIST},INCLUDE,.*Cclass:Device#.*Cgroup:STM32 HAL#.*Csub:COMP(#.*|$)>,>>>")
message(DEBUG "CMSIS condition STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition enabled")


# condition: STMicroelectronics.rgbled_part_drivers.2.0.0:RGBLED CFG Condition
# description: STMicroelectronics configuration for RGBLED part driver
set(STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_CFG_Condition "1")
message(DEBUG "CMSIS condition STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_CFG_Condition enabled")


# condition: STMicroelectronics.rgbled_part_drivers.2.0.0:RGBLED Core Condition
# description: STMicroelectronics RGBLED Core Driver
set(STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Core_Condition "$<NOT:$<STREQUAL:$<FILTER:${CMSIS_COMPONENTS_LIST},INCLUDE,.*Cclass:Device#.*Cgroup:STM32 HAL#.*Csub:TIM(#.*|$)>,>>")
message(DEBUG "CMSIS condition STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Core_Condition enabled")

# Files and components in this package
if("Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM NO CONTROL#Cversion:1.0.0" IN_LIST CMSIS_COMPONENTS_LIST)  # Driver for RGB LEDs driven by STM32 HAL TIM PWM channels
  message(DEBUG "Using component Board_Part_RGBLED_PWM_NO_CONTROL_1_0_0")
  target_compile_definitions(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Core_Condition}:-DCMSIS_USE_Board_Part_RGBLED_PWM_NO_CONTROL_1_0_0=1>")
  target_include_directories(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Core_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_no_control>")
  target_sources(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Core_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_no_control/rgbled.c>")
endif()

if("Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM THRESHOLD CONTROL#Cversion:1.0.0" IN_LIST CMSIS_COMPONENTS_LIST)  # Driver for RGB LEDs driven by STM32 HAL TIM PWM channels
  message(DEBUG "Using component Board_Part_RGBLED_PWM_THRESHOLD_CONTROL_1_0_0")
  target_compile_definitions(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:-DCMSIS_USE_Board_Part_RGBLED_PWM_THRESHOLD_CONTROL_1_0_0=1>")
  target_include_directories(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_threshold_control>")
  target_sources(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_threshold_control/rgbled.c>")
endif()

if("Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:RGBLED#Csub:PWM BURST CONTROL#Cversion:1.0.0" IN_LIST CMSIS_COMPONENTS_LIST)  # Driver for RGB LEDs driven by STM32 HAL TIM PWM channels
  message(DEBUG "Using component Board_Part_RGBLED_PWM_BURST_CONTROL_1_0_0")
  target_compile_definitions(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:-DCMSIS_USE_Board_Part_RGBLED_PWM_BURST_CONTROL_1_0_0=1>")
  target_include_directories(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_burst_control>")
  target_sources(STMicroelectronics_rgbled_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.rgbled_part_drivers.2.0.0_RGBLED_Advanced_Condition}:${CMAKE_CURRENT_LIST_DIR}/pwm_burst_control/rgbled.c>")
endif()

