# CMSIS Build CMakeLists generated on 2024-03-04T21:25:44

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET Project)
set(CPU Cortex-M4)
set(PRJ_DIR "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM")
set(OUT_DIR "C:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM/out/Project/Project")
set(INT_DIR "C:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM/tmp/Project/Project")
set(FPU SP_FPU)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE speed)
set(DEBUG on)
set(AS_FLAGS_GLOBAL "-masm=auto")
set(CC_FLAGS_GLOBAL "-std=c99 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-xc++ -std=c++11 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM/Project_Project.sct")

set(DEFINES
  STM32G431xx
  _RTE_
  USE_HAL_DRIVER
)

set(INC_PATHS
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Inc"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Inc"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/CMSIS/Device/ST/STM32G4xx/Include"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/CMSIS/Include"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Application"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM/RTE/_Project"
  "C:/Users/19308/AppData/Local/arm/packs/ARM/CMSIS/5.9.0/CMSIS/Core/Include"
  "C:/Users/19308/AppData/Local/arm/packs/Keil/STM32G4xx_DFP/1.5.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include"
)

set(AS_ARM_SRC_FILES
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/MDK-ARM/startup_stm32g431xx.s"
)

set(CC_SRC_FILES
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Application/lcd.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Application/led.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/adc.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/gpio.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/main.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/stm32g4xx_hal_msp.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/stm32g4xx_it.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/system_stm32g4xx.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Core/Src/tim.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c"
  "c:/Users/19308/Desktop/Keil 5-project/14th-lanqiaocup1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c"
)

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "c:/Users/19308/.vcpkg/artifacts/2139c4c6/compilers.arm.armclang/6.21.0/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.21.0")
set(TOOLCHAIN_VERSION_MIN "6.21.0")
include ("C:/Users/19308/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/AC6.6.18.0.cmake")
include ("C:/Users/19308/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS} ${AS_FLAGS_GLOBAL}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "Project")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
