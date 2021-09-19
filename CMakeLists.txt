# To generate build files, create build/ enter the directory and run the following
#     cmake -DCMAKE_TOOLCHAIN_FILE="arm-none-eabi-gcc.cmake" -DCMAKE_MAKE_PROGRAM=ninja.exe -G "Ninja" ..

cmake_minimum_required(VERSION 3.15.3)

# Optional: print out extra messages to see what is going on. Comment it to have less verbose messages
set(CMAKE_VERBOSE_MAKEFILE ON)

# Generate build_commands.json for intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

# Setup project, output and linker file
project(FirstNucleo)
set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/STM32F446RETx_FLASH.ld)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Optional: issue a message to be sure it uses the correct toolchain file.
message(STATUS "CMAKE_TOOLCHAIN_FILE is: ${CMAKE_TOOLCHAIN_FILE}")

file(GLOB CORE_SRC Core/Src/*.c)
set(DRIVER_SRC
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c 
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c 
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c 
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c 
    Middlewares/Third_Party/FreeRTOS/Source/list.c 
    Middlewares/Third_Party/FreeRTOS/Source/queue.c 
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c 
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c 
    Middlewares/Third_Party/FreeRTOS/Source/timers.c 
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c 
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    )

set(ASM_SRC
    startup_stm32f446xx.s
)

# List of source files
set(SRC_FILES
        ${ASM_SRC}
        ${CORE_SRC}
        ${DRIVER_SRC}
        )

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SRC_FILES})

# List of compiler defines, prefix with -D compiler option
target_compile_definitions(${EXECUTABLE} PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32F446xx
        )

# List of include directories
target_include_directories(${EXECUTABLE} PRIVATE
        Core/Inc 
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        )

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard

        -fdata-sections
        -ffunction-sections

        -Wall
        -O0
        -g3
        )

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
        -T${LINKER_FILE}
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Xlinker -print-memory-usage -Xlinker
        )

# Optional: Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})

# Optional: Create hex, bin and S-Record files after the build
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 ${EXECUTABLE} ${PROJECT_NAME}.s19
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)