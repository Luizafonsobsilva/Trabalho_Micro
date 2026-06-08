cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion RISCV_ZCB_RPI_2_2_0_3)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# RP2350 em modo RISC-V (Hazard3)
set(PICO_PLATFORM rp2350-riscv)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(ap2_testes C CXX ASM)
pico_sdk_init()

# ========================================
# ALVO PRINCIPAL: teste_integrado
# ADC + UART funcionando juntos
# ========================================
add_executable(teste_integrado
    teste_integrado.c
)
pico_set_program_name(teste_integrado "teste_integrado")
pico_set_program_version(teste_integrado "0.1")

target_link_libraries(teste_integrado
    pico_stdlib
    hardware_adc
    hardware_uart
)

pico_enable_stdio_usb(teste_integrado 1)
pico_enable_stdio_uart(teste_integrado 0)
pico_add_extra_outputs(teste_integrado)

# ========================================
# ALVOS ISOLADOS (fallback para debug)
# Descomente se precisar testar um modulo sozinho
# ========================================

add_executable(teste_adc teste_adc.c)
pico_set_program_name(teste_adc "teste_adc")
pico_set_program_version(teste_adc "0.1")
target_link_libraries(teste_adc pico_stdlib hardware_adc)
pico_enable_stdio_usb(teste_adc 1)
pico_enable_stdio_uart(teste_adc 0)
pico_add_extra_outputs(teste_adc)

add_executable(teste_uart teste_uart.c)
pico_set_program_name(teste_uart "teste_uart")
pico_set_program_version(teste_uart "0.1")
target_link_libraries(teste_uart pico_stdlib hardware_uart)
pico_enable_stdio_usb(teste_uart 1)
pico_enable_stdio_uart(teste_uart 0)
pico_add_extra_outputs(teste_uart)

# ========================================
# AP3: PWM + Timer/Timer Interrupt + Interrupcao Externa
# (ADC e UART da AP2 como entradas de apoio)
# ========================================
add_executable(teste_ap3 teste_ap3.c)
pico_set_program_name(teste_ap3 "teste_ap3")
pico_set_program_version(teste_ap3 "0.1")
target_link_libraries(teste_ap3
    pico_stdlib
    hardware_pwm
    hardware_timer
    hardware_clocks
    hardware_adc
    hardware_uart
)
pico_enable_stdio_usb(teste_ap3 1)
pico_enable_stdio_uart(teste_ap3 0)
pico_add_extra_outputs(teste_ap3)
