cmake_minimum_required(VERSION 3.20)

project(myStm32_Pj C ASM)

set(TARGET ${PROJECT_NAME})


# ============================================================
# MCU
# ============================================================

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

add_compile_options(
    ${MCU_FLAGS}

    -ffunction-sections
    -fdata-sections
)


# ============================================================
# STM32
# ============================================================

add_compile_definitions(
    STM32F407xx
    USE_HAL_DRIVER
    DEBUG_INFO
    DEBUG_WARN
    DEBUG_ERROR
)


# ============================================================
# Source files
# ============================================================

set(SOURCES
    User/main.c
    User/cmd.c
    CMSIS/Device/ST/STM32F4xx/Source/system_stm32f4xx.c
    Interrupt/src/stm32f4xx_it.c
    SYSTEM/delay/delay.c
    SYSTEM/sys/sys.c
    SYSTEM/usart/usart.c
    SYSTEM/esay_log/easy_log.c
    BSP/Led/led.c
    BSP/Motor/motor.c
    BSP/PID/pid.c
    CMSIS/Device/ST/STM32F4xx/Source/gcc/startup_stm32f407xx.s
)


# ============================================================
# HAL sources
# ============================================================

list(APPEND SOURCES
    HAL/Src/stm32f4xx_hal.c
    HAL/Src/stm32f4xx_hal_adc.c
    HAL/Src/stm32f4xx_hal_adc_ex.c
    HAL/Src/stm32f4xx_hal_can.c
    HAL/Src/stm32f4xx_hal_cec.c
    HAL/Src/stm32f4xx_hal_cortex.c
    HAL/Src/stm32f4xx_hal_crc.c
    HAL/Src/stm32f4xx_hal_cryp.c
    HAL/Src/stm32f4xx_hal_cryp_ex.c
    HAL/Src/stm32f4xx_hal_dac.c
    HAL/Src/stm32f4xx_hal_dac_ex.c
    HAL/Src/stm32f4xx_hal_dcmi.c
    HAL/Src/stm32f4xx_hal_dcmi_ex.c
    HAL/Src/stm32f4xx_hal_dfsdm.c
    HAL/Src/stm32f4xx_hal_dma.c
    HAL/Src/stm32f4xx_hal_dma2d.c
    HAL/Src/stm32f4xx_hal_dma_ex.c
    HAL/Src/stm32f4xx_hal_dsi.c
    HAL/Src/stm32f4xx_hal_eth.c
    HAL/Src/stm32f4xx_hal_exti.c
    HAL/Src/stm32f4xx_hal_flash.c
    HAL/Src/stm32f4xx_hal_flash_ex.c
    HAL/Src/stm32f4xx_hal_flash_ramfunc.c
    HAL/Src/stm32f4xx_hal_fmpi2c.c
    HAL/Src/stm32f4xx_hal_fmpi2c_ex.c
    HAL/Src/stm32f4xx_hal_fmpsmbus.c
    HAL/Src/stm32f4xx_hal_fmpsmbus_ex.c
    HAL/Src/stm32f4xx_hal_gpio.c
    HAL/Src/stm32f4xx_hal_hash.c
    HAL/Src/stm32f4xx_hal_hash_ex.c
    HAL/Src/stm32f4xx_hal_hcd.c
    HAL/Src/stm32f4xx_hal_i2c.c
    HAL/Src/stm32f4xx_hal_i2c_ex.c
    HAL/Src/stm32f4xx_hal_i2s.c
    HAL/Src/stm32f4xx_hal_i2s_ex.c
    HAL/Src/stm32f4xx_hal_irda.c
    HAL/Src/stm32f4xx_hal_iwdg.c
    HAL/Src/stm32f4xx_hal_lptim.c
    HAL/Src/stm32f4xx_hal_ltdc.c
    HAL/Src/stm32f4xx_hal_ltdc_ex.c
    HAL/Src/stm32f4xx_hal_mmc.c
    HAL/Src/stm32f4xx_hal_nand.c
    HAL/Src/stm32f4xx_hal_nor.c
    HAL/Src/stm32f4xx_hal_pccard.c
    HAL/Src/stm32f4xx_hal_pcd.c
    HAL/Src/stm32f4xx_hal_pcd_ex.c
    HAL/Src/stm32f4xx_hal_pwr.c
    HAL/Src/stm32f4xx_hal_pwr_ex.c
    HAL/Src/stm32f4xx_hal_qspi.c
    HAL/Src/stm32f4xx_hal_rcc.c
    HAL/Src/stm32f4xx_hal_rcc_ex.c
    HAL/Src/stm32f4xx_hal_rng.c
    HAL/Src/stm32f4xx_hal_rtc.c
    HAL/Src/stm32f4xx_hal_rtc_ex.c
    HAL/Src/stm32f4xx_hal_sai.c
    HAL/Src/stm32f4xx_hal_sai_ex.c
    HAL/Src/stm32f4xx_hal_sd.c
    HAL/Src/stm32f4xx_hal_sdram.c
    HAL/Src/stm32f4xx_hal_smartcard.c
    HAL/Src/stm32f4xx_hal_smbus.c
    HAL/Src/stm32f4xx_hal_spdifrx.c
    HAL/Src/stm32f4xx_hal_spi.c
    HAL/Src/stm32f4xx_hal_sram.c
    HAL/Src/stm32f4xx_hal_tim.c
    HAL/Src/stm32f4xx_hal_tim_ex.c
    HAL/Src/stm32f4xx_hal_uart.c
    HAL/Src/stm32f4xx_hal_usart.c
    HAL/Src/stm32f4xx_hal_wwdg.c
    HAL/Src/Legacy/stm32f4xx_hal_can.c
    HAL/Src/Legacy/stm32f4xx_hal_eth.c
    HAL/Src/stm32f4xx_ll_adc.c
    HAL/Src/stm32f4xx_ll_crc.c
    HAL/Src/stm32f4xx_ll_dac.c
    HAL/Src/stm32f4xx_ll_dma.c
    HAL/Src/stm32f4xx_ll_dma2d.c
    HAL/Src/stm32f4xx_ll_exti.c
    HAL/Src/stm32f4xx_ll_fmc.c
    HAL/Src/stm32f4xx_ll_fmpi2c.c
    HAL/Src/stm32f4xx_ll_fsmc.c
    HAL/Src/stm32f4xx_ll_gpio.c
    HAL/Src/stm32f4xx_ll_i2c.c
    HAL/Src/stm32f4xx_ll_lptim.c
    HAL/Src/stm32f4xx_ll_pwr.c
    HAL/Src/stm32f4xx_ll_rcc.c
    HAL/Src/stm32f4xx_ll_rng.c
    HAL/Src/stm32f4xx_ll_rtc.c
    HAL/Src/stm32f4xx_ll_sdmmc.c
    HAL/Src/stm32f4xx_ll_spi.c
    HAL/Src/stm32f4xx_ll_tim.c
    HAL/Src/stm32f4xx_ll_usart.c
    HAL/Src/stm32f4xx_ll_usb.c
    HAL/Src/stm32f4xx_ll_utils.c
)


# ============================================================
# Create target
# ============================================================

add_executable(${TARGET} ${SOURCES})


# ============================================================
# Include directories
# ============================================================

target_include_directories(${TARGET} PRIVATE
    HAL/Inc
    HAL/Inc/Legacy
    HAL_Config
    Interrupt/include
    CMSIS/Include
    CMSIS/Device/ST/STM32F4xx/Inclde
    SYSTEM/delay
    SYSTEM/sys
    SYSTEM/usart
    SYSTEM/esay_log
    BSP/Led
    BSP/Motor
    BSP/PID
)


# ============================================================
# Linker Script
# ============================================================

set(LINKER_SCRIPT
    ${CMAKE_SOURCE_DIR}/Linker/STM32F407ZGTX_FLASH.ld
)

target_link_options(${TARGET} PRIVATE

    -T${LINKER_SCRIPT}

    ${MCU_FLAGS}

    -Wl,-Map=${TARGET}.map
    -Wl,--gc-sections

    --specs=nano.specs
    --specs=nosys.specs
    -u _printf_float
)


# ============================================================
# Generate HEX / BIN
# ============================================================

add_custom_command(TARGET ${TARGET} POST_BUILD

    COMMAND arm-none-eabi-size ${TARGET}

    COMMAND arm-none-eabi-objcopy
            -O ihex
            ${TARGET}
            ${TARGET}.hex

    COMMAND arm-none-eabi-objcopy
            -O binary
            ${TARGET}
            ${TARGET}.bin
)