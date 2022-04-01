/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "platform.h"

#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "fc/fc_init.h"

#include "io/serial.h"

#include "flight/mixer.h"

#if defined(STM32F7)
#include "stm32f7xx_hal.h"
#else
#error "configuration not supported yet!"
#endif

typedef struct PG_PACKED crashStackContext
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t return_address;
    uint32_t xpsr;
} crashStackContext_t;

#define JMP_INTO_SECONDARY_FAULT_HANDLER() __asm volatile(          \
    " tst lr, #4                                                \n" \
    " ite eq                                                    \n" \
    " mrseq r0, msp                                             \n" \
    " mrsne r0, psp                                             \n" \
    " ldr r1, [r0, #24]                                         \n" \
    " bl secondary_fault_handler                                 \n")

extern void SystemClock_Config(void);

void delay_dumb(int ms)
{
    for (int i = 0; i < ms * 200000; i++)
        asm volatile("\tnop\n");
}

void secondary_fault_handler(crashStackContext_t *ctxt)
{
    UART_HandleTypeDef huart3;
    LED2_OFF;
    LED1_OFF;
    LED0_ON; /*errored out*/
    delay_dumb(1000);

    HAL_DeInit();
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        while (1)
        {
            LED0_OFF;
            delay_dumb(500);
            LED0_ON;
            delay_dumb(500);
        }
    }
    while (1)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)"hardfault handler\n", 19, 500);
    }
}
