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

#ifdef DEBUG_ALL_FAULTS

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

static serialPort_t *crashDumpUARTPort = NULL;
static char uartBuf[128];

void setup_fault_handler_mode(void)
{
    LED2_ON;

#ifndef USE_OSD_SLAVE
    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors)
    {
        stopMotors();
    }
#endif

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder)
    {
        transponderIrDisable();
    }
#endif

    LED1_OFF;
    LED0_OFF;

    // setup uart for debugging
    crashDumpUARTPort = findSharedSerialPort(FUNCTION_MSP, ~0); // if the serial port is open for anything else, we need to close it.
    if (!crashDumpUARTPort)
    {
        // Port in use close it.
        closeSerialPort(crashDumpUARTPort);
    }
    // open serial port again to dump data.
    crashDumpUARTPort = openSerialPort(crashDumpUARTPort->identifier, FUNCTION_MSP, NULL, NULL, 115200, MODE_RXTX, SERIAL_PARITY_NO | SERIAL_NOT_INVERTED);
}

void HardFault_Handler_c(crashStackContext_t *ctxt)
{
    setup_fault_handler_mode();
    snprintf(uartBuf, 128, "fault handler: %s\n", __func__);
    while (1)
        ;
}

void MemManage_Handler_c(crashStackContext_t *ctxt)
{

    setup_fault_handler_mode();
    snprintf(uartBuf, 128, "fault handler: %s\n", __func__);
    while (1)
        ;
}

void BusFault_Handler_c(crashStackContext_t *ctxt)
{

    setup_fault_handler_mode();
    snprintf(uartBuf, 128, "fault handler: %s\n", __func__);
    while (1)
        ;
}

void UsageFault_Handler_c(crashStackContext_t *ctxt)
{

    setup_fault_handler_mode();
    snprintf(uartBuf, 128, "fault handler: %s\n", __func__);
    while (1)
        ;
}

#else
#ifdef DEBUG_HARDFAULTS
// from: https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
 * hard_fault_handler_c:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void hard_fault_handler_c(unsigned long *hardfault_args)
{
    volatile unsigned long stacked_r0;
    volatile unsigned long stacked_r1;
    volatile unsigned long stacked_r2;
    volatile unsigned long stacked_r3;
    volatile unsigned long stacked_r12;
    volatile unsigned long stacked_lr;
    volatile unsigned long stacked_pc;
    volatile unsigned long stacked_psr;
    volatile unsigned long _CFSR;
    volatile unsigned long _HFSR;
    volatile unsigned long _DFSR;
    volatile unsigned long _AFSR;
    volatile unsigned long _BFAR;
    volatile unsigned long _MMAR;

    stacked_r0 = ((unsigned long)hardfault_args[0]);
    stacked_r1 = ((unsigned long)hardfault_args[1]);
    stacked_r2 = ((unsigned long)hardfault_args[2]);
    stacked_r3 = ((unsigned long)hardfault_args[3]);
    stacked_r12 = ((unsigned long)hardfault_args[4]);
    stacked_lr = ((unsigned long)hardfault_args[5]);
    stacked_pc = ((unsigned long)hardfault_args[6]);
    stacked_psr = ((unsigned long)hardfault_args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28)));

    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30)));

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34)));
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38)));

    __asm("BKPT #0\n"); // Break into the debugger
}

#else
void HardFault_Handler(void)
{
    LED2_ON;

#ifndef USE_OSD_SLAVE
    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors)
    {
        stopMotors();
    }
#endif

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder)
    {
        transponderIrDisable();
    }
#endif

    LED1_OFF;
    LED0_OFF;

    while (1)
    {
#ifdef LED2
        delay(50);
        LED2_TOGGLE;
#endif
    }
}
#endif
#endif
