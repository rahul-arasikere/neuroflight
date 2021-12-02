#ifndef UART4SERIAL_H
#define UART4SERIAL_H


#include "io/serial.h"

static serialPort_t *uart4Serial = NULL;

static void initUART4() {
	uart4Serial = openSerialPort(SERIAL_PORT_UART4, FUNCTION_BLACKBOX, NULL, NULL, 115200, MODE_RXTX, 0);
}

#endif