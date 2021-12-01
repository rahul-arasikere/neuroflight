#include "uart4Serial.h"

serialPort_t *uart4Serial = NULL;

serialPort_t* getUART4() {
	if(uart4Serial == NULL)
		uart4Serial = openSerialPort(SERIAL_PORT_UART4, FUNCTION_BLACKBOX, NULL, NULL, 115200, MODE_RXTX, 0);
	return uart4Serial;
}