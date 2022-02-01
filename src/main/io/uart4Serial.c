#include "uart4Serial.h"
#include "drivers/time.h"

serialPort_t *uart4Serial = NULL;

serialPort_t* getUART4() {
	if(uart4Serial == NULL) {
		uart4Serial = openSerialPort(SERIAL_PORT_UART4, FUNCTION_BLACKBOX, NULL, NULL, 115200, MODE_RXTX, 0);
		if(millis() < 10000) // waiting 10 seconds so that the xbee connects since drone power on
			delay(10000 - millis()); 
	}
	return uart4Serial;
}

void write_byte(unsigned char byte) {
	serialWrite(getUART4(), byte);
}