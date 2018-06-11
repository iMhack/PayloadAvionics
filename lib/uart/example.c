#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "uart.h"

#define BAUD_RATE 38400

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

// write a string to the uart
#define uart_print(s) uart_print_P(PSTR(s))
void uart_print_P(const char *str)
{
	char c;
	while (1) {
		c = pgm_read_byte(str++);
		if (!c) break;
		uart_putchar(c);
	}
}

// A very basic example...
// when the user types a character, print it back
int main(void)
{
	uint8_t c;

	CPU_PRESCALE(0);  // run at 16 MHz
	uart_init(BAUD_RATE);
	uart_print("UART Example\r\n");
	while (1) {
		if (uart_available()) {
			c = uart_getchar();
			uart_print("Byte: ");
			uart_putchar(c);
			uart_putchar('\r');
			uart_putchar('\n');
		}
	}
}
