/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n:
* Esta librer�a implementa funciones para comunicaci�n serial USART del
* microcontrolador AVR. Proporciona funcionalidades completas para
* transmisi�n y recepci�n de datos seriales, incluyendo configuraci�n
* autom�tica de velocidad de transmisi�n y manejo de interrupciones.
*
*
* Conexiones de Hardware:
*   - RX (Recepci�n): PD0 - Pin de entrada serial
*   - TX (Transmisi�n): PD1 - Pin de salida serial
************************************************************************/

#include "USART.h"
#include <avr/interrupt.h>

void initUSART(void) {
	// Configurar velocidad de transmisi�n
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	
	// Habilitar transmisor y receptor, y habilitar interrupci�n de recepci�n
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	
	// Formato de trama: 8 bits de datos, 1 bit de parada, sin paridad
	UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}

void sendUSARTData(char dato) {
	// Esperar hasta que el buffer de transmisi�n est� vac�o
	while (!(UCSR0A & (1 << UDRE0)));
	
	// Colocar dato en el buffer
	UDR0 = dato;
}

void sendUSARTString(const char* cadena) {
	// Enviar cada car�cter de la cadena
	while (*cadena) {
		sendUSARTData(*cadena++);
	}
}

char receiveUSARTData(void) {
	// Esperar hasta que haya un dato disponible
	while (!(UCSR0A & (1 << RXC0)));
	
	// Retornar dato recibido
	return UDR0;
}