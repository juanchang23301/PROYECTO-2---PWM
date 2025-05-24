/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción:
* Esta librería implementa funciones para comunicación serial USART del
* microcontrolador AVR. Proporciona funcionalidades completas para
* transmisión y recepción de datos seriales, incluyendo configuración
* automática de velocidad de transmisión y manejo de interrupciones.
*
*
* Conexiones de Hardware:
*   - RX (Recepción): PD0 - Pin de entrada serial
*   - TX (Transmisión): PD1 - Pin de salida serial
************************************************************************/

#include "USART.h"
#include <avr/interrupt.h>

void initUSART(void) {
	// Configurar velocidad de transmisión
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	
	// Habilitar transmisor y receptor, y habilitar interrupción de recepción
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	
	// Formato de trama: 8 bits de datos, 1 bit de parada, sin paridad
	UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}

void sendUSARTData(char dato) {
	// Esperar hasta que el buffer de transmisión esté vacío
	while (!(UCSR0A & (1 << UDRE0)));
	
	// Colocar dato en el buffer
	UDR0 = dato;
}

void sendUSARTString(const char* cadena) {
	// Enviar cada carácter de la cadena
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