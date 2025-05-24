/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n: 
* Este archivo define la interfaz p�blica para el m�dulo de comunicaci�n
* USART. Proporciona declaraciones de funciones para configuraci�n,
* transmisi�n y recepci�n de datos seriales, as� como constantes de
* configuraci�n para velocidad de transmisi�n.
*
* Conexiones de Hardware Requeridas: 
*   - RX (Recepci�n): PD0 - Conectar a TX del dispositivo externo
*   - TX (Transmisi�n): PD1 - Conectar a RX del dispositivo externo
*   - GND: Com�n entre dispositivos
************************************************************************/

#ifndef USART_H
#define USART_H

#define F_CPU 16000000UL
#include <avr/io.h>

#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)

void initUSART(void);                           // Initialize USART
void sendUSARTData(char dato);                  // Send data via USART
void sendUSARTString(const char* cadena);       // Send string via USART
char receiveUSARTData(void);                    // Receive data from USART

#endif // USART_H