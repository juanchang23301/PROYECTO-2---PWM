/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción: 
* Este archivo define la interfaz pública para el módulo de comunicación
* USART. Proporciona declaraciones de funciones para configuración,
* transmisión y recepción de datos seriales, así como constantes de
* configuración para velocidad de transmisión.
*
* Conexiones de Hardware Requeridas: 
*   - RX (Recepción): PD0 - Conectar a TX del dispositivo externo
*   - TX (Transmisión): PD1 - Conectar a RX del dispositivo externo
*   - GND: Común entre dispositivos
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