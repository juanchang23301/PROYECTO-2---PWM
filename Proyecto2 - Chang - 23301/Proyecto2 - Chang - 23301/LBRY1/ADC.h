/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n: 
* Este archivo de encabezado define la interfaz p�blica para el m�dulo ADC.
* Proporciona declaraciones de funciones para la configuraci�n del convertidor
* anal�gico-digital, lectura de canales, filtrado de se�ales y conversi�n
* de valores digitales a �ngulos para control de servomotores.
*

* Conexiones de Hardware Soportadas: 
*   - ADC0: PC0 (Potenci�metro Control Base)
*   - ADC1: PC1 (Potenci�metro Control Brazo1)
*   - ADC2: PC2 (Potenci�metro Control Brazo2)
*   - ADC3: PC3 (Potenci�metro Control Pinza)
************************************************************************/

#ifndef ADC_H
#define ADC_H
#include <avr/io.h>

// PROTOTIPOS DE FUNCIONES
void ADC_init(void);
uint16_t ADC_read(uint8_t canal);
uint8_t ADC_Angulo(uint16_t ADC_VALUE);
uint16_t ADC_read_Filtr(uint8_t canal, uint8_t numMuestras);

#endif // ADC_H