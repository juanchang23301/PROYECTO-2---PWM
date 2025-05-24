/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción: 
* Este archivo de encabezado define la interfaz pública para el módulo ADC.
* Proporciona declaraciones de funciones para la configuración del convertidor
* analógico-digital, lectura de canales, filtrado de señales y conversión
* de valores digitales a ángulos para control de servomotores.
*

* Conexiones de Hardware Soportadas: 
*   - ADC0: PC0 (Potenciómetro Control Base)
*   - ADC1: PC1 (Potenciómetro Control Brazo1)
*   - ADC2: PC2 (Potenciómetro Control Brazo2)
*   - ADC3: PC3 (Potenciómetro Control Pinza)
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