/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción del Módulo:
* Esta librería implementa funciones para la configuración y lectura del
* convertidor analógico-digital (ADC) del microcontrolador AVR. Incluye
* funcionalidades de filtrado digital para reducir el ruido en las lecturas
* de los potenciómetros y conversión directa a valores de ángulo para
* control de servomotores.
*
* Conexiones de Hardware:
*   - ADC0: PC0 (Potenciómetro Control Base)
*   - ADC1: PC1 (Potenciómetro Control Brazo1)
*   - ADC2: PC2 (Potenciómetro Control Brazo2)
*   - ADC3: PC3 (Potenciómetro Control Pinza)
************************************************************************/

#include "ADC.h"
#include <util/delay.h>

void ADC_init(void) {
	// Configurar los pines como entradas (PC0-PC3)
	DDRC &= ~((1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3));
	
	// Deshabilitar pull-ups internos
	PORTC &= ~((1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3));
	
	// Configurar ADC
	ADMUX = (0 << REFS1) | (1 << REFS0); // Referencia AVCC
	ADCSRA = (1 << ADEN) |                // Habilitar ADC
	(0 << ADATE) |              // Deshabilitar auto-trigger
	(0 << ADIF) |               // Limpiar flag de interrupción
	(0 << ADIE) |               // Deshabilitar interrupción
	(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

uint16_t ADC_read(uint8_t canal) {
	// Seleccionar canal (0-7)
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	
	// Iniciar conversión
	ADCSRA |= (1 << ADSC);
	
	// Esperar a que la conversión termine
	while (ADCSRA & (1 << ADSC));
	
	// Leer resultado
	return ADC;
}

uint8_t ADC_Angulo(uint16_t ADC_VALUE) {
	// Convertir valor ADC (0-1023) a ángulo (0-180)
	return (uint8_t)((uint32_t)ADC_VALUE * 180 / 1023);
}

uint16_t ADC_read_Filtr(uint8_t canal, uint8_t numMuestras) {
	uint32_t suma = 0;
	
	// Tomar varias muestras y promediar
	for (uint8_t i = 0; i < numMuestras; i++) {
		suma += ADC_read(canal);
		_delay_ms(10); // Pequeña pausa entre lecturas
	}
	
	// Calcular el promedio
	uint16_t valorFiltrado = (uint16_t)(suma / numMuestras);
	
	// También podemos aplicar un filtro de zona muerta para evitar pequeñas fluctuaciones
	static uint16_t valorAnterior[4] = {0, 0, 0, 0}; // Para 4 canales (0-3)
	
	// Si la diferencia es pequeña, mantener el valor anterior
	if (canal < 4) { // Asegurarse de que el canal está en rango
		if (abs(valorFiltrado - valorAnterior[canal]) < 5) {
			valorFiltrado = valorAnterior[canal];
			} else {
			valorAnterior[canal] = valorFiltrado;
		}
	}
	
	return valorFiltrado;
}