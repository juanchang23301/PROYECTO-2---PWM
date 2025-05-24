/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n del M�dulo:
* Esta librer�a implementa funciones para la configuraci�n y lectura del
* convertidor anal�gico-digital (ADC) del microcontrolador AVR. Incluye
* funcionalidades de filtrado digital para reducir el ruido en las lecturas
* de los potenci�metros y conversi�n directa a valores de �ngulo para
* control de servomotores.
*
* Conexiones de Hardware:
*   - ADC0: PC0 (Potenci�metro Control Base)
*   - ADC1: PC1 (Potenci�metro Control Brazo1)
*   - ADC2: PC2 (Potenci�metro Control Brazo2)
*   - ADC3: PC3 (Potenci�metro Control Pinza)
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
	(0 << ADIF) |               // Limpiar flag de interrupci�n
	(0 << ADIE) |               // Deshabilitar interrupci�n
	(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

uint16_t ADC_read(uint8_t canal) {
	// Seleccionar canal (0-7)
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	
	// Iniciar conversi�n
	ADCSRA |= (1 << ADSC);
	
	// Esperar a que la conversi�n termine
	while (ADCSRA & (1 << ADSC));
	
	// Leer resultado
	return ADC;
}

uint8_t ADC_Angulo(uint16_t ADC_VALUE) {
	// Convertir valor ADC (0-1023) a �ngulo (0-180)
	return (uint8_t)((uint32_t)ADC_VALUE * 180 / 1023);
}

uint16_t ADC_read_Filtr(uint8_t canal, uint8_t numMuestras) {
	uint32_t suma = 0;
	
	// Tomar varias muestras y promediar
	for (uint8_t i = 0; i < numMuestras; i++) {
		suma += ADC_read(canal);
		_delay_ms(10); // Peque�a pausa entre lecturas
	}
	
	// Calcular el promedio
	uint16_t valorFiltrado = (uint16_t)(suma / numMuestras);
	
	// Tambi�n podemos aplicar un filtro de zona muerta para evitar peque�as fluctuaciones
	static uint16_t valorAnterior[4] = {0, 0, 0, 0}; // Para 4 canales (0-3)
	
	// Si la diferencia es peque�a, mantener el valor anterior
	if (canal < 4) { // Asegurarse de que el canal est� en rango
		if (abs(valorFiltrado - valorAnterior[canal]) < 5) {
			valorFiltrado = valorAnterior[canal];
			} else {
			valorAnterior[canal] = valorFiltrado;
		}
	}
	
	return valorFiltrado;
}