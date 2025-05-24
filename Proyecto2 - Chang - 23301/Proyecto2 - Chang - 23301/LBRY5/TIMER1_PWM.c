/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción:
* Esta librería implementa el control de servomotores mediante señales PWM
* generadas con el Timer1 del microcontrolador AVR. Utiliza modo Fast PWM
* con ICR1 para mayor precisión y control de dos servomotores independientes.

* Conexiones de Hardware:
*   - PWM Timer1 Canal A: PB1 (OC1A) - Servo Brazo2
*   - PWM Timer1 Canal B: PB2 (OC1B) - Servo Pinza
************************************************************************/

#include "TIMER1_PWM.h"

void Timer1_init(void) {
	// Configurar pines como salida
	DDRB |= (1 << DDB1) | (1 << DDB2);  // PB1 (OC1A) y PB2 (OC1B) como salidas
	
	// Configurar Timer1 para PWM modo 14 (Fast PWM con ICR1)
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | // Modo no invertido para OC1A
	(1 << COM1B1) | (0 << COM1B0) | // Modo no invertido para OC1B
	(1 << WGM11) | (0 << WGM10);    // Fast PWM con ICR1
	
	TCCR1B = (1 << WGM13) | (1 << WGM12) |   // Fast PWM con ICR1
	(0 << CS12) | (1 << CS11) | (0 << CS10); // Prescaler 8
	
	// Establecer periodo para Timer1 (20ms para servos -> 50Hz)
	ICR1 = 39999; // 16MHz / 8 / 50Hz - 1 = 39999
	
	// Valores iniciales para Timer1
	OCR1A = 3000;
	OCR1B = 3000;
}

void setPWM1A(uint16_t pwmValue) {
	OCR1A = pwmValue;
}

void setPWM1B(uint16_t pwmValue) {
	OCR1B = pwmValue;
}

uint16_t calculate_PWM1(uint8_t angle) {
	// Para Timer1: Convertir ángulo (0-180) a valor de PWM (2000-4000)
	// 0° = 1ms = 2000, 180° = 2ms = 4000
	if (angle > 180) angle = 180;
	
	uint16_t pwmValue = 2000 + (uint32_t)angle * 2000 / 180;
	return pwmValue;
}

uint16_t calculate_PWM1_inverted(uint8_t angle) {
	// Para Timer1: Convertir ángulo (0-180) a valor de PWM invertido (4000-2000)
	// 0° = 2ms = 4000, 180° = 1ms = 2000
	if (angle > 180) angle = 180;
	
	uint16_t pwmValue = 4000 - (uint32_t)angle * 2000 / 180;
	return pwmValue;
}