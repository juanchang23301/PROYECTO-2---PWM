/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción:
* Esta librería implementa el control de servomotores mediante señales PWM
* generadas con el Timer0 del microcontrolador AVR. Proporciona funciones
* para configuración del timer, control de posición angular y cálculo
* automático de valores PWM para dos servomotores independientes.
*

* Aplicación específica:
* - Canal A (OC0A): Control de Servo Brazo1
* - Canal B (OC0B): Control de Servo Base
* - Rango de movimiento: 0-180 grados
* - Resolución: ~1.4 grados por paso (8-bit PWM)
*
* Conexiones de Hardware:
*   - PWM Timer0 Canal A: PD6 (OC0A) - Servo Brazo1
*   - PWM Timer0 Canal B: PD5 (OC0B) - Servo Base
************************************************************************/

#include "TIMER0_PWM.h"

void Timer0_init(void) {
	// Configurar pines como salida
	DDRD |= (1 << SERVO_PIN_OC0A) | (1 << SERVO_PIN_OC0B);  // PD6 (OC0A) y PD5 (OC0B) como salidas
	
	// Configurar Timer0 para PWM rápido, modo de comparación no invertido
	TCCR0A = (1 << WGM01) | (1 << WGM00);  // Modo Fast PWM
	
	// Configurar OC0A y OC0B en modo no invertido
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
	
	// Configurar prescaler a 1024 para Timer0
	TCCR0B = (1 << CS02) | (1 << CS00);
	
	// Valor inicial para los servos en Timer0 (posición 0 grados)
	OCR0A = SERVO_MIN_T0;
	OCR0B = SERVO_MIN_T0;
}

void set_pos0A(uint8_t pos) {
	// Limitamos pos a 0-180
	if (pos > 180) pos = 180;
	
	// Mapear 0-180
	uint8_t duty = SERVO_MIN_T0 + (((SERVO_MAX_T0 - SERVO_MIN_T0) * (uint16_t)pos) / 80);
	
	// Establecer el valor de comparación
	OCR0A = duty;
}

void set_pos0B(uint8_t pos) {
	// Limitamos pos a 0-180
	if (pos > 180) pos = 180;
	
	// Mapear 0-180
	uint8_t duty = SERVO_MIN_T0 + (((SERVO_MAX_T0 - SERVO_MIN_T0) * (uint16_t)pos) / 180);
	
	// Establecer el valor de comparación
	OCR0B = duty;
}

void setPWM0A(uint8_t pwmValue) {
	OCR0A = pwmValue;
}

void setPWM0B(uint8_t pwmValue) {
	OCR0B = pwmValue;
}

uint8_t calculate_PWM0(uint8_t angle) {
	// Limitamos angulo a 0-180
	if (angle > 180) angle = 180;
	
	// Mapear 0-180 a SERVO_MIN_T0-SERVO_MAX_T0
	uint8_t pwmValue = SERVO_MIN_T0 + (((SERVO_MAX_T0 - SERVO_MIN_T0) * (uint16_t)angle) / 180);
	
	return pwmValue;
}