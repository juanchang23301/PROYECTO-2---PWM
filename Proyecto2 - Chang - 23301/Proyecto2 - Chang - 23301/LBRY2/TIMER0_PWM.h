/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción: 
* Este archivo define la interfaz pública para el control de servomotores
* mediante PWM generado por Timer0. Proporciona constantes de configuración,
* definiciones de pines y declaraciones de funciones para el manejo
* de dos canales independientes de control servo.
*
* Conexiones de Hardware: 
*   - PWM Timer0 Canal A: PD6 (OC0A) - Control Servo Brazo1
*   - PWM Timer0 Canal B: PD5 (OC0B) - Control Servo Base
************************************************************************/

#ifndef TIMER0_PWM_H
#define TIMER0_PWM_H
#include <avr/io.h>
#include <stdint.h>

#define SERVO_PIN_OC0A PIND6  // Pin 6 del puerto D
#define SERVO_PIN_OC0B PIND5  // Pin 5 del puerto D

#define SERVO_MIN_T0 15   // Posicion 0 grados
#define SERVO_MAX_T0 37   // Posicion 180 grados

void Timer0_init(void);

void set_pos0A(uint8_t pos);				// Establecer posición del servo en OC0A (0-180)
void set_pos0B(uint8_t pos);           // Set servo position on OC0B (0-180)
void setPWM0A(uint8_t pwmValue);                // Set direct PWM value on OC0A
void setPWM0B(uint8_t pwmValue);                // Set direct PWM value on OC0B
uint8_t calculate_PWM0(uint8_t angle);      // Calculate PWM value for servo on Timer0

#endif // TIMER0_PWM_H