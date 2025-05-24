/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción: 
* Este archivo define la interfaz pública para el control de servomotores
* mediante PWM generado por Timer1. Proporciona declaraciones de funciones
* para el manejo de dos canales independientes con mayor precisión que
* Timer0, ideal para servos que requieren control angular fino.
*
* Conexiones de Hardware: 
*   - PWM Timer1 Canal A: PB1 (OC1A) - Control Servo Brazo2
*   - PWM Timer1 Canal B: PB2 (OC1B) - Control Servo Pinza
************************************************************************/
#ifndef TIMER1_PWM_H
#define TIMER1_PWM_H
#include <avr/io.h>
#include <stdint.h>


void Timer1_init(void);                                       // Initialize Timer1 PWM
void setPWM1A(uint16_t pwmValue);                              // Set PWM value on OC1A
void setPWM1B(uint16_t pwmValue);                              // Set PWM value on OC1B
uint16_t calculate_PWM1(uint8_t angle);                   // Calculate PWM value for servo on Timer1
uint16_t calculate_PWM1_inverted(uint8_t angle);           // Calculate inverted PWM value for servo on Timer1

#endif // TIMER1_PWM_H