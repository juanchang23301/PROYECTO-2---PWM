/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programaci�n de Microcontroladores
* Conexi�n UART
*
* Autor: Juan Ren� Chang Lam
*
* Descripci�n: 
* Este archivo define la interfaz p�blica para el m�dulo de manejo de
* EEPROM. Proporciona estructuras de datos, constantes de configuraci�n
* y declaraciones de funciones para almacenamiento persistente de
* posiciones de garra rob�tica y gesti�n de memoria no vol�til.

************************************************************************/

#ifndef EEPROM_H
#define EEPROM_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Estructura para almacenar posiciones de servos
typedef struct {
	uint8_t base;
	uint8_t brazo1;
	uint8_t brazo2;
	uint8_t pinza;
} PosicionGarra;


#define MAX_POSICIONES_GUARDADAS 10
#define BYTES_POR_POSICION 4
#define DIRECCION_BASE_EEPROM 0
#define DIRECCION_NUM_POSICIONES (MAX_POSICIONES_GUARDADAS * BYTES_POR_POSICION)

void initEEPROM(void);                                          // Initialize EEPROM
void writeEEPROMB(uint16_t address, uint8_t dato);          // Write byte to EEPROM
uint8_t readEEPROM(uint16_t address);                      // Read byte from EEPROM
void savePosition(uint8_t positionNum, PosicionGarra posicion); // Save gripper position
PosicionGarra loadPosition(uint8_t positionNum);               // Load gripper position
uint8_t Saved_Pos_Count(void);                          // Get number of saved positions
void increment_Saved_Count(void);                       // Increment saved positions counter
void clearAllPositions(void);                                  // Clear all saved positions

#endif /* EEPROM_H */