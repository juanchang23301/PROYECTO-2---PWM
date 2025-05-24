/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción:
* Esta librería implementa funciones para el manejo de memoria EEPROM del
* microcontrolador AVR. Proporciona funcionalidades para almacenamiento
* persistente de posiciones de la garra robótica, incluyendo operaciones
* de lectura, escritura y gestión de múltiples posiciones guardadas.
*
* Estructura de datos en EEPROM:
* - Dirección 0: Número de posiciones guardadas
* - Direcciones 1+: Posiciones guardadas (4 bytes cada una)
*   - Byte 0: Posición base
*   - Byte 1: Posición brazo1
*   - Byte 2: Posición brazo2
*   - Byte 3: Posición pinza
*
************************************************************************/

#include "EEPROM.h"

// Inicializar la EEPROM
void initEEPROM(void) {
	// La EEPROM en AVR no requiere inicialización específica
	// Pero podemos verificar si es la primera vez que se usa
	if (readEEPROM(DIRECCION_NUM_POSICIONES) == 0xFF) {
		// Si es la primera vez, inicializar el contador de posiciones
		writeEEPROMB(DIRECCION_NUM_POSICIONES, 0);
	}
}

// Escribir un byte en la EEPROM
void writeEEPROMB(uint16_t address, uint8_t dato) {
	// Esperar a que cualquier escritura previa termine
	while(EECR & (1<<EEPE));
	
	// Preparar la dirección y los datos
	EEAR = address;
	EEDR = dato;
	
	// Escribir uno lógico en EEMPE
	EECR |= (1<<EEMPE);
	
	// Iniciar la escritura estableciendo EEPE
	EECR |= (1<<EEPE);
}

// Leer un byte de la EEPROM
uint8_t readEEPROM(uint16_t address) {
	// Esperar a que cualquier escritura previa termine
	while(EECR & (1<<EEPE));
	
	// Preparar la dirección
	EEAR = address;
	
	// Iniciar la lectura estableciendo EERE
	EECR |= (1<<EERE);
	
	// Devolver los datos leídos
	return EEDR;
}

// Guardar una posición completa en la EEPROM
void savePosition(uint8_t positionNum, PosicionGarra posicion) {
	// Calcular la dirección base para esta posición
	uint16_t direccionBase = DIRECCION_BASE_EEPROM + (positionNum * BYTES_POR_POSICION);
	
	// Escribir cada componente de la posición
	writeEEPROMB(direccionBase, posicion.base);
	writeEEPROMB(direccionBase + 1, posicion.brazo1);
	writeEEPROMB(direccionBase + 2, posicion.brazo2);
	writeEEPROMB(direccionBase + 3, posicion.pinza);
	
	// Si estamos guardando en una nueva posición, incrementar el contador
	uint8_t numPosiciones = Saved_Pos_Count();
	if (positionNum >= numPosiciones) {
		increment_Saved_Count();
	}
}

// Cargar una posición desde la EEPROM
PosicionGarra loadPosition(uint8_t positionNum) {
	PosicionGarra posicion;
	
	// Calcular la dirección base para esta posición
	uint16_t direccionBase = DIRECCION_BASE_EEPROM + (positionNum * BYTES_POR_POSICION);
	
	// Leer cada componente de la posición
	posicion.base = readEEPROM(direccionBase);
	posicion.brazo1 = readEEPROM(direccionBase + 1);
	posicion.brazo2 = readEEPROM(direccionBase + 2);
	posicion.pinza = readEEPROM(direccionBase + 3);
	
	return posicion;
}

// Obtener el número de posiciones guardadas
uint8_t Saved_Pos_Count(void) {
	return readEEPROM(DIRECCION_NUM_POSICIONES);
}

// Incrementar el contador de posiciones guardadas
void increment_Saved_Count(void) {
	uint8_t numPosiciones = Saved_Pos_Count();
	if (numPosiciones < MAX_POSICIONES_GUARDADAS) {
		writeEEPROMB(DIRECCION_NUM_POSICIONES, numPosiciones + 1);
	}
}

// Borrar todas las posiciones (reiniciar contador)
void clearAllPositions(void) {
	writeEEPROMB(DIRECCION_NUM_POSICIONES, 0);
}