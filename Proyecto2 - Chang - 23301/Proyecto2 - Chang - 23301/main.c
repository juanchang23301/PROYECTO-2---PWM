/************************************************************************
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* Conexión UART
*
* Autor: Juan René Chang Lam
*
* Descripción del Proyecto:
* Este proyecto implementa un sistema de control para garra robótica con 3 modos de operación:
* - Modo Manual: Control con potenciómetros con posicionamiento en tiempo real
* - Modo UART: Control remoto a través de comandos seriales
* - Modo EEPROM: Guardar/cargar posiciones predefinidas para secuencias automatizadas
*
* Conexiones:
*   Servomotores:
*     - Servo Base: PD5 (OC0B) - Controla rotación horizontal
*     - Servo Brazo1: PD6 (OC0A) - Controla primera articulación del brazo
*     - Servo Brazo2: PB1 (OC1A) - Controla segunda articulación del brazo
*     - Servo Pinza: PB2 (OC1B) - Controla apertura/cierre de pinza
*
*   Potenciómetros (Control Manual):
*     - Control Base: PC0 (ADC0)
*     - Control Brazo1: PC1 (ADC1)
*     - Control Brazo2: PC2 (ADC2)
*     - Control Pinza: PC3 (ADC3)
*
*   Comunicación:
*     - UART: PD0 (RX), PD1 (TX) - Interfaz de comunicación serial
*
*   Interfaz de Usuario:
*     - Botón Cambio Modo: PB0 (con pull-up interno)
*     - LED Modo Manual: PD2
*     - LED Modo UART: PD3
*     - LED Modo EEPROM: PD4
*     - Botón Reproducir Secuencia: PD7 (con pull-up interno)
*     - Botón Guardar Posición: PB3 (con pull-up interno)
*     - LED Indicador Posición Bit0: PC4
*     - LED Indicador Posición Bit1: PC5
************************************************************************/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "LBRY1/ADC.h"
#include "LBRY2/TIMER0_PWM.h"
#include "LBRY3/USART.h"
#include "LBRY4/EEPROM.h"
#include "LBRY5/TIMER1_PWM.h"

// Servos
#define SERVO_BASE 0
#define SERVO_BRAZO1 1
#define SERVO_BRAZO2 2
#define SERVO_PINZA 3

// Modos
#define MENU_MODE 0
#define MANUAL_MODE 1
#define USART_MODE 2
#define EEPROM_MODE 3

// Leds indicadores
#define LED_MANUAL PD2
#define LED_USART PD3
#define LED_EEPROM PD4
#define LED_POS_BIT0 PC4
#define LED_POS_BIT1 PC5

// Push Buttons
#define PUSH_MODE PB0
#define PUSH_SAVE PB3
#define PUSH_PLAY PD7

// Posición inicial
volatile uint8_t posServoBase = 90;
volatile uint8_t posServoBrazo1 = 180;
volatile uint8_t posServoBrazo2 = 90;
volatile uint8_t posServoPinza = 0;

// Modo de operación
volatile uint8_t modoOperacion = 0; // 0: Sin seleccionar, 1: Manual, 2: USART, 3: EEPROM

// Buffer para USART
char bufferRx[20];
uint8_t indiceBuffer = 0;
uint8_t comandoCompleto = 0;

// Estado de los botones
volatile uint8_t estadoBotonAnterior = 1; // Pull-up, por lo que 1 es estado sin presionar
volatile uint8_t estadoBotonReproducirAnterior = 1;
volatile uint8_t estadoBotonGuardarAnterior = 1;
volatile uint8_t flagBotonPresionado = 0;
volatile uint8_t flagBotonReproducirPresionado = 0;
volatile uint8_t flagBotonGuardarPresionado = 0;

// Variables para modo EEPROM
volatile uint8_t posicionActualEEPROM = 0;
volatile uint8_t posicionSiguienteGuardado = 0;
volatile uint8_t ejecutandoSecuencia = 0;

// PROTOTIPO DE FUNCIONES
void initSystem(void);                                          // Initialize system
void updateServos(void);                                        // Update servos
void processCommand(void);                                      // Process command
void showMenu(void);                                           // Show menu
void setServoPosition(uint8_t servo, uint8_t angle);          // Set servo position
void configure_push(void);                                      // Configure buttons
void check_push(void);                                       // Check buttons
void changeOperationMode(void);                                // Change operation mode
void showCurrentMode(void);                                    // Show current mode
void configureLEDs(void);                                      // Configure LEDs
void updateLEDs(void);                                         // Update LEDs
void updatePositionLEDs(uint8_t position);                     // Update position LEDs
void saveCurrentPosition(uint8_t positionNum);                 // Save current position
void loadSavedPosition(uint8_t positionNum);                   // Load saved position
void executeSequence(void);                                    // Execute sequence
void sendAdafruitData(void);                                   // Send data to Adafruit
void playNextPosition(void);                                   // Play next position
void saveNextPosition(void);                                   // Save next position

int main(void) {
	initSystem();
	
	showMenu();
	
	while (1) {
		// Verificar estado de los botones
		check_push();
		
		// Si el botón de modo fue presionado, cambiar modo
		if (flagBotonPresionado) {
			changeOperationMode();
			flagBotonPresionado = 0;
			updateLEDs();
		}
		
		// Si el botón de reproducir fue presionado en modo EEPROM
		if (flagBotonReproducirPresionado && modoOperacion == EEPROM_MODE) {
			playNextPosition();
			flagBotonReproducirPresionado = 0;
		}
		
		// Si el botón de guardar fue presionado en modo Manual o USART
		if (flagBotonGuardarPresionado && (modoOperacion == MANUAL_MODE || modoOperacion == USART_MODE)) {
			saveNextPosition();
			flagBotonGuardarPresionado = 0;
		}
		
		// Si estamos en modo control por potenciómetros
		if (modoOperacion == MANUAL_MODE) {
			// Usar lecturas filtradas para reducir el ruido
			posServoBase = ADC_Angulo(ADC_read_Filtr(0, 5));
			posServoBrazo1 = ADC_Angulo(ADC_read_Filtr(1, 5));
			posServoBrazo2 = ADC_Angulo(ADC_read_Filtr(2, 5));
			// Invertir el rango para la pinza
			posServoPinza = 180 - ADC_Angulo(ADC_read_Filtr(3, 5));
			
			// Mantener actualización no muy frecuente para reducir jitter
			static uint8_t contador = 0;
			contador++;
			if (contador >= 3) {
				updateServos();
				contador = 0;
			}
			
			_delay_ms(30); // Retraso para estabilidad
		}
		// Si estamos en modo EEPROM y ejecutando secuencia
		else if (modoOperacion == EEPROM_MODE && ejecutandoSecuencia) {
			executeSequence();
		}
		
		// Si se recibió un comando completo por USART
		if (comandoCompleto) {
			processCommand();
			comandoCompleto = 0;
			indiceBuffer = 0;
		}
	}
	
	return 0;
}

void initSystem(void) {
	// Inicializar PWM para servos (Timer0 y Timer1 separados)
	Timer0_init();
	Timer1_init();
	
	// Inicializar ADC para potenciómetros
	ADC_init();
	
	// Inicializar USART para comunicación
	initUSART();
	
	// Inicializar EEPROM
	initEEPROM();
	
	// Configurar botones
	configure_push();
	
	// Configurar LEDs de modo y posición
	configureLEDs();
	
	// Posiciones iniciales de los servos
	setPWM0A(calculate_PWM0(posServoBrazo1)); // Brazo1 (Timer0)
	setPWM0B(calculate_PWM0(posServoBase));   // Base (Timer0)
	setPWM1A(calculate_PWM1(posServoBrazo2)); // Brazo2 (Timer1)
	setPWM1B(calculate_PWM1(posServoPinza));  // Pinza (Timer1)
	
	// Obtener el número de posiciones guardadas para inicializar el contador de siguiente posición
	posicionSiguienteGuardado = Saved_Pos_Count();
	
	// Habilitar interrupciones globales
	sei();
}

void configure_push(void) {
	// Configurar PB0 como entrada con pull-up interno (botón de modo)
	DDRB &= ~(1 << PUSH_MODE);
	PORTB |= (1 << PUSH_MODE);
	
	// Configurar PB3 como entrada con pull-up interno (botón de guardar)
	DDRB &= ~(1 << PUSH_SAVE);
	PORTB |= (1 << PUSH_SAVE);
	
	// Configurar PD7 como entrada con pull-up interno (botón de reproducir)
	DDRD &= ~(1 << PUSH_PLAY);
	PORTD |= (1 << PUSH_PLAY);
}

void configureLEDs(void) {
	// Configurar PD2, PD3 y PD4 como salidas (LEDs de modo)
	DDRD |= (1 << LED_MANUAL) | (1 << LED_USART) | (1 << LED_EEPROM);
	
	// Configurar PC4 y PC5 como salidas (LEDs indicadores de posición)
	DDRC |= (1 << LED_POS_BIT0) | (1 << LED_POS_BIT1);
	
	// Apagar todos los LEDs inicialmente
	PORTD &= ~((1 << LED_MANUAL) | (1 << LED_USART) | (1 << LED_EEPROM));
	PORTC &= ~((1 << LED_POS_BIT0) | (1 << LED_POS_BIT1));
}

void updateLEDs(void) {
	// Apagar todos los LEDs de modo primero
	PORTD &= ~((1 << LED_MANUAL) | (1 << LED_USART) | (1 << LED_EEPROM));
	
	// Encender el LED correspondiente al modo actual
	switch (modoOperacion) {
		case MANUAL_MODE:
		PORTD |= (1 << LED_MANUAL);
		break;
		case USART_MODE:
		PORTD |= (1 << LED_USART);
		break;
		case EEPROM_MODE:
		PORTD |= (1 << LED_EEPROM);
		break;
		default:
		break;
	}
}

void updatePositionLEDs(uint8_t posicion) {
	// Apagar los LEDs de posición
	PORTC &= ~((1 << LED_POS_BIT0) | (1 << LED_POS_BIT1));
	
	// Mostrar la posición en binario (bits 0 y 1)
	if (posicion & 0x01) PORTC |= (1 << LED_POS_BIT0);
	if (posicion & 0x02) PORTC |= (1 << LED_POS_BIT1);
}

void check_push(void) {
	// Leer estado actual de los botones
	uint8_t estadoBotonActual = (PINB & (1 << PUSH_MODE)) ? 1 : 0;
	uint8_t estadoBotonReproducirActual = (PIND & (1 << PUSH_PLAY)) ? 1 : 0;
	uint8_t estadoBotonGuardarActual = (PINB & (1 << PUSH_SAVE)) ? 1 : 0;
	
	// Detectar flanco descendente del botón de modo (botón presionado)
	if (estadoBotonAnterior == 1 && estadoBotonActual == 0) {
		_delay_ms(50);
		
		// Verificar si sigue presionado
		if ((PINB & (1 << PUSH_MODE)) == 0) {
			flagBotonPresionado = 1;
		}
	}
	
	// Detectar flanco descendente del botón de reproducir (botón presionado)
	if (estadoBotonReproducirAnterior == 1 && estadoBotonReproducirActual == 0) {
		_delay_ms(50);

		// Verificar si sigue presionado
		if ((PIND & (1 << PUSH_PLAY)) == 0) {
			flagBotonReproducirPresionado = 1;
		}
	}
	
	// Detectar flanco descendente del botón de guardar (botón presionado)
	if (estadoBotonGuardarAnterior == 1 && estadoBotonGuardarActual == 0) {
		_delay_ms(50);
		
		// Verificar si sigue presionado
		if ((PINB & (1 << PUSH_SAVE)) == 0) {
			flagBotonGuardarPresionado = 1;
		}
	}
	
	// Actualizar estados anteriores
	estadoBotonAnterior = estadoBotonActual;
	estadoBotonReproducirAnterior = estadoBotonReproducirActual;
	estadoBotonGuardarAnterior = estadoBotonGuardarActual;
}

void changeOperationMode(void) {
	// Ciclar entre modos
	modoOperacion = (modoOperacion + 1) % 4;
	if (modoOperacion == MENU_MODE) {
		modoOperacion = MANUAL_MODE;
	}
	
	// Si estábamos ejecutando una secuencia, detenerla
	ejecutandoSecuencia = 0;
	
	// Apagar los LEDs de posición al cambiar de modo
	PORTC &= ~((1 << LED_POS_BIT0) | (1 << LED_POS_BIT1));
	
	// Mostrar modo actual en terminal
	showCurrentMode();
}

void showCurrentMode(void) {
	switch (modoOperacion) {
		case MANUAL_MODE:
		sendUSARTString("\r\n[BOTON] Modo de control por potenciometros activado\r\n");
		sendUSARTString("Presiona el boton en PB3 para guardar la posicion actual en EEPROM\r\n");
		sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
		break;
		case USART_MODE:
		sendUSARTString("\r\n[BOTON] Modo de control por USART activado\r\n");
		sendUSARTString("Formato: S,base,brazo1,brazo2,pinza\r\n");
		sendUSARTString("Presiona el boton en PB3 para guardar la posicion actual en EEPROM\r\n");
		sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
		break;
		case EEPROM_MODE:
		sendUSARTString("\r\n[BOTON] Modo EEPROM activado\r\n");
		sendUSARTString("Comandos disponibles:\r\n");
		sendUSARTString("G,n - Guardar posicion actual en la posicion n\r\n");
		sendUSARTString("C,n - Cargar posicion n\r\n");
		sendUSARTString("E - Ejecutar secuencia de posiciones guardadas\r\n");
		sendUSARTString("B - Borrar todas las posiciones guardadas\r\n");
		sendUSARTString("L - Listar posiciones guardadas\r\n");
		sendUSARTString("Presiona el boton en PD7 para reproducir las posiciones guardadas una por una\r\n");
		sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
		break;
		default:
		break;
	}
}

void updateServos(void) {
	// Usar los valores de PWM específicos para cada timer
	setPWM0B(calculate_PWM0(posServoBase));      // Base (Timer0)
	setPWM0A(calculate_PWM0(posServoBrazo1));    // Brazo1 (Timer0)
	setPWM1A(calculate_PWM1(posServoBrazo2));    // Brazo2 (Timer1)
	setPWM1B(calculate_PWM1_inverted(posServoPinza)/3);  // Pinza invertida (Timer1)
}

void showMenu(void) {
	sendUSARTString("\r\n--- PROYECTO #2 IE2023 - GARRA ROBOTICA ---\r\n");
	sendUSARTString("Presione el numero de la opcion que quiere realizar:\r\n");
	sendUSARTString("1. Controlar garra con Pots (Modo Manual)\r\n");
	sendUSARTString("2. Establecer una nueva posicion por USART\r\n");
	sendUSARTString("3. Modo EEPROM (guardar/cargar posiciones)\r\n");
	sendUSARTString("También puede presionar el botón conectado a PB0 para cambiar de modo\r\n");
	sendUSARTString("Ingrese opcion: ");
}

void processCommand(void) {
	// Si el usuario quiere volver al menú principal desde cualquier modo
	if (strcmp(bufferRx, "menu") == 0) {
		modoOperacion = MENU_MODE;
		showMenu();
		updateLEDs();
		return; // Salir de la función después de procesar "menu"
	}

	// Si no hay modo seleccionado, interpretar como selección de modo
	if (modoOperacion == MENU_MODE) {
		if (bufferRx[0] == '1') {
			modoOperacion = MANUAL_MODE;
			sendUSARTString("\r\nModo de control por potenciometros activado\r\n");
			sendUSARTString("Presiona el boton en PB3 para guardar la posicion actual en EEPROM\r\n");
			sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
			updateLEDs();
			} else if (bufferRx[0] == '2') {
			modoOperacion = USART_MODE;
			sendUSARTString("\r\nModo de control por USART activado\r\n");
			sendUSARTString("Formato: S,base,brazo1,brazo2,pinza\r\n");
			sendUSARTString("Ejemplo: S,90,45,120,30\r\n");
			sendUSARTString("Presiona el boton en PB3 para guardar la posicion actual en EEPROM\r\n");
			sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
			updateLEDs();
			} else if (bufferRx[0] == '3') {
			modoOperacion = EEPROM_MODE;
			sendUSARTString("\r\nModo EEPROM activado\r\n");
			sendUSARTString("Comandos disponibles:\r\n");
			sendUSARTString("G,n - Guardar posicion actual en la posicion n\r\n");
			sendUSARTString("C,n - Cargar posicion n\r\n");
			sendUSARTString("E - Ejecutar secuencia de posiciones guardadas\r\n");
			sendUSARTString("B - Borrar todas las posiciones guardadas\r\n");
			sendUSARTString("L - Listar posiciones guardadas\r\n");
			sendUSARTString("Presiona el boton en PD7 para reproducir las posiciones guardadas una por una\r\n");
			sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
			updateLEDs();
			} else {
			sendUSARTString("\r\nOpcion no valida. Intente de nuevo\r\n");
			showMenu();
		}
	}
	// Si estamos en modo USART, procesar comandos de posición
	else if (modoOperacion == USART_MODE) {
		// Verificar si es un comando de posición (formato: S,base,brazo1,brazo2,pinza)
		if (bufferRx[0] == 'S' && bufferRx[1] == ',') {
			// Extraer valores usando strtok
			char* token = strtok(bufferRx, ",");
			token = strtok(NULL, ","); // Obtener valor de base
			if (token != NULL) {
				posServoBase = atoi(token);
				token = strtok(NULL, ","); // Obtener valor de brazo1
				if (token != NULL) {
					posServoBrazo1 = atoi(token);
					token = strtok(NULL, ","); // Obtener valor de brazo2
					if (token != NULL) {
						posServoBrazo2 = atoi(token);
						token = strtok(NULL, ","); // Obtener valor de pinza
						if (token != NULL) {
							posServoPinza = atoi(token);
							
							// Actualizar posiciones de servos
							updateServos();
							sendUSARTString("\r\nPosicion actualizada\r\n");
						}
					}
				}
			}
		}
		else {
			sendUSARTString("\r\nComando no valido\r\n");
			sendUSARTString("Formato: S,base,brazo1,brazo2,pinza\r\n");
			sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
		}
	}
	// Si estamos en modo EEPROM, procesar comandos de EEPROM
	else if (modoOperacion == EEPROM_MODE) {
		// Guardar posición actual (G,n)
		if (bufferRx[0] == 'G' && bufferRx[1] == ',') {
			uint8_t positionNum = atoi(bufferRx + 2);
			if (positionNum < MAX_POSICIONES_GUARDADAS) {
				saveCurrentPosition(positionNum);
				// Actualizar el contador de posición siguiente si es necesario
				if (positionNum >= posicionSiguienteGuardado) {
					posicionSiguienteGuardado = positionNum + 1;
				}
				char mensaje[50];
				sprintf(mensaje, "\r\nPosicion guardada en la ranura %d\r\n", positionNum);
				sendUSARTString(mensaje);
				} else {
				sendUSARTString("\r\nNumero de posicion invalido\r\n");
			}
		}
		// Cargar posición guardada (C,n)
		else if (bufferRx[0] == 'C' && bufferRx[1] == ',') {
			uint8_t positionNum = atoi(bufferRx + 2);
			if (positionNum < Saved_Pos_Count()) {
				loadSavedPosition(positionNum);
				char mensaje[50];
				sprintf(mensaje, "\r\nPosicion %d cargada\r\n", positionNum);
				sendUSARTString(mensaje);
				
				// Actualizar LEDs para mostrar la posición actual
				updatePositionLEDs(positionNum);
				} else {
				sendUSARTString("\r\nNumero de posicion invalido o no guardada\r\n");
			}
		}
		// Ejecutar secuencia (E)
		else if (bufferRx[0] == 'E') {
			if (Saved_Pos_Count() > 0) {
				ejecutandoSecuencia = 1;
				posicionActualEEPROM = 0;
				sendUSARTString("\r\nEjecutando secuencia de posiciones guardadas\r\n");
				} else {
				sendUSARTString("\r\nNo hay posiciones guardadas para ejecutar\r\n");
			}
		}
		// Borrar todas las posiciones (B)
		else if (bufferRx[0] == 'B') {
			clearAllPositions();
			posicionSiguienteGuardado = 0;  // Reiniciar el contador de posición siguiente
			sendUSARTString("\r\nTodas las posiciones han sido borradas\r\n");
			// Apagar los LEDs de posición
			PORTC &= ~((1 << LED_POS_BIT0) | (1 << LED_POS_BIT1));
		}
		// Listar posiciones guardadas (L)
		else if (bufferRx[0] == 'L') {
			uint8_t numPosiciones = Saved_Pos_Count();
			char mensaje[50];
			sprintf(mensaje, "\r\nPosiciones guardadas: %d\r\n", numPosiciones);
			sendUSARTString(mensaje);
			for (uint8_t i = 0; i < numPosiciones; i++) {
				PosicionGarra pos = loadPosition(i);
				sprintf(mensaje, "Pos %d: Base=%d, Brazo1=%d, Brazo2=%d, Pinza=%d\r\n",
				i, pos.base, pos.brazo1, pos.brazo2, pos.pinza);
				sendUSARTString(mensaje);
			}
		}
		else {
			sendUSARTString("\r\nComando no valido\r\n");
			sendUSARTString("Comandos disponibles:\r\n");
			sendUSARTString("G,n - Guardar posicion actual en la posicion n\r\n");
			sendUSARTString("C,n - Cargar posicion n\r\n");
			sendUSARTString("E - Ejecutar secuencia de posiciones guardadas\r\n");
			sendUSARTString("B - Borrar todas las posiciones guardadas\r\n");
			sendUSARTString("L - Listar posiciones guardadas\r\n");
			sendUSARTString("Presiona el boton en PD7 para reproducir las posiciones guardadas una por una\r\n");
			sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
		}
	}
	// Si estamos en modo potenciómetros, permitir algunos comandos especiales
	else if (modoOperacion == MANUAL_MODE) {
		// Puedes agregar más comandos especiales para el modo manual si es necesario
		sendUSARTString("\r\nEn modo de control por potenciometros\r\n");
		sendUSARTString("Presiona el boton en PB3 para guardar la posicion actual en EEPROM\r\n");
		sendUSARTString("Escribe 'menu' para volver al menu principal\r\n");
	}
}

void setServoPosition(uint8_t servo, uint8_t angle) {
	switch (servo) {
		case SERVO_BASE:
		posServoBase = angle;
		setPWM0B(calculate_PWM0(angle));
		break;
		case SERVO_BRAZO1:
		posServoBrazo1 = angle;
		setPWM0A(calculate_PWM0(angle));
		break;
		case SERVO_BRAZO2:
		posServoBrazo2 = angle;
		setPWM1A(calculate_PWM1(angle));
		break;
		case SERVO_PINZA:
		posServoPinza = angle;
		setPWM1B(calculate_PWM1_inverted(angle)); // Usar función invertida
		break;
		default:
		break;
	}
}

void saveCurrentPosition(uint8_t positionNum) {
	PosicionGarra posicion;
	posicion.base = posServoBase;
	posicion.brazo1 = posServoBrazo1;
	posicion.brazo2 = posServoBrazo2;
	posicion.pinza = posServoPinza;
	
	savePosition(positionNum, posicion);
}

void loadSavedPosition(uint8_t positionNum) {
	PosicionGarra posicion = loadPosition(positionNum);
	
	posServoBase = posicion.base;
	posServoBrazo1 = posicion.brazo1;
	posServoBrazo2 = posicion.brazo2;
	posServoPinza = posicion.pinza;
	
	updateServos();
}

void executeSequence(void) {
	uint8_t numPosiciones = Saved_Pos_Count();
	
	if (numPosiciones == 0 || posicionActualEEPROM >= numPosiciones) {
		ejecutandoSecuencia = 0;
		sendUSARTString("\r\nSecuencia completada\r\n");
		return;
	}
	
	// Cargar y aplicar la posición actual
	loadSavedPosition(posicionActualEEPROM);
	
	// Actualizar LEDs para mostrar la posición actual
	updatePositionLEDs(posicionActualEEPROM);
	
	// Enviar información a terminal
	char mensaje[50];
	sprintf(mensaje, "\r\nEjecutando posicion %d de %d\r\n", posicionActualEEPROM + 1, numPosiciones);
	sendUSARTString(mensaje);
	
	// Avanzar a la siguiente posición
	posicionActualEEPROM++;
	
	// Esperar un tiempo entre posiciones
	_delay_ms(1000);
	
	// Si hemos llegado al final, detener la secuencia
	if (posicionActualEEPROM >= numPosiciones) {
		ejecutandoSecuencia = 0;
		sendUSARTString("\r\nSecuencia completada\r\n");
	}
}

// Función para reproducir la siguiente posición guardada al presionar el botón
void playNextPosition(void) {
	uint8_t numPosiciones = Saved_Pos_Count();
	
	// Verificar si hay posiciones guardadas
	if (numPosiciones == 0) {
		sendUSARTString("\r\nNo hay posiciones guardadas para reproducir\r\n");
		return;
	}
	
	// Si ya hemos llegado al final, volver al inicio
	if (posicionActualEEPROM >= numPosiciones) {
		posicionActualEEPROM = 0;
	}
	
	// Cargar y aplicar la posición actual
	loadSavedPosition(posicionActualEEPROM);
	
	// Actualizar LEDs para mostrar la posición actual
	updatePositionLEDs(posicionActualEEPROM);
	
	// Enviar información a terminal
	char mensaje[50];
	sprintf(mensaje, "\r\n[BOTON REPROD] Reproduciendo posicion %d de %d\r\n", posicionActualEEPROM + 1, numPosiciones);
	sendUSARTString(mensaje);
	
	// Avanzar a la siguiente posición para la próxima vez
	posicionActualEEPROM++;
}

// Función para guardar la posición actual en la siguiente ranura disponible
void saveNextPosition(void) {
	// Verificar si hay espacio disponible
	if (posicionSiguienteGuardado >= MAX_POSICIONES_GUARDADAS) {
		sendUSARTString("\r\n[BOTON GUARD] Error: Memoria EEPROM llena\r\n");
		return;
	}
	
	// Guardar la posición actual en la siguiente ranura disponible
	saveCurrentPosition(posicionSiguienteGuardado);
	
	// Enviar información a terminal
	char mensaje[50];
	sprintf(mensaje, "\r\n[BOTON GUARD] Posicion guardada en la ranura %d\r\n", posicionSiguienteGuardado);
	sendUSARTString(mensaje);
	
	// Incrementar el contador para la próxima vez
	posicionSiguienteGuardado++;
}

void sendAdafruitData(void) {
	// Crear y enviar una cadena con los valores actuales para que se puedan recibir desde Python
	char mensaje[50];
	sprintf(mensaje, "P,%d,%d,%d,%d\r\n", posServoBase, posServoBrazo1, posServoBrazo2, posServoPinza);
	sendUSARTString(mensaje);
}

// INTERRUPCIONES
ISR(USART_RX_vect) {
	char datoRx = UDR0;
	
	// Si es un enter, marcar como comando completo
	if (datoRx == '\r' || datoRx == '\n') {
		if (indiceBuffer > 0) {
			bufferRx[indiceBuffer] = '\0'; // Terminar string
			comandoCompleto = 1;
		}
	}
	// Si no, agregar al buffer
	else if (indiceBuffer < 19) {
		bufferRx[indiceBuffer++] = datoRx;
		sendUSARTData(datoRx); // Echo
	}
}