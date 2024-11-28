/*
 * Experiencia2.c
 *
 * Created: 11/10/2024 11:09:00
 * Author : Ramiro Uffelmann
 */ 

// Definiciones -----------------------------------------------------------------------------------------
#define F_CPU 16000000UL		// 16 MHz Frecuencia del cristal.

// Pines usados por la librería lcd_328.h:
#define RS	eS_PORTB0			// Pin RS = PB0 (8) (Reset).
#define EN	eS_PORTB1			// Pin EN = PB1 (9) (Enable).
#define D4	eS_PORTB2			// Pin D4 = PB2 (10) (Data D4).
#define D5	eS_PORTB3			// Pin D5 = PB3 (11) (Data D5).
#define D6	eS_PORTB4			// Pin D6 = PB4 (12) (Data D6).
#define D7	eS_PORTB5			// Pin D7 = PB5 (13) (Data D7).

// Pines usados para los botones, el motor y la alarma sonora -------------------------------------------
#define P1 PD2					// Botón de marcha/parada en pin de INT0.
#define P2 PC2					// Botón de ajuste de velocidad.
#define P3 PC3					// Botón de ajuste de puesto.
#define Motor PD1				// Salida para encender motor de la cinta transportadora.
#define Alarma PD0				// Salida para encender la alarma sonora.

// Macros y constantes ----------------------------------------------------------------------------------
#define	sbi(p,b)		p |= _BV(b)				//	Para setear el bit b de p.
#define	cbi(p,b)		p &= ~(_BV(b))			//	Para borrar el bit b de p.
#define	tbi(p,b)		p ^= _BV(b)				//	Para togglear el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	Para testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	Para testear si el bit b de p es 0.

// Inclusión de archivos --------------------------------------------------------------------------------
#include <stdio.h>				// Cabecera estándar de E/S.
#include <stdlib.h>				// Cabecera de la biblioteca estándar de propósito general.
#include <avr/io.h>				// Contiene definiciones estándares (puertos, memorias, etc.)
#include <util/delay.h>			// Contiene macros para generar retardos.
#include <avr/interrupt.h>		// Contiene macros para uso de interrupciones.
#include "lcd_328.h"			// Contiene funciones para manejo del LCD.
#include <string.h>

// Variables globales -----------------------------------------------------------------------------------
char buffer[16];				// Vector de caracteres que almacena string (16 = Nº de filas del LCD).

int P2_act = 1, P2_ant = 1, flag_P2 = 1;
int P3_act = 1, P3_ant = 1, flag_P3 = 1;
int encenderCinta = 0;
int dispararAlarma = 0;
volatile unsigned int tiempoTimer2 = 0;

const int velocidad[3] = {54, 66, 72};				// Velocidad en m/min.
const int puesto[3] = {10, 20, 35};					// Distancia del origen a cada puesto en m.
int tiempos [3][3] = {	{11111, 22222, 38888},
						{9090, 18181, 31818}, 
						{8333, 16666, 29166}};
unsigned int v = 0;		// Velocidad seleccionada.
unsigned int p = 0;		// Puesto seleccionado.
unsigned int momentoPulsadoP1 = 0;		// Para eliminar efecto rebote.
unsigned int tiempoEncendido = 0;		// Tiempo que lleva encendido el motor o la alarma.
int tiempoRestante = 0;					// Tiempo que falta para se apague el motor o la alarma.
int tiempoRestantePrevio = 0;			// Auxiliar, para mostrar cuenta regresiva en display cada 1 segundo solamente.
//------------------------------------
void activarTimer0();
void desactivarTimer0();
void activarTimer2();
void desactivarTimer2();
void activarTimer1();
void desactivarTimer1();

void intTOstring(int, char*);	// Función que convierte N° entero en un string.
void flotTOstring(float, char*);// Función que convierte N° flotante en un string.
// ------------------------------------------------------------------------------------------
ISR (INT0_vect) {
	// Deshabilita la interrupción INT0 hasta que se lea el botón
	EIMSK &= ~(1<<INT0);				// Deshabilita interrupción externa INT0.
	activarTimer0();					// Inicia un temporizador corto (antirrebote de 10ms).
}

ISR (TIMER0_OVF_vect) {
	// Se llama tras el overflow de TIMER0 (cada 10ms)
	if (is_low(PIND, P1)) {				// Verifica si el botón sigue presionado
		// Cambia el estado de la cinta
		if (encenderCinta == 1) {
			encenderCinta = 0;  // Apaga el motor
			desactivarTimer1();
		} else {
			encenderCinta = 1;  // Enciende el motor
			activarTimer1();     // Reanuda el temporizador si no ha llegado a destino
		}
	}
	// Detener temporizador TIMER0 y restablecer INT0 después del tiempo de antirrebote
	desactivarTimer0();
	momentoPulsadoP1 = tiempoTimer2;	// Guarda el momento en que fue pulsado el botón
	EIMSK |= (1<<INT0);				// Reactiva la interrupción INT0.
	TCNT0 = 6;						// Reinicia contador de TIMER0
}

ISR (TIMER2_OVF_vect) {
	// Temporizador principal de 10ms
	tiempoTimer2 += 10;
	
	if(tiempoTimer2 - momentoPulsadoP1 >= 1000) {
		EIMSK |= (1<<INT0);			// Vuelvo a habilitar interrup. externa INT0 cada 1 segundo.
	}
	
	// Cada 10ms, verificar P2 y P3 (ajustes de velocidad y puesto)
	if (!encenderCinta) {				// Sólo se configuran mientras la cinta está apagada
		// Verificación de P2 (velocidad)
		P2_act = is_high(PINC, P2);
		if(P2_ant && P2_act) flag_P2 = 1;
		if(!P2_ant && !P2_act && flag_P2) {
			v = (v + 1) % 3;			// Cambia velocidad en ciclo V1 -> V2 -> V3 -> V1
			flag_P2 = 0;
		}
		P2_ant = P2_act;

		// Verificación de P3 (puesto)
		P3_act = is_high(PINC, P3);
		if(P3_ant && P3_act) flag_P3 = 1;
		if(!P3_ant && !P3_act && flag_P3) {
			p = (p + 1) % 3;			// Cambia puesto en ciclo P1 -> P2 -> P3 -> P1
			flag_P3 = 0;
		}
		P3_ant = P3_act;
	}
	
	TCNT2 = 6;						// Reinicia contador de TIMER2
}

ISR (TIMER1_OVF_vect) {
	// Entra acá cada 10 ms.
	tiempoEncendido += 10;  // Aumenta el tiempo en 10 ms
	TCNT1 = 45535;
}
// ------------------------------------------------------------------------------------------
int main(void) {
//     for (int i = 0; i<=2; i++) {
// 	    for (int j = 0; j<=2; j++) {
// 		    // t = d/v
// 		    tiempos[i][j] = puesto[i]  * 60 * 1000 / velocidad[j];	// Tiempo en ms
// 	    }
//     }
	// ------------------------------------------------------------------------------------------
	EICRA = (0<<ISC00)|(1<<ISC01);	// Configura interr. INT0 sensible a flanco descendente.
	EIMSK = (1<<INT0);				// Habilita interr. externa INT0.
	EIFR = 0x00;					// Borra flag INTF0 p/evitar alguna interr-. espúrea.
	sei();							// Habilita las interrup. globalmente.
	// ------------------------------------------------------------------------------------------
	DDRB = 0b11111111;				// Puerto B todo como salida, para conectar display.
	DDRC = 0b110011;				// PC2 y PC3 como entrada.
	DDRD = 0b11111011;				// Sólo PD2 (INT0) como entrada, para P1.
	activarTimer2();
	sbi(PORTB, PB1);
	Lcd4_Init();				// Inicializa el LCD (siempre debe estar antes de usar el LCD).
	Lcd4_Clear();				// Borra el display.
    while (1) {
		if(encenderCinta && tiempoEncendido < tiempos[v][p]) {
			sbi(PORTD, Motor);	// Encender si se debe encender.
			dispararAlarma = 0;
 			tiempoRestante = (tiempos[v][p] - tiempoEncendido) / 1000;	// En segundos.
 			if(tiempoRestante != tiempoRestantePrevio) {
				tiempoRestantePrevio = tiempoRestante;
				Lcd4_Set_Cursor(1,0);
				sprintf(buffer, "Falta: %d seg.  ", tiempoRestante);
				Lcd4_Write_String(buffer);
			}
			
		} else if(encenderCinta && tiempoEncendido >= tiempos[v][p]) { // Si llegó a destino.
			cbi(PORTD, Motor);					// Apagar motor de la cinta.
			// Disparar alarma... Dado que llegó, puedo reutilizar TIMER 1.
			encenderCinta = 0;
			tiempoEncendido = 0;
			dispararAlarma = 1;
		} else if (!dispararAlarma) {
			cbi(PORTD, Motor);
			sprintf(buffer, "Vel.: %d m/min  ", velocidad[v]);
			Lcd4_Set_Cursor(1,0);
			Lcd4_Write_String(buffer);

		}
		if (dispararAlarma && tiempoEncendido < 5000) {
			sbi(PORTD, Alarma);
			sprintf(buffer, "---- llego! ----");
			Lcd4_Set_Cursor(1,0);
			Lcd4_Write_String(buffer);

		} else if (dispararAlarma && tiempoEncendido >= 5000) {
			cbi(PORTD, Alarma);
			dispararAlarma = 0;
			tiempoEncendido = 0;
			desactivarTimer1();
		}
		
		Lcd4_Set_Cursor(2, 0);			// Posiciona cursor en fila 2 (de 2), columna 0 (de 16).
		sprintf(buffer, "Puesto: %d", p+1);
		Lcd4_Write_String(buffer);		// Escribe string.
		
    }
}

void activarTimer0() {
	TCCR0A = (0<<WGM01)|(0<<WGM00);						// Modo timer free-run.
	TCCR0B = (0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00);	// Modo timer y prescaler = 64.
	TIMSK0 = (1<<TOIE0);			// Habilito interrupción de este timer.
	TCNT0  = 6;						// VPC = 6
}

void desactivarTimer0() {
	TCCR0B = (0<<CS02)|(0<<CS01)|(0<<CS00);	// Timer parado
	TIMSK0 = (0<<TOIE0);					// Desabilito interrupción de este timer.
}



void desactivarTimer2() {
	TCCR2B = (0<<CS02)|(0<<CS01)|(0<<CS00);	// Timer parado
	TIMSK2 = (0<<TOIE0);					// Desabilito interrupción de este timer.
}

void activarTimer1() {
	TCCR1A = (0<<WGM11)|(0<<WGM10);						// Modo timer free-run.
	TCCR1B = (0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);	// Modo timer y prescaler = 8.
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 45535;
}
void desactivarTimer1() {
	TCCR1B = (0<<CS12)|(0<<CS11)|(0<<CS10);	// Timer parado
	TIMSK1 = (0<<TOIE1);					// Desabilito interrupción de este timer.
}