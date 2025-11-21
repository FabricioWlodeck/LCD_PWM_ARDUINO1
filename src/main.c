// DEFINICIONES 
// -----------------------------------------------------------------------------------------------
#define F_CPU		16000000UL		//	Define etiqueta Fcpu = 16 MHz (p/calc. de retardos).

// INCLUSION DE ARCHIVOS
// -----------------------------------------------------------------------------------------------
#include <avr/io.h>					//	Contiene definiciones est�ndares (puertos, etc.)
#include <avr/interrupt.h>			//	Contiene macros para manejo de interrupciones.
#include <util/delay.h>				//	Contiene macros para generar retardos.
#include <stdio.h>
#include <stdlib.h>	


// MACROS
// -----------------------------------------------------------------------------------------------
#define sbi(p,b)		p |= _BV(b)                // sbi(p,b) setea el bit b de p.
#define cbi(p,b)		p &= ~(_BV(b))             // cbi(p,b) borra el bit b de p.
#define tbi(p,b)		p ^= _BV(b)                // tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)    // is_high(p,b) testea si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0         // is_low(p,b) testea si el bit b de p es 0 (devuelve 1). Devuelve 1 si el bit b de p es cero

// DEFINICIONES
// -----------------------------------------------------------------------------------------------
#define	VPC1_2MS		65411				//	Valor de precarga para el Timer1 (modo Timer NORMAL free-running) interrup cada 2ms
#define	TOP_T2				124								//  valor comparacion mi timer 0 para f=2k Hz.
#define	TOP_T0				249								//  valor comparacion mi timer 0 para lograr 1ms.

#define DEBOUNCE_DELAY  10  // Retardo para eliminar el rebote 10 ms

// DEFINE PINES PARA MANEJO DEL LCD:
// ---------------------------------------------------------------------------------------
#define RS	eS_PORTB0			// Pin RS = PB0 (8) (Reset).
#define EN	eS_PORTB1			// Pin EN = PB1 (9) (Enable).
#define D4	eS_PORTB2			// Pin D4 = PB2 (10) (Data D4).
#define D5	eS_PORTB3			// Pin D5 = PB3 (11) (Data D5).
#define D6	eS_PORTB4			// Pin D6 = PB4 (12) (Data D6).
#define D7	eS_PORTB5			// Pin D7 = PB5 (13) (Data D7).
#include "lcd_2560.h"			// Contiene funciones para manejo del LCD.


// BOTONES
// -----------------------------------------------------------------------------------------------
#define P1_ON_OFF   PD2      // prende y apaga baliza y entra y sale de modo config
#define P2_CHANGE_CONFIG  PC0		            // pulsador P2, cambia de modo de configuracion


// VARIABLES GLOBALES
// -----------------------------------------------------------------------------------------------
volatile uint8_t P1_debounce = 0; // este flag me permitira saber si tengo que calcular el antirrebote para P1
volatile uint8_t P1_in_out_flag = 0; // flag para saber si apagar a los 5seg
volatile uint16_t timer_P1_debounce = 0; //variable para contabilizar el tiempo de antirrebote de P1
volatile uint16_t timer_P1_in_out_config = 0; //variable para contabilizar el tiempo para entrar o salir de modo config

char buffer[16];
char tiempo_temp[10];

volatile uint16_t switch_channel = 3;

volatile uint16_t v_ldr = 0; //c

volatile uint16_t v_rv2 = 0; //c

volatile uint16_t ADC_flag = 0; //nos indica si hay una nueva conversion
volatile uint16_t duty_cycle_valor = 0;

volatile uint16_t timer_ADC = 0; //contador para actualizar el valor medido por el ADC

volatile uint16_t config_mode = 0; //por defecto no entra en config mode
volatile uint16_t operation_mode = 0; //varia entre 0-1-2-3 Autom-NOnOff-NT5-NT10
volatile uint16_t time_on_leds = 0; //varia entre 0-1-2-3 Autom-NOnOff-NT5-NT10
volatile uint16_t time_leds_end = 0; //
volatile uint16_t pwm_value = 0;


volatile uint16_t changed_mode_flag = 1; //flag para saber cuando actulizar display AUXXX


volatile uint8_t P1_baliza_flag = 0;
volatile uint8_t P2_change_mode_flag = 0;

volatile uint8_t last_state_P2 = 0;
volatile uint8_t baliza_flag = 0;




// FUNCIONES
// -----------------------------------------------------------------------------------------------
void pinesConfig(){
  DDRB = 0b111111; //define todos los puertos B (son 6)como salidas
  PORTB = 0x00;

  DDRD = 0b11111000; //define a los 4 ultimos puertos D como salidas, entre ellos mas import PD3 PWM
  PORTD = 0x00;

  DDRC  = 0x00; //puerto C como entrada de pulsadores
  PORTC = 0x00; 

  // === Configuración de interrupción externa INT0 ===
  EICRA = 0x02;       // Configurar la interrupcion en el flanco de bajada.
  EIMSK = 0x01;		  // Habilitar la interrupcion externa INT0.

};

void startupSequence(){
  PORTD = 0b00000000; //pongo todo en 0 primero por si acaso

  PORTD = 0b00100000; //prendo el transistor de los leds
  _delay_ms(500);

  PORTD = 0b00000000; // Apago para el transistor de los leds
  _delay_ms(500);

  PORTD = 0b00100000; //prendo el transistor de los leds
  _delay_ms(500);

  PORTD = 0b00000000; // Apago para el transistor de los leds
  _delay_ms(500);
};

void inicioLCD()						// ESCRITURA INICIAL EN LCD
{	Lcd4_Set_Cursor(1,0);				// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
	Lcd4_Write_String("Tec. Digitales 2");	// Escribe string.
	Lcd4_Set_Cursor(2,0);				// Posiciona cursor en fila 2 (de 2), columna 0 (de 16).
	Lcd4_Write_Char('O');				// Escribe caracter
	Lcd4_Write_Char('K');				// Escribe caracter
	_delay_ms(500);					// Retarda x s.
	Lcd4_Clear();						// Borra el display.

  Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
  Lcd4_Write_String("Experiencia 1:"); // Escribe string.
  Lcd4_Set_Cursor(2,0);
  Lcd4_Write_String("Baliza LED"); // Escribe string.

  _delay_ms(500);		// Retarda x s.
  Lcd4_Clear();			// Borra el display.
  _delay_ms(500);		// Retarda x s.
}


void mostrar_tiempo(int x,char *y){
	float xseg = (float)x/1000;
	sprintf(y, "%.2fs", xseg);
}
void mostrar_pwm(int x, char *y){		
	float xpor = (float)x*100/124;
	if(xpor>100) xpor = 100;
	sprintf(y,"PWM al %.2f%%", xpor);		
}


void showLCD_config(uint16_t leds_time){
  switch (operation_mode){
    case 0:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.
        
        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("Select M: Auto"); // Escribe string.
        
        char lcd_buffer[17];
        sprintf(lcd_buffer, "DC: %u  T: %u s", duty_cycle_valor, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;

    case 1:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.
        
        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("Select M: N-OnOFf"); // Escribe string.
        
        char lcd_buffer[17];
        sprintf(lcd_buffer, "DC: %u  T: %u s", duty_cycle_valor, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;

    case 2:
    if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.
        
        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("Select M: N-T5"); // Escribe string.
        
        
        char lcd_buffer[17];
        sprintf(lcd_buffer, "DC: %u  T: %u s", duty_cycle_valor, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;
    case 3:
    if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.
        
        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("Select M: N-T10"); // Escribe string.
        
        char lcd_buffer[17];
        sprintf(lcd_buffer, "DC: %u  T: %u s", duty_cycle_valor, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;
  }
};

 void showLCD_WM(uint16_t leds_time){
  switch (operation_mode){
    case 0:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.
        
        /* Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("M: Automatico:"); // Escribe string.

        duty_cycle_valor = (long)v_rv2*124/1023;
        mostrar_pwm(duty_cycle_valor, &buffer[0]);

        
        Lcd4_Write_String("T: ");
				mostrar_tiempo(leds_time, tiempo_temp);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(buffer);
				Lcd4_Write_String(tiempo_temp); */

        Lcd4_Set_Cursor(1,0);
        Lcd4_Write_String("T5 ON -> ");
        mostrar_tiempo(leds_time, tiempo_temp);
        Lcd4_Set_Cursor(1,9);
        Lcd4_Write_String(tiempo_temp);
        duty_cycle_valor = (long)v_rv2*124/1023;
        mostrar_pwm(duty_cycle_valor, &buffer[0]);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(buffer);
        _delay_ms(100);
        duty_cycle_valor = 0;        
        changed_mode_flag = 0;
      }
    break;

    case 1:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.

        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("M: Normal On-Off:"); // Escribe string.
        
        duty_cycle_valor = (long)v_rv2*124/1023;
        mostrar_pwm(duty_cycle_valor, &buffer[0]);

        char lcd_buffer[17];
        sprintf(lcd_buffer, "%s T:%us", buffer, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;

    case 2:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.

        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("M: Normal T5:"); // Escribe string.
        
				duty_cycle_valor = (long)v_rv2*124/1023;
        mostrar_pwm(duty_cycle_valor, &buffer[0]);

        char lcd_buffer[17];
        sprintf(lcd_buffer, "%s T:%us", buffer, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;
    case 3:
      if(changed_mode_flag){
        Lcd4_Clear();			// Borra el display.
        _delay_ms(250);		// Retarda 250 ms.

        Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
        Lcd4_Write_String("M: Normal T10:"); // Escribe string.
        
        duty_cycle_valor = (long)v_rv2*124/1023;
        mostrar_pwm(duty_cycle_valor, &buffer[0]);

        char lcd_buffer[17];
        sprintf(lcd_buffer, "%s T:%us", buffer, leds_time);
        Lcd4_Set_Cursor(2,0);
        Lcd4_Write_String(lcd_buffer);
        changed_mode_flag = 0;
      }
    break;
  }
}



// TIMERS 
void timer0_config(){
  TCCR0A = 0b00000010;									// Modo CTC.
	TCCR0B = 0b00000011;									//(03 Hexa) Prescaler N = 64.
	TIMSK0 = 0b00000010;									// Habilita interrupcion por comparacion en comparador A.
	OCR0A = TOP_T0;									 // Carga el valor de TOP con 249. T = (1+OCR0A)*N/16MHz = 1ms => OCR0A = TOPE = 249.
};

void timer2_config(){
  TCCR2A = 0b00100011;									//(0x23 Hexa) Modo FAST PWN No invertido.
	TCCR2B = 0b00001100;									//(0x0c Hexa) Prescaler N = 64.
	TIMSK2 = 0b00000010;						// Activa la interrupción por igualación en OC0A
	OCR2A = TOP_T2;									// Carga el valor de TOP con 124. T = (1+OCR0A)*N/16MHz = 1ms => OCR0A = TOPE = 124.
  OCR2B = 0;
};


void confCONVAD(){            // Configuracion ADC	
  DIDR0 = 0b00001100;     // Deshabilita buffers digitales ADC2 y ADC3
  ADMUX = 0b01000011;     // Ref AVCC 5V ext capacitor, conversion 10 bits, Canal inicial 3

  //fuente de inicio: conv auto timer0 compare match A
  ADCSRB = 0b00000011;    
  
  // ADCSRA = 10101111 (0xAF)
  ADCSRA = 0b10101111;// ADEN=1; ADSC=0; ADATE=1; ADIF=0; ADIE=1; PRE=128

  // ==========================================
  // " este bit debe ponerse a “1” para iniciar la primera conversión; luego de esto, el bit ADSC permanece en “1” de forma permanente."
  // ==========================================
  sbi(ADCSRA, ADSC);
}

ISR(ADC_vect){
  //DEBUUUUG---------------------
  timer_ADC++;
  if(timer_ADC>=250){
    PORTD ^= (1 << PD6);
    timer_ADC=0;
  }
  //si 5 V = 1023
  //   3 V =~ 613 
  // la mitad de iluminosidad seria =~ 307 
  //----------------------------
  if(switch_channel==3){						// Si esta en el tercer canal tiene que leer el RV1
    if(v_rv2 != ADC) changed_mode_flag = 1;
    v_rv2 = ADC;					// Guarda la lectur del RV1	
    ADMUX = (ADMUX & 0xF0) | 0x02;		// Limpia MUX y establece ADC2
    switch_channel = 2;						// Seteo la bandera en el canal 2
  } else {								// En el canal dos tiene que leer el LDR
    if(v_ldr != ADC) changed_mode_flag = 1;
    v_ldr = ADC;					// Guarda la lectura del LDR
    ADMUX = (ADMUX & 0xF0) | 0x03;		// Limpia MUX y establece ADC3
    switch_channel = 3;						// Setea la bandera en el canal 3
  }
    //pruebo que entra cada 1ms
    sbi(PORTD,PD7);//Inicia conversion ADC  
    cbi(PORTD,PD7);	//termina conversion ADC  
}

ISR (TIMER2_COMPA_vect){
  OCR2B = duty_cycle_valor;
}

//Servicio de (INT 0) Interrupcion externa para P1
ISR(INT0_vect) {                       
  // Rutina de interrupción externa INT0 para P1
  //Si el boton 1 no esta presionado entro
  if(is_low(PIND,P1_ON_OFF)){
    EIMSK &= ~(1 << INT0); //dehabilito SOOOOLO!!!! la interrupcion INT0
    P1_debounce = 1; // Ahora debo realiza el antirebote para P2
    P1_in_out_flag = 1;  //Aca doy a saber que P1 fue presionado            NO Se CHEEEEEE
    timer_P1_debounce = 0; //con esto contabilizo los ms del antirebote
    timer_P1_in_out_config = 0; //variable para contabilizar el tiempo para entrar o salir de modo config
  }
}

//Sirvicio de interrupcion TIMER 0
ISR (TIMER0_COMPA_vect){		// RSI por comparacion del Timer0 con OCR0A (interrumpe cada 1 ms).
  
  // DEBUUUUG
  //pruebo que entra cada 1ms
  timer_ADC++;
  if(timer_ADC>=250){
    PORTD ^= (1 << PD7);
    timer_ADC=0;
  }
  
  PORTD = 0b00100000; // leds baliza ON
  
  //CONTADOR DE ms DE LOS MODOS
  if(baliza_flag){
    if(time_on_leds>0){
      time_on_leds--;
      changed_mode_flag=1; //permito actualizar el display en cada disminucion de tiempo
      if(time_on_leds==1){
        baliza_flag = 0;
        time_leds_end = 1;
      }
    }
  }
  if(time_leds_end){
    changed_mode_flag=1; //permito actualizar el display para apagarlo despues de terminar el timer de los modos
  }
  

  if(P1_debounce){
    timer_P1_debounce++;
    if (timer_P1_debounce >= 10){
      if (is_low(PIND, P1_ON_OFF)){
        P1_in_out_flag=1;
      }
    }

    if(P1_in_out_flag){
      // PULSACION CORTA
      // -----------------------------------------------------------------------------------------------
      if (is_high(PIND, P1_ON_OFF)){
        if(!config_mode) { // Tu restricción: solo fuera de config
            if(!baliza_flag){
              switch (operation_mode){
                case 0:
                  // recardo el tiempo
                  time_on_leds = 3000; //se prende durante 3 seg posterior a estimulo y se apagara luego
                break;
                case 1:
                  // recardo el tiempo
                  time_on_leds = 3000; //se prende durante 5 seg posterior a estimulo y se apagara luego
                break;
                  case 2:
                  // recardo el tiempo
                  time_on_leds = 5000; //se prende durante 10 seg posterior a estimulo y se apagara luego
                break;
                case 3:
                  // recardo el tiempo
                  time_on_leds = 10000; //se prende durante 3 seg posterior a estimulo y se apagara luego
                break;
              }
            }
            baliza_flag = !baliza_flag;
            changed_mode_flag = 1;
            P1_in_out_flag = 0;
            
        }
        P1_debounce = 0;						// reset flag antirebote
        EIMSK |= (1 << INT0);								// vuelvo a habilitar "SOLO!!" interrupcion INT0
        timer_P1_debounce=0; // vuelvo a dejar en 0 el tiempo de antirrebote para P1

      // PULSACION LARGA
      // -----------------------------------------------------------------------------------------------
      }else if(timer_P1_debounce >= 2000){
        P1_debounce = 0;						// reset flag antirebote
        EIMSK |= (1 << INT0);								// vuelvo a habilitar "SOLO!!" interrupcion INT0
        timer_P1_debounce=0; // vuelvo a dejar en 0 el tiempo de antirrebote para P1
        P1_in_out_flag = 0;
        
        config_mode = !config_mode;
        changed_mode_flag=1;
        baliza_flag = 0; // siempre la pongo en 0 al entrar o salir para que el usuario inicie manualmente

      }
    }
  }
}



// PROGRAMA PRINCIPAL
//-----------------------------------------------------------------------------------------------
int main(void){
  // INICIALIZACION
  //-----------------------------------------------------------------------------------------------
    pinesConfig();
    /* startupSequence(); */
    confCONVAD(); //soy un pelotudo, me habia olvidado de ponerlo en el main
    timer0_config();
    timer2_config();
    Lcd4_Init();				// Inicializa el LCD (debe estar antes de escribir x 1ra vez en el LCD).
    inicioLCD();				// Inicializa el LCD (siempre debe estar antes de usar el LCD).
    Lcd4_Clear();				// Borra el display.
    // Habilitar interrupciones globales
    sei();

    //BUCLE PRINCIPAL
    //-----------------------------------------------------------------------------------------------
    while(1){
      
      //MODO CONFIGURACION
      //-----------------------------------------------------------------------------------------------
      while(config_mode){
        showLCD_config(time_on_leds);
        //P2 por POOLING: Cambio de modos de configuracion
        if(is_low(PINC, P2_CHANGE_CONFIG)){ 
          _delay_ms(DEBOUNCE_DELAY);  // esperar para evitar el rebote
          if(is_low(PINC, P2_CHANGE_CONFIG) && last_state_P2 == 0){ //si sigue abajo despues del delay y su estado anterior fue bajo pongo en alto
            last_state_P2 = 1;
            switch (operation_mode){
              case 0:
                // CAMBIO A MODO NORMALON-OFF
                operation_mode = 1;
                changed_mode_flag=1;
                PORTD = 0b00100000; // leds baliza ON
                _delay_ms(200);
                PORTD = 0b00000000; // leds baliza OFF
                time_on_leds = 3000; //se prende durante 3 seg posterior a estimulo y se apagara luego
              break;
              case 1:
                // CAMBIO A MODO T5
                operation_mode = 2;
                changed_mode_flag=1;
                PORTD = 0b00100000; // leds baliza ON
                _delay_ms(200);
                PORTD = 0b00000000; // leds baliza OFF
                time_on_leds = 5000; //se prende durante 5 seg posterior a estimulo y se apagara luego
              break;
                case 2:
                // CAMBIO A MODO T10
                operation_mode = 3;
                changed_mode_flag=1;
                PORTD = 0b00100000; // leds baliza ON
                _delay_ms(200);
                PORTD = 0b00000000; // leds baliza OFF
                time_on_leds = 10000; //se prende durante 10 seg posterior a estimulo y se apagara luego
              break;
              case 3:
                // CAMBIO A MODO AUTOMATICO
                operation_mode = 0;
                changed_mode_flag=1;
                PORTD = 0b00100000; // leds baliza ON
                _delay_ms(200);
                PORTD = 0b00000000; // leds baliza OFF
                time_on_leds = 3000; //se prende durante 3 seg posterior a estimulo y se apagara luego
              break;
            }
          }
        }else { // restablecer el estado cuando el boton no se esta presionando
            last_state_P2 = 0;  
        }
      }

      //MODO BALIZA (entra por defecto)
      //-----------------------------------------------------------------------------------------------
      if(!config_mode){     
        //Si activo la baliza puedo dejar actuar a cada modo
        if(baliza_flag){
          PORTD = 0b00100000; // leds baliza ON
          showLCD_WM(time_on_leds);
        }else{
          if(changed_mode_flag){
            PORTD = 0b00000000; // leds baliza OFF
            Lcd4_Clear();			// Borra el display.
            _delay_ms(250);		// Retarda 250 ms.
            
            Lcd4_Set_Cursor(1,0);	// Posiciona cursor en fila 1 (de 2), columna 0 (de 16).
            Lcd4_Write_String("Baliza Apagada ...");
            Lcd4_Set_Cursor(2,0);
            changed_mode_flag=0;
            time_leds_end=0;
          } 
        }
      }   
  }
}