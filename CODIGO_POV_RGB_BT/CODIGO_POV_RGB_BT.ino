/*
 * Código C++ para Proyecto de grado - Tecnico Profesional en Electrónica -UDI
 * Elaborado por Michael Vargas
 * 2015
 */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "font.h"

//Configuración
#define PRINT_LETTERS 1
#define ENABLE_USB_DETECT  //Activar programación por USB
#define DEBUG 1//Enviar mensajes por consola para depurar
#define BUFFER_SIZE 64 // Tamaño maximo del buffer Serial
#define NUMERO_LEDS 8 //Numero de LEDS a usar
#define NUM_ESPACIO 6 //Define el número en el cual se inicializar el contar de columnas 
// Configurar los puertos a usar
#define LED_RED_PINNUM 5
#define LED_GREEN_PINNUM 4
#define LED_BLUE_PINNUM 3
#define TILTSWITCH_PINNUM 2
//Configuracion de registro de puertos
#define LED_CATHODES_DDRx  DDRB
#define LED_CATHODES_PORTx PORTB
#define LED_8_4_PORTx PORTD
#define LED_3_1_PORTx PORTC
#define TILTSWITCH_DDRx DDRB
#define TILTSWITCH_PORTx PORTB
#define SENSOR_HALL_PORTx PORTC
#define SENSOR_HALL_DDRx DDRC
#define SENSOR_HALL 5
#define USBDETECT 0
#define OFF_NPN() (LED_CATHODES_PORTx &= ~_BV(LED_RED_PINNUM) & ~_BV(LED_GREEN_PINNUM) & ~_BV(LED_BLUE_PINNUM)) //Funcion para poner en corte los 3 transistores

//Variables volatiles para usar dentro de la funcion de la interrupcion
//Variables para usar las IMAGENES almacenadas en la EEPROM
volatile int ani_length;
volatile int ani_idx;
volatile uint8_t colour_idx = 0; // 0 = R, 1 = G, 2 = B
volatile uint8_t shade_idx = 0;
volatile byte frame_buffer[8];
//Varaibles para mostrar letras
volatile uint8_t counter, columna = 0;
volatile byte buffer[BUFFER_SIZE];

// Recepcion de datos UART
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int size;

void setup() {
  Serial.begin(38400);
#ifdef DEBUG
  Serial.println("Display POV RGB Bluetooth V1.0");
  Serial.print("RAM: ");
  Serial.print(displayFreeRam());
  Serial.println(" bytes");
#endif
  // Seteo de pines E/S
  LED_CATHODES_DDRx  |= _BV(LED_RED_PINNUM) | _BV(LED_GREEN_PINNUM) | _BV(LED_BLUE_PINNUM);
  DDRC |= _BV(0) | _BV(1) | _BV(2); //Leds 1-3
  DDRD |= 0b11111000; // leds 3-8
  DDRB &= ~_BV(USBDETECT); //Setear el pin USB como entrada
  // Activar pull-up
  TILTSWITCH_PORTx |= _BV(TILTSWITCH_PINNUM);
  SENSOR_HALL_PORTx |= _BV(SENSOR_HALL);
  TILTSWITCH_DDRx &= ~_BV(TILTSWITCH_PINNUM);
  SENSOR_HALL_DDRx &= ~_BV(SENSOR_HALL);
  // Apagar todos los LEDS Y desactivar el pull-up en los demás pines
  OFF_NPN(); //Poner en corte los transistores y apagar los LEDS
  LED_3_1_PORTx &= ~(0x07);
  LED_8_4_PORTx &= ~(0xF8);
  for (byte i = 0; i < 3; i++) LEDS_TEST(i, 30); //Probar los leds 1x1 con un delay de 50ms
  //Configurar e iniciar el timer 1
  cli();//Deshabilitar interrupciones
  TCCR1A = 0; //Asignar en 0 el registro TCCR1
  TCCR1B = 0; // lo mismo para el TCCR1B
  // Asignar el numero de conteos para comparar
  // Sabiendo que se asiganará con un prescaler de 8
  // 1 segundo = 12Mhz / 8 = 1500000
  // Se quiere que el comparador se active cada 100us entonces = 1.5*100 -> 150
  // Encender el modo CTC segun lo que se desee mostrar imagenes o letras


#if PRINT_LETTERS
  OCR1A = 44; // 4ms
  TCCR1B = _BV(WGM12) | 0x05;
#else
  OCR1A = 0x96; //Establecer el registro de salida a ~100us (NOTA: NO USAR EL PIN 10)
  TCCR1B |= (1 << WGM12);
  //Asignar prescaler de 8
  TCCR1B |= (1 << CS11);
#endif
  // Activar comparar temporizador de interrupción:
  TIMSK1 |= (1 << OCIE1A);
  sei(); // habilitar las interrupciones
  inputString.reserve(200); //Reservar 200 bytes para la cadena que recibe los datos
}

void loop() {
#ifdef ENABLE_USB_DETECT
  // Si la conexión USB es detectada, saltar al bootloader
  if (PINB & 0x01) { //Leer el estado del pin 8
    wdt_enable(WDTO_15MS); //Reiniciar
  }
#endif //Activar USB

#if PRINT_LETTERS
  serialEvent(); //Llamar a la funcion para almacenar los datos seriales
  if (stringComplete) {
    if (inputString.startsWith("&", 0)) {
#if DEBUG
      Serial.println(inputString);
#endif
      inputString.trim();// Quitar espacios en blanco
      switch(inputString.charAt(1)){
        case 'R': colour_idx=0; break;
        case 'G': colour_idx=1; break;
        case 'B': colour_idx=2; break;
        default: colour_idx=0; break;
      }
      setBufferBytes(inputString.substring(2)); //Convertir los caracteres ASCII en posiciones del vector para cada caracter
    }
    // Limpiar entrada
    inputString = "";
    stringComplete = false;
  }
#endif
};
//Función que se ejecuta cada 100us ó 4ms
ISR (TIMER1_COMPA_vect)
{
#if PRINT_LETTERS
  uint8_t cathodeport = 0;
  if (counter >= size) counter = 0;
  if (columna < 5) {
    if (colour_idx == 0)  cathodeport = _BV(LED_RED_PINNUM);
    else if (colour_idx == 1) cathodeport = _BV(LED_GREEN_PINNUM);
    else if (colour_idx == 2) cathodeport = _BV(LED_BLUE_PINNUM);
    if (buffer[counter] != 0) {
      uint8_t temp = font[(int)buffer[counter]][columna];
      uint8_t aux = temp & 0xF8;
      //Cambiar los bits para que queden en orden los leds
      bitWrite(aux, 6, bitRead(temp, 3));
      bitWrite(aux, 5, bitRead(temp, 4));
      bitWrite(aux, 4, bitRead(temp, 5));
      bitWrite(aux, 3, bitRead(temp, 6));
      OFF_NPN();//OFF 
      LED_3_1_PORTx &= ~(0x07);
      LED_8_4_PORTx &= ~(0xF8);
      LED_CATHODES_PORTx |= cathodeport; //Asignar el color escogido, por defecto es rojo
      LED_3_1_PORTx |= temp & 0x07;
      LED_8_4_PORTx |= aux;
    } else {
      LED_3_1_PORTx &= ~(0x07);
      LED_8_4_PORTx &= ~(0xF8);
    }

  } else {
    LED_3_1_PORTx &= ~(0x07);
    LED_8_4_PORTx &= ~(0xF8);
    if (columna == NUM_ESPACIO) columna = 0;
    if (size > 0 ) counter++;
  }
  columna++;

#else
  uint8_t i, b, aux, temp, cathodeport = 0, ledport = 0;
  if (colour_idx == 0) {
    cathodeport = _BV(LED_RED_PINNUM);

    for (i = 0; i < 8; i++) {
      b = (frame_buffer[i] & 0xE0) >> 5;
      if (b > shade_idx)
        ledport |= _BV(i);
    }
  }
  else if (colour_idx == 1)
  {
    cathodeport = _BV(LED_GREEN_PINNUM);

    for (i = 0; i < 8; i++) {
      b = (frame_buffer[i] & 0x1C) >> 2;
      if (b > shade_idx)
        ledport |= _BV(i);
    }
  }
  else if (colour_idx == 2)
  {
    cathodeport = _BV(LED_BLUE_PINNUM);

    uint8_t s = shade_idx >> 1;

    for (i = 0; i < 8; i++) {
      b = frame_buffer[i] & 0x03;
      if (b > s)
        ledport |= _BV(i);
    }
  }
  temp = ledport & 0xF8;
  bitWrite(aux, 7, bitRead(temp, 7));
  bitWrite(aux, 6, bitRead(temp, 3));
  bitWrite(aux, 5, bitRead(temp, 4));
  bitWrite(aux, 4, bitRead(temp, 5));
  bitWrite(aux, 3, bitRead(temp, 6));
  LED_3_1_PORTx &= ~(0x07);
  LED_8_4_PORTx &= ~(0xF8);
  OFF_NPN();
  LED_CATHODES_PORTx |= cathodeport;
  LED_3_1_PORTx |= ledport & 0x7;
  LED_8_4_PORTx |= aux;
  colour_idx++;
  if (colour_idx >= 3) {
    colour_idx = 0;
    shade_idx++;
    if (shade_idx >= 7) {
      shade_idx = 0;
      ani_idx++;
      if (ani_idx >= ani_length) {
        ani_idx = 0;
      }
      eeprom_read_block((void*)frame_buffer, (const void*)(2 + (ani_idx * 8)), 8); // Cargar siguiente columna
    }
  }
#endif

}
//######### Función para probar los LED's cada vez que se encienda el POV #########
void LEDS_TEST(byte color, int time) {
  static byte bit;
  bit = 0b00000001;
  OFF_NPN();
  switch (color) { //Saturar a un transistor NPN segun el color deseado
    case 0: LED_CATHODES_PORTx |= _BV(LED_RED_PINNUM); break;
    case 1: LED_CATHODES_PORTx |= _BV(LED_GREEN_PINNUM); break;
    case 2: LED_CATHODES_PORTx |= _BV(LED_BLUE_PINNUM); break;
  }
  for (int i = 0; i < NUMERO_LEDS; i++) { //Encender de a un LED
    if (i < 3) {
      LED_3_1_PORTx |= bit & 0x07;
      bit = bit << 1;
    }
    else {
      if (i == 3) bit = 0b01000000;
      else if (i == 7)  bit = 0b10000000;
      LED_8_4_PORTx |= bit & 0xF8;
      bit = bit >> 1;
    }
    delay(time);
  }
  LED_3_1_PORTx &= ~(0x07); // Apagar leds
  LED_8_4_PORTx &= ~(0xF8);
}
//######## Función para asignar el buffer de caracteres ###
void setBufferBytes(String input) {
#if DEBUG
  Serial.println(input);
#endif
  size = input.length(); //Tamaño de la cadena o frase
  if (size > BUFFER_SIZE) size = BUFFER_SIZE;
  for (int k = 0; k < sizeof(buffer); k++) buffer[k] = 0; //Borra el contenido del buffer
  for (int i = 0; i < size; i++) buffer[i] = (byte)input[i] - 32; //Guarda los valores ASCII en número para asignarlos en el vector
}
//#######Almacenar datos recibos como cadena de caracteres(String)####
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // Concatenar
    inputString += inChar;

    if (inChar == '\n') { //Si hay un salto de linea
      stringComplete = true;
    }
  }
}

