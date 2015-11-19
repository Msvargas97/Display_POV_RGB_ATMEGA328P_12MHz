/*
   Código C++ para Proyecto de grado - Tecnico Profesional en Electrónica -UDI

  /*ALGORITMO

    Los pasos para realización del código son los siguientes:

    0.Esperar que el sensor de efecto Hall indique el punto de partida de visualización
    1.Se encienden los LEDs según los bits correspondientes de un número en binario
    que hace referencia a una columna de cualquier caracter, ya sea una image,numeros o letras.
    2.Gracias a que el timer esta configurado como comparador se asigna un periodo, el cual sera el tiempo que permaneceran
    encendida la columna de LEDs del paso anterior.
    3.Apagar los LEDs y repetir el algoritmo desde el paso 0.

    Posteriormente se tiene caracteristicas extras que se añaden en cualquiera de los pasos
    mencionados anteriormente segun sea su función,por ejemplo escoger el color de los LEDs
    esto se logra enviando un '1' lógico a la base del transistor que controla el color deseado,
    dicha caracteristica se implemento en el paso 1.

    Elaborado por Michael Vargas
    2015
    msvargas97@gmail.com
    Open Source GitHub: https://github.com/Msvargas97/Display_POV_RGB_ATMEGA328P_12MHz

    Créditos al MiniPOV4 de Adrafuit, por el bootloader y el software para cargar imagenes.

    #MADE_IN_COLOMBIA

*/
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "font.h"

//Configuración
#define ENABLE_USB_DETECT  //Activar programación por USB
#define ENABLE_HALL 1
#define DEBUG 1//Enviar mensajes por consola para depurar
#define BUFFER_SIZE 32 // Tamaño maximo del buffer Serial
#define NUMERO_LEDS 8 //Numero de LEDS a usar
#define NUM_ESPACIO 5 //Define el número en el cual se inicializar el contar de columnas 
// Configurar los puertos a usar para los transistores
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
volatile uint8_t colour_idx = 0, lastColour = 0; // 0 = R, 1 = G, 2 = B, repeating
volatile uint8_t shade_idx = 0;
volatile byte frame_buffer[8];
//Varaibles para mostrar letras
volatile uint8_t counter, led_test = 5, toggleMode = '0',counter2;
volatile int8_t  columna = 0;
volatile byte buffer[BUFFER_SIZE];
volatile byte temp_buffer[BUFFER_SIZE];
volatile boolean Flag = true, Flag2 = true,FlagHall=false,ENABLE = false,HOLD=false;
String tmp_str; //Almacena los datos de fecha y hora
// Recepcion de datos UART
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int size;
//Varaibles para visualizar la hora
int sampleTime = 5;
unsigned long time; //timer para actualizar el reloj
volatile unsigned int date_clock[] = {10, 58, 00, 12, 10, 2015};
void setup() {
  Serial.flush();    //Borra los datos almacenados en el buffer
  Serial.begin(38400); //Inicializa la comunicación serial
#ifdef DEBUG
  Serial.println("Display LED giratorio controlado por Bluetooth");
  Serial.println("                Michael Vargas                ");
  Serial.println("");
  Serial.print("RAM libre: ");
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
  PORTB |= _BV(2);
  OFF_NPN(); //Poner en corte los transistores y apagar los LEDS
  LED_3_1_PORTx &= ~(0x07);
  LED_8_4_PORTx &= ~(0xF8);
  for (byte i = 0; i < 3; i++) LEDS_TEST(i,80); //Probar los leds 1x1 con un delay de 30ms
  //Configurar e iniciar el timer 1
  noInterrupts();//Deshabilitar interrupciones
  TCCR1A = 0; //Asignar en 0 el registro TCCR1
  TCCR1B = 0; // lo mismo para el TCCR1B
  // Asignar el numero de conteos para comparar
  // Sabiendo que se asiganará con un prescaler de 8
  // 1 segundo = 12Mhz / 8 = 46875 , como el timer1 es de 16bits es un número que se puede asignar al registro
  // Se quiere que el comparador se active cada 100us entonces = 1.5*100 -> 150
  OCR1A = 14; // 750us
  TCCR1B = _BV(WGM12) | 0x04; //Asignar Prescaler de 256
  // Activar comparar temporizador de interrupción:
  TIMSK1 |= (1 << OCIE1A);
  inputString.reserve(200); //Reservar 200 bytes para la cadena que recibe los datos
  // Obtener el ancho de la imagen
  ani_length = (eeprom_read_byte((uint8_t*)0) << 8) + eeprom_read_byte((uint8_t*)1);
  Serial.print("Imagen encontrada:"); Serial.print(ani_length); Serial.println(" byte");
  setTime(00, 00, 00, 00, 00, 0000); //Asignar la fecha para el reloj
  interrupts();// habilitar las interrupciones
}
void loop() {
#ifdef ENABLE_USB_DETECT
  // Si la conexión USB es detectada, saltar al bootloader
  if (PINB & 0x01) { //Leer el estado del pin 8
    wdt_enable(WDTO_15MS); //Reiniciar
  }
#endif //Activar USB

  serialEvent(); //Llamar a la funcion para almacenar los datos seriales

  if (stringComplete) {
    noInterrupts();//Deshabilitar interrupciones
    inputString.trim();// Quitar espacios en blanco
    if (inputString.startsWith("&", 0)) {
      setColor(1);
      lastColour = colour_idx;
      setBufferBytes(inputString.substring(2)); //Convertir los caracteres ASCII en posiciones del vector para cada caracter
      Flag = false;
    } else  if (inputString.startsWith("$", 0)) {
      toggleMode = inputString.charAt(1); //Selecciona el modo de operación del display
      if ( toggleMode < 58) {
        switch (toggleMode) {
          case '0':
          case '2': cli(); OCR1A = 14; TCCR1B = _BV(WGM12) | 0x04; columna = 0;  colour_idx = lastColour; sei(); break; //Cambia el periodo a 750us
          case '1': cli(); OCR1A = 1; TCCR1B = _BV(WGM12) | 0x01;  break; //Cambia el periodo a 100us ya que se necesita una mayor frecuencia para visualizar las imagenes
          case '3': OCR1A = map(inputString.substring(2).toInt(), 0, 1000, 0, 46875); break; //Regla de tres para determinar el periodo lo maximo es 1 segundo
          case '4': OFF_NPN(); LED_3_1_PORTx &= ~(0x07);  LED_8_4_PORTx &= ~(0xF8); Flag = Flag2 = true; break;
        }
        if (toggleMode == '2') {
          setColor(2);
          setTime(inputString.substring(3, 5).toInt(), inputString.substring(5, 7).toInt(), inputString.substring(7, 9).toInt(), inputString.substring(9, 11).toInt(), inputString.substring(11, 13).toInt(), inputString.substring(13, 16).toInt());
          sampleTime = 1000;//Tiempo en milisegundos
        } else {
          sampleTime = 5; //Tiempo de muestreo inicial
        }
        OFF_NPN();
#if ENABLE_HALL
        ENABLE = false;
#endif
        LED_3_1_PORTx &= ~(0x07);
        LED_8_4_PORTx &= ~(0xF8);
        Flag = Flag2 = true;
      }
#if DEBUG
      Serial.println(toggleMode);
#endif
      Flag = false;
    }
    // Limpiar entrada
    inputString = "";
    stringComplete = false;
    interrupts();// habilitar las interrupciones
  }
  if (millis() - time >= sampleTime) {
    noInterrupts();//Deshabilitar interrupciones
    if (toggleMode == '2') setBufferBytes(clock());
    interrupts();// habilitar las interrupciones
    time = millis();
    }
};

ISR(TIMER1_COMPA_vect)
{
  
  if (Flag) {
#if DEBUG
    if (Flag2) Serial.println("Esperando datos...");
    Flag2 = false;
#endif
    return;
  }
#if  ENABLE_HALL
  if (!digitalRead(A5) && !FlagHall){ENABLE = true; FlagHall=true;} //Determinar el punto de inicio del display usando el sensor hall
  else if(digitalRead(A5)){ FlagHall=false;}
  
 if(counter == size ){counter=0;ENABLE=false;}
  if (ENABLE) {
#endif
    PORTB |= _BV(2);//Se puede poner un led en el pin 10 como indicador del ciclo del programa
    if (toggleMode == '0' || toggleMode == '2') { //Mode '0' visualización de frases enviadas por bluetooth o visualziar la hora '2'

      uint8_t cathodeport = 0; //Reinicia las variables
      if (columna < 5) {
        // Serial.println(columna);
        if (colour_idx == 0)  cathodeport = _BV(LED_RED_PINNUM);
        else if (colour_idx == 1) cathodeport = _BV(LED_GREEN_PINNUM);
        else if (colour_idx == 2) cathodeport = _BV(LED_BLUE_PINNUM);
        if (buffer[counter] != 0) {
          uint8_t  temp = font[(int)buffer[counter]][columna];//Selecciona el valor segun el vector multidimensional
          uint8_t  aux = temp & 0xF8; //Usar los pines al que estan los LEDs dejar los otros intactos
          //Cambiar los bits para que queden en orden los leds
          bitWrite(aux, 6, bitRead(temp, 3));
          bitWrite(aux, 5, bitRead(temp, 4));
          bitWrite(aux, 4, bitRead(temp, 5));
          bitWrite(aux, 3, bitRead(temp, 6));
          OFF_NPN();//Apagar los LEDS
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
        if (columna == NUM_ESPACIO) columna = -1;
        //Serial.println(counter);
        counter++;
      }
      columna++;
    } else if (toggleMode == '1') {
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
    } else if (toggleMode == '3') {
      OFF_NPN();
      LED_CATHODES_PORTx |= _BV(led_test);
      LED_3_1_PORTx |= 0x07;
      LED_8_4_PORTx |= 0xF8;
      led_test--;
      if (led_test == 2)led_test = 5;
    }
    PORTB &= ~_BV(2);

#if ENABLE_HALL
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
void setColor(byte position) {
  switch (inputString.charAt(position)) {
    case 'R': colour_idx = 0; break;
    case 'G': colour_idx = 1; break;
    case 'B': colour_idx = 2; break;
    default: colour_idx = 0; break;
  }
}
String clock() {
  static String result, str1, str2, str3; //Crear variables estáticas
  date_clock[2] += 1;
  if (date_clock[2] == 60) {
    date_clock[1] += 1;
    date_clock[2] = 0;
  }
  if (date_clock[1] == 60) {
    date_clock[0] += 1;
    date_clock[1] = 0;
  }
  if (date_clock[0] == 25) {
    date_clock[0] = 1;
    date_clock[3] += 1;
  }
  if (date_clock[3] == 31) {
    date_clock[3] = 1;
    date_clock[4] += 1;
  }
  if (date_clock[4] == 12) {
    date_clock[4] = 1;
    date_clock[5] += 1;
  }
  if (date_clock[0] < 10)
    str1 = "0" + String(date_clock[0]); //Castear de int a String
  else
    str1 = String(date_clock[0]);
  if (date_clock[1] < 10)
    str2 = "0" + String(date_clock[1]);
  else
    str2 = String(date_clock[1]);
  if (date_clock[2] < 10)
    str3 = "0" + String(date_clock[2]);
  else
    str3 = String(date_clock[2]);
  result = str1 + ":" + str2 + ":" + str3;
  return result;
}
void setTime(int HH, int MM, int SS, int dd, int mm, int aaaa) {
  date_clock[0] = HH;
  date_clock[1] = MM;
  date_clock[2] = SS;
  date_clock[3] = dd;
  date_clock[4] = mm;
  date_clock[5] = aaaa;
}


