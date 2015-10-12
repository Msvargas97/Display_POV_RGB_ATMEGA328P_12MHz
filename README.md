 Display_POV_RGB_ATMEGA328P_12MHz
Control de display POV RGB v1.1 + Codigo Arduino+Bootloader V-USB

 Código C++ para Proyecto de grado - Tecnico Profesional en Electrónica -UDI

 ALGORITMO
  
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
   
   
Caracteristicas:

Posee tres modos de operación:
-Modo frases ingresadas por Bluetooth o comunicacion serial.
-Modo visualizador de imagenes cargadas desde la EEPROM
-Modo reloj digital

  Elaborado por Michael Vargas
  
  2015
  
  msvargas97@gmail.com
  
  Open Source GitHub: https://github.com/Msvargas97/Display_POV_RGB_ATMEGA328P_12MHz
  
  Créditos al MiniPOV4 de Adrafuit, por el bootloader y el software para cargar imagenes.
  
  https://github.com/adafruit/Adafruit-MiniPOV4-Kit
