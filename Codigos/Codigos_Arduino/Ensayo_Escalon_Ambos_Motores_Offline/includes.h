/*
Este codigo fue copiado parcialmente de uno de Cliver Carrascal.

El archivo se llama A_Includes porque cuando Arduino compila, pone primero el .ino y concatena el resto de los archivos en orden alfabético.
//Segun entendi, si se declara una variable en A.h, ya la conoce B.h, pero no viceversa
*/
#include "Arduino.h"    //Contiene todas las declaraciones de funciones y registros de arduino
#include "Controlados.h" //Acá están las funciones propias para no tener todo mezclado.
                          //Controlados es la clase. Ahora necesito crear objetos de esta clase:
#include "funciones.h"
#include "interrupciones.h"


// #   #   #   # Definiciones $1
#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales

