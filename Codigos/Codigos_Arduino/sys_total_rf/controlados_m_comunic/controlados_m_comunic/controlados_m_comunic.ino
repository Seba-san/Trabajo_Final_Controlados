
#include "Arduino.h"    //Contiene todas las declaraciones de funciones y registros de arduino
#include "Controlados.h" //Acá están las funciones propias para no tener todo mezclado.
                          //Controlados es la clase. Ahora necesito crear objetos de esta clase:

int entero_=0;
float flotante_;
long largo_;
int trama_activa=0;
//Comunicacion entero(&entero_);
//Comunicacion flotante(&flotante_);
Comunicacion largo(&largo_);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }


//entero.iniciar();
//flotante.iniciar();
largo.iniciar();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  /*
  Serial.println("solo: ");
  entero_=entero_+1;
  Serial.println(entero_);
    Serial.println("direccion: ");
    int a=&entero_;
    Serial.println(a);
    Serial.println("modificado con puntero: ");
    int *ab;
    ab=&entero_;
    *ab=*ab+1;
    Serial.println(entero_);

    
Serial.println("solo: ");
  entero_=entero_+1;
  Serial.println(entero_);
    Serial.println("direccion: ");
    int a=&entero_;
    Serial.println(a);
    Serial.println("con funcion: ");
    entero.enviar();
    entero.modificar(43);
    Serial.println("modificado: ");
    Serial.println(entero_);
    Serial.println("con funcion: ");
    entero.enviar();
*/
    

/*
    Serial.println("solo: ");
    flotante_=flotante_+1;
    Serial.println(flotante_);
    Serial.println("direccion: ");
    int a=&flotante_;
    Serial.println(a);
    Serial.println("con funcion: ");
    flotante.enviar();
    flotante.modificar(43);
    Serial.println("modificado: ");
    Serial.println(flotante_);
    Serial.println("con funcion: ");
    flotante.enviar();

*/


    Serial.println("solo: ");
    largo_=largo_+1;
    Serial.println(largo_);
    Serial.println("direccion: ");
    int a=&largo_;
    Serial.println(a);
    Serial.println("con funcion: ");
    largo.enviar();
   // largo.modificar(43);
    Serial.println("modificado: ");
    Serial.println(largo_);
    Serial.println("con funcion: ");
    largo.enviar();

    
}



void serialEvent() { // $4 esta funcion se activa sola, es por interrupciones
  unsigned long dato;
  if (Serial.available() > 0) {
        if(trama_activa==0){
          

          
    dato= Serial.read();

      

      }
      }
      }
      }
