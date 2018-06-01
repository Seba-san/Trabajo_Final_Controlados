/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */


 // Instrucciones traducidas
#define ins_PWM 250 // instrucciones traducidas.
#define ins_trama 253
#define ins_online 252
#define ins_stop 251 // baja de transmitir online
#define ins_test 249
#define ins_setpoint 248
#define ins_dev_ensayo 245
#include <SoftwareSerial.h>
float dato;
int a=0;
int trama_activa=0;
SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
  
}

void loop() { // run over and over

  if (mySerial.available()) {
    switch (trama_activa){
      case 0:
      break;

      case 100:
          dato=mySerial.parseFloat();
          Serial.println(dato,DEC);
         
      break;
      case 101:
      dato=mySerial.parseFloat();
       Serial.println(dato,DEC);
      break;
      default:
      break;
      }
    }
   // Serial.write(mySerial.read());
    //Serial.write( '\r' );
    // Serial.write(13);
  }
 



void serialEvent() { // $4 esta funcion se activa sola, es por interrupciones
  unsigned long datos;
if (Serial.available() > 0) {
    datos= Serial.read();
   // if(trama_activa==0){
      switch (datos){
        case ins_trama://Instrucción 253: transmitir con identificador de trama.
           mySerial.write(datos);
          trama_activa=101;
          break;
        case ins_online://Instrucción 252: transmitir sin identificador de trama.
            mySerial.println(datos);
          trama_activa=101;
          break;
        case ins_stop://Instrucción 251: cortar transmición
            mySerial.println(datos);
          trama_activa=100;
          break;
        case ins_PWM://Instrucción 250: cambiar PWM de los motores
            mySerial.println(datos);
          trama_activa=0;
          break;
        case ins_test://Instrucción 249: codigo para exigir una respuesta preestablecida. Sirve para saber si hay conexion.
         // Serial.println(170,DEC);
          mySerial.write(datos);
          trama_activa=100;
           break;
        case ins_setpoint://Instrucción 248: cambiar el valor del setpoint
          trama_activa=3;
          break;
           case  ins_dev_ensayo://Instrucción 248: cambiar el valor del setpoint
          trama_activa=101;
          mySerial.println(datos);
          break;
         
        default://No hace nada si no recibe una instrucción válida
          break;
   // }
    
}
}
}
    
