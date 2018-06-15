#include "interrupciones.h"
#include "Controlados.h"
#include "Arduino.h"

ISR (TIMER2_COMPA_vect){//Interrupción por Timer2 para definir frec de muestreo del sist cte; Resetea con el valor de comparacion del A
   timer_interrupt();}
 // $5 se utiliza esta funcion ya que permite llamarla desde otras funciones. Si lo haces desde la interrupcion, no podrias.

void  timer_interrupt(void){
  cantOVerflowA++;
  cantOVerflowB++;
  soft_prescaler++;
  if(cantOVerflowA>cota){
    bitWrite(Bandera,1,1);
    cantOVerflowA=0;
  }
  if(cantOVerflowB>cota){
    bitWrite(Bandera,3,1);
    cantOVerflowB=0;
  }
  bitWrite(Bandera,0,1); // Esta bandera le avisa al resto de las funciones que se produjo una interrupcion por timer

  if (soft_prescaler>=2){ // $VER ponerlo en 2 para 200Hz por interrupt. 
    soft_prescaler=0;
    bitWrite(Bandera,5,1);
    }
if (soft_prescaler==1){ // Lo hago en 2 pasos para que la acualizacion si sea controlada. $interrup
      // Esta funcion mete mucho tiempo de computo 120 uS
      int auxA=uA[2],auxB=uB[2];
      //#if controlador
      controlados1.actualizarDutyCycleMotores((int)(auxA),(int)auxB);//Realizo la actualización simultánea de la velocidad de ambos motores. $VER que haya terminado de calcular el PID
      //#endif
    }
}


ISR(PCINT1_vect){
// $6
 /*
  * Hay que verificar donde fue el cambio de estado, porque al tener 2 ruedas, no se sabe de donde provino (habria que hacer una comparacion manual)
  */
  int A0, A1, MA, MB;
  A0=bitRead(PINC,0);//Leo el estado actual de los encoders.
  A1=bitRead(PINC,1);
  MA=bitRead(estadoEncoder,0);//Separo por comodidad los estados anteriores del encoder.
  MB=bitRead(estadoEncoder,1);

  //Serial.println("Interrupt");
  //Serial.print(PINC,BIN);Serial.print(" ");Serial.print(A0);Serial.print(" ");Serial.print(A1);Serial.print(" ");Serial.print(MA);Serial.print(" ");Serial.println(MB);
  
  int auxiliar;//$$$BORRAR

  if(A0!=MA){//Si A0 es distinto a MA es porque el estado del motor A cambió y eso fue lo que generó la interrupción.   
    bitWrite(estadoEncoder,0,A0);//Almaceno el estado del encoder para la proxima interrupción.
    if(A0==1 && MA==0){//Solo actualizo los valores cuando la señal del encoder tuvo un flanco ascendente
      TCNT2anteriorA=TCNT2actualA;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.
      TCNT2actualA=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.
      if (bitRead(TIFR2,1)){ // me fijo si hay overflow
      timer_interrupt();
      bitSet(TIFR2,1); // borro bandera para que no entre de nuevo
      }
      cantOVerflow_actualA=cantOVerflowA;
      cantOVerflowA=0;
      bitWrite(Bandera,2,1);
      }
  }
  if(A1!=MB){//Si A1 es distinto a MB es porque el estado del motor B cambió y eso fue lo que generó la interrupción.    
    bitWrite(estadoEncoder,1,A1);//Almaceno el estado del encoder para la proxima interrupción.
    if(A1==1 && MB==0){//Solo actualizo los valores cuando la señal del encoder tuvo un flanco ascendente
      TCNT2anteriorB=TCNT2actualB;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.
      TCNT2actualB=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.
      if (bitRead(TIFR2,1)){ // me fijo si hay overflow
      timer_interrupt();
      bitSet(TIFR2,1); // borro bandera para que no entre de nuevo
      }
      cantOVerflow_actualB=cantOVerflowB;
      cantOVerflowB=0;
      bitWrite(Bandera,4,1);
    }
  }
}

void serialEvent() { // $4 esta funcion se activa sola, es por interrupciones
  unsigned long dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    if(trama_activa==0){
      switch (dato){
        case ins_trama://Instrucción 253: transmitir con identificador de trama.
          online=false;
          tx_activada=true;
          break;
        case ins_online://Instrucción 252: transmitir sin identificador de trama.
          online=true;
          tx_activada=true;
          break;
        case ins_stop://Instrucción 251: cortar transmición
          tx_activada=false;
          break;
        case ins_PWM://Instrucción 250: cambiar PWM de los motores
          trama_activa=1;
          break;
        case ins_test://Instrucción 249: codigo para exigir una respuesta preestablecida. Sirve para saber si hay conexion.
          Serial.println(170,DEC);
           break;
        case ins_setpoint://Instrucción 248: cambiar el valor del setpoint
          trama_activa=3;
          break;
        case ins_resultado_ensayo://Instrucción 245: Devuelve de forma secuencial los datos del ensayo al escalón almacenados en w[N]
          enviar_datos=1;
          break;
           case ins_ack://Instrucción 244 ack
          ack=true;
          break;
        default://No hace nada si no recibe una instrucción válida
          break;}
    }
    else if (trama_activa==1){//Si trama_activa=1 es porque estaba esperando a recibir el nuevo valor de PWM del motor A
      PWMA=dato;
      trama_activa=2;//Le indico al nano que el próximo byte que reciba es el valor del PWM del motor B.
    }
    else if (trama_activa==2){//Si trama_activa=2 es porque estaba esperando a recibir el nuevo valor de PWM del motor B
      PWMB=dato;
      trama_activa=0;//Le indico al nano que terminé, por lo que el próximo byte que reciba debería ser una nueva instrucción.
      controlados1.actualizarDutyCycleMotores(PWMA,PWMB);//Realizo la actualización simultánea de la velocidad de ambos motores.
      controlados1.modoAdelante();
      if (!PWMA && !PWMB ){Motores_ON=false;}
      else {Motores_ON=true;}
      }
    else if (trama_activa==3){
      set_pointA=dato*10; // Actualiza el valor del setpoint de A
      trama_activa=4;
      }
    else if (trama_activa==4){
      set_pointB=dato*10; // Actualiza el valor del setpoint de B
      trama_activa=0;
      }
  }
}
