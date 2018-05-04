#include "interrupciones.h"
#include "Controlados.h"





ISR (TIMER2_COMPA_vect){//Interrupción por Timer2 para definir frec de muestreo del sist cte; Resetea con el valor de comparacion del A
   timer_interrupt();}
 // $5 se utiliza esta funcion ya que permite llamarla desde otras funciones. Si lo haces desde la interrupcion, no podrias.
//*/
  //estado2=!estado2;
void  timer_interrupt(void){
  cantOVerflow++;
 soft_prescaler++;
  if(cantOVerflow>cota){
   bitWrite(Bandera,1,1); //medirVelocidad(0);//Llamo a la rutina de medición de vel indicándole que pasó demasiado tiempo y que tiene que asumir que la velocidad es 0.
  cantOVerflow=0;
  }
  bitWrite(Bandera,0,1); // Esta bandera le avisa al resto de las funciones que se produjo una interrupcion por timer

  if (soft_prescaler>=2){
    soft_prescaler=0;
    bitWrite(Bandera,3,1);
    toc();
    }
if (soft_prescaler==1){ // Lo hago en 2 pasos para que la acualizacion si se acontrolada. $interrup
      //PID_online();
      // Esta funcion mete mucho tiempo de computo 120 uS
      int aux=u[2]/k0;
      controlados1.actualizarDutyCycleMotores((int)(aux),(int)aux);//Realizo la actualización simultánea de la velocidad de ambos motores. $VER que haya terminado de calcular el PID
   estado=!estado;
   digitalWrite(SalidaTest,estado);
    }


  //estado=!estado;
  //if (estado){
   // estado2=!estado2;
  //digitalWrite(SalidaTest,estado2);
  //}
  //digitalWrite(SalidaTest,0);
}


ISR(PCINT1_vect){
// $6
//*/
 /*
  * Hay que verificar donde fue el cambio de estado, porque al tener 2 ruedas, no se sabe de donde provino (habria que hacer una comparacion manual)
  */
//digitalWrite(SalidaTest2,HIGH);

  TCNT2anterior=TCNT2actual;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.
  if (bitRead(TIFR2,1)){ // me fijo si hay overflow
  timer_interrupt();
  bitSet(TIFR2,1); // borro bandera para que no entre de nuevo
  }
  TCNT2actual=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.
  //toc();
  cantOVerflow_actual=cantOVerflow;
  cantOVerflow=0;
  bitWrite(Bandera,2,1);   //medirVelocidad(1);
  //digitalWrite(SalidaTest2,LOW);
}





void serialEvent() { // $4 esta funcion se activa sola, es por interrupciones (ponele)
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
         // Serial.println(171,DEC);
          break;
          case ins_test://Instrucción 249: codigo para exigir una resputa preestablecida. Sirve para saber si hay conexion.
          Serial.println(170,DEC);
           break;
          case ins_setpoint://Instrucción 248: cambiar el valor del setpoint
          trama_activa=3;
          break;
          case ins_parametros://Instrucción 248: cambiar el valor del setpoint
          trama_activa=4;
          break;
          case ins_u_control://Instrucción 248: cambiar el valor del setpoint
          trama_activa=5;
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
      if (!PWMA && !PWMB ){
        Motores_ON=false;
        }
        else {
        Motores_ON=true;}
      //Serial.println(172,DEC);
    }
    else if (trama_activa==3){
      set_point=dato; // Actualiza el valor del setpoint
      trama_activa=0;
      }
    else if (trama_activa==4){
      Parametros[0]=dato/10000;
      trama_activa=0;
      }
      else if (trama_activa==5){
       // digitalWrite(13,HIGH);
      u[2]=dato;
      trama_activa=0;
       // digitalWrite(13,LOW)  ;
      }
  }
}
