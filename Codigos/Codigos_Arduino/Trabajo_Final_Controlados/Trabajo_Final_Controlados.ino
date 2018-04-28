
#include <avr/interrupt.h> //Esto lo pongo porque decía el manual de avr que
#include <avr/io.h>        //supuestamente lo necesito para las interrupciones

#include "Controlados.h" //Acá están las funciones propias para no tener todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta clase:
Controlados controlados1;

// #   #   #   # Definiciones 
#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
// Instrucciones traducidas
#define ins_PWM 250 // instrucciones traducidas. 
#define ins_trama 253
#define ins_online 252
#define ins_stop 251 // baja de transmitir online
#define ins_test 249
#define ins_setpoint 248


// #   #   #   # Constantes 
const int cantMarcasEncoder = 8; //Es la cantidad de huecos que tiene el encoder de cada motor.
const int FsEncoders = 2000;//8000 2000 // Esto significa Overflow cada 2Khz
const int preescaler = 32;//8 32 64 
const int cota = 75;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
const unsigned long _OCR2A = 250;
// F_CPU es el valor con el que esta trabajando el clock del micro.


// ############# Probandos cosas locas 
#define NOP __asm__ __volatile__ ("nop\n\t")
#define SalidaTest 3
#define SalidaTest2 4
#define SalidaTest3 5
bool estado=0,estado2=0,estado3=0, estado4=0;
 //unsigned long suma=0;//float suma=0;
 
// #   #   #   # Variable Basura
int Bandera=0; // bandera para administrar procesos fuera de interrupciones 
float b, freq; 
unsigned long cuenta;    
//################

// #   #   #   # Variables
unsigned long ticc,tocc;
unsigned char trama_activa=0;//Lo pongo en unsigned char para que ocupe 1 byte (int ocupa 2)
bool online;//Me indica si poner o no el identificador para la transmisión (online=true no lo transmite para ahorrar tiempo)
bool tx_activada;//Me indica si transmitir o no.
bool iniciar=false; // le dice al micro cuando puede iniciar.
int PWMA;int PWMB;//Acá guardo los valores de nuevos de PWM que me mada Matlab para que actualice. La actualización efectiva se hace cuando tengo estos dos valores.
int cantOVerflow=0;//Variable que almacena la cantidad de veces que se desbordó el timer2 hasta que vuelvo a tener interrupción por pin de entrada. Esto permite realizar la medición de 
                  //tiempos entre aujeros del encoder.                                                                              
/*
int TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
int TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.                
int cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.          
*/
unsigned long TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
unsigned long TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.                
unsigned long cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior. 
unsigned long aux[6];  // este es un buffer para enviar datos en formato trama, corresponde a la funcion "EnviarTX"       
unsigned long  bufferVel[2*cantMarcasEncoder];//buffer donde almaceno las últimas velocidades calculadas.
bool BanderadelBuffer=true;
bool Motores_ON=false;
float velAngular=0;//última velocidad angular calculada. Lo separo del vector bufferVel para que no haya problemas por interrumpir en medio de una actualización de éste.
float velDeseada=0;//Empieza detenido el motor.
// Variables del PID
float u[3]; // historia del error cometido y la historia de las salidas de control ejecutadas.
float error[3];
float set_point=0; // Set_point esta en RPM
const float Parametros[]={0.0179  ,  0.0161  ,  0.0214, 1, 0};//{1.261400, -2.522133, 1.260733, 1.999500, -0.999500};
int soft_prescaler=0;


// #   #   #   # Declaracion de Funciones

void medirVelocidad(unsigned char);
void EnviarTX(int cantidad,char identificador, unsigned long *datos);
void EnviarTX_online(unsigned long var);
void timer_interrupt(void);
void PID_offline(void);


void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.

Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)

 controlados1.configPinesMotores();
 controlados1.modoStop();
 controlados1.configTimerMotores();
 controlados1.configTimer2Contador(FsEncoders,preescaler,1);//Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.   
 controlados1.actualizarDutyCycleMotores(0,0);
 controlados1.modoAdelante();

  interruptON;//Activo las interrupciones
  pinMode(A0, INPUT);
  // pinMode(A2, OUTPUT);
   //digitalWrite(A2,0);
  // $Prueba
  //pinMode(SalidaTest, OUTPUT);
  //pinMode(SalidaTest2, OUTPUT);
  //pinMode(SalidaTest3, OUTPUT);
  tic();
}

void loop() {
  NOP;

  //while (!iniciar) {
   //  NOP;
  //}
  
//controlados1.actualizarDutyCycleMotores(0,0);
//controlados1.modoAdelante();
if (bitRead(Bandera,0)){ // timer 2 overflow
  
  } 
  if (bitRead(Bandera,1)){ // Entra cuando no registra cambio en la entrada
  // Serial.println  (Bandera,BIN);
  bitWrite(Bandera,1,0);
  medirVelocidad(0);   

  } 
  if (bitRead(Bandera,2)){ // se registra cambio en la entrada
  bitWrite(Bandera,2,0);
  medirVelocidad(1);  
  } 
   if (bitRead(Bandera,3)){ // Se midio un tiempo de 15mS, se realiza el calculo del PID
  bitWrite(Bandera,3,0);
  PID_offline(); // $VER, analizar esto, porque va a entrar varias veces (entre 8 y 9 o mas) antes de tener una nueva medida de las RPM
  // Si no me equivoco lo mejor seria tomar muestras a 66Hz (considerando 500RPM como minimo) eso da 15mS de Ts. 
  } 
}

void serialEvent() { // esta funcion se activa sola, es por interrupciones (ponele)
  int dato;
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
  }
}

ISR (TIMER2_COMPA_vect){//Interrupción por Timer2 para definir frec de muestreo del sist cte; Resetea con el valor de comparacion del A

timer_interrupt(); 

}
void timer_interrupt(){ // se utiliza esta funcion ya que permite llamarla desde otras funciones. Si lo haces desde la interrupcion, no podrias.
  //estado2=!estado2;
  //digitalWrite(SalidaTest,estado2);
  cantOVerflow++;
 soft_prescaler++;
  if(cantOVerflow>cota){
   bitWrite(Bandera,1,1); //medirVelocidad(0);//Llamo a la rutina de medición de vel indicándole que pasó demasiado tiempo y que tiene que asumir que la velocidad es 0.                                        
  cantOVerflow=0;
  }
  bitWrite(Bandera,0,1); // Esta bandera le avisa al resto de las funciones que se produjo una interrupcion por timer

  if (soft_prescaler==30){
    soft_prescaler=0;
    bitWrite(Bandera,3,1);
    }
if (soft_prescaler==1){ // Lo hago en 2 pasos para que la acualizacion si se acontrolada. $interrup
      //PID_online();
      controlados1.actualizarDutyCycleMotores((int)(u[2]),(int)u[2]);//Realizo la actualización simultánea de la velocidad de ambos motores. $VER que haya terminado de calcular el PID
    controlados1.modoAdelante();
    }

  
  //estado=!estado;
  //if (estado){
   // estado2=!estado2;
  //digitalWrite(SalidaTest,estado2);
  //}
  //digitalWrite(SalidaTest,0);
}
ISR(PCINT1_vect){
 /*
  * Hay que verificar donde fue el cambio de estado, porque al tener 2 ruedas, no se sabe de donde provino (habria que hacer una comparacion manual)
  */
 // estado3=!estado3;
 // if (estado3){
 //digitalWrite(SalidaTest2,estado3);
  TCNT2anterior=TCNT2actual;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.   
  if (bitRead(TIFR2,1)){ // me fijo si hay overflow
  timer_interrupt();
  bitSet(TIFR2,1); // borro bandera para que no entre de nuevo
  }                        
  TCNT2actual=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.                   
  cantOVerflow_actual=cantOVerflow; 
  cantOVerflow=0;
  bitWrite(Bandera,2,1);   //medirVelocidad(1); 
  toc();//Actualizo el valor de tocc
  //estado=!estado;
   //digitalWrite(SalidaTest2,0);
  //}
  
}

void medirVelocidad(unsigned char interrupcion)
{


  
unsigned long suma=0;
 long t,tmh; // Lo hago por partes porque todo junto generaba errores en algunas ocaciones
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:  
  for(int k=0;k<(2*cantMarcasEncoder-1);k++)
  {
    bufferVel[k]=bufferVel[k+1];//Desplazamiento a la derecha de los datos del buffer
    suma=suma+bufferVel[k];
  }  
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if(interrupcion){
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
     t=cantOVerflow_actual*OCR2A;
     tmh=TCNT2actual-TCNT2anterior+t; // Ver problemas de variables
    bufferVel[2*cantMarcasEncoder-1]=long(preescaler)*(tmh); 
    suma=suma+ bufferVel[2*cantMarcasEncoder-1];
    freq=float(F_CPU*60)/(suma);
     BanderadelBuffer=true;
  }
  else{
     bufferVel[2*cantMarcasEncoder-1]=0;
     BanderadelBuffer=false;
     suma=suma+ bufferVel[2*cantMarcasEncoder-1];
     freq=0; // Hay que calcularla aca porque sino da cualquier valor.
  }   




// Pruebas. Para proximas modificaciones poner la version para la que funcionan.

  
  //aux[0]=freq;
  //aux[1]=tocc;//Envío el tiempo en el que se tomó la muestra
  //EnviarTX(2,'a',aux);
EnviarTX_online(freq);
//EnviarTX_online(tocc);

}

// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, unsigned long *datos){ // ojo, datos es un puntero
  
 // [inicio][cantidad de datos][identificador][Datos][fin] Agregar un CRC y ACK???

 if (online==false && tx_activada==true){ 

  Serial.println(0xFF,DEC); // inicio
  Serial.println(cantidad,DEC);// cantidad de datos
  Serial.println(identificador);// identificador, se puede sacar, pero para fines debuggeros puede ayudar
  float a;
for (int i=0;i<cantidad;i++){
  a=*(datos+i); // como datos es un puntero, necesito guardar en "a" el apuntador.
   Serial.println(a,DEC);
  }
  
  Serial.println(0xFE,DEC); // fin
 }
   
  }

void EnviarTX_online(float var){ // funcion de envio de datos de corta duracion. No se envia en formato trama, solo verifica una bandera.
  if (online==true && tx_activada==true){
  Serial.println(var,DEC);
  }
  }
// ##################### Estas 2 funciones hacen los mismo que en matlab :-)
  void tic()
  {
    ticc=micros();
  }

void toc()
  {
   tocc=micros()-ticc;    
    }



void PID_offline (void){ 

  /* La idea de esta funcion es que realice los calculos que no requieren que esten sincronizados.
   Luego, en la interrupcion por overflow, que e hagan solo los calculos necesarios y se aplique la señal de control.
   */
 
for(int k=0;k<2;k++)
  {
   error[k]=error[k+1];//Desplazamiento a la derecha de los datos del buffer
   u[k]=u[k+1];
  }  
error[2]=((float)(set_point)-freq)*(1.f);
u[2]=Parametros[0]*error[2]+Parametros[1]*error[1]+Parametros[2]*error[0]+(float)(Parametros[3]*u[1]+Parametros[4]*u[0]);
if (u[2]>100){u[2]=100;}
if (u[2]<0){u[2]=0;}
//EnviarTX_online(error[2]);
//EnviarTX_online(error[1]);
//EnviarTX_online(error[0]);

//EnviarTX_online(u[2]);

}
     
