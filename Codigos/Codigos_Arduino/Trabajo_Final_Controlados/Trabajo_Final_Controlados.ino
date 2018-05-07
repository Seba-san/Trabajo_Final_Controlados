#include "includes.h"

Controlados controlados1;


// #   #   #   # Constantes
const int cantMarcasEncoder = 8; //Es la cantidad de huecos que tiene el encoder de cada motor.
const int FsEncoders = 2000;//400;//2000;//8000 2000 // Esto significa Overflow cada 2Khz
const int preescaler = 32;//1024;//32;//8 32 64
const int cota = 200;//75;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
 unsigned long _OCR2A ;
// F_CPU es el valor con el que esta trabajando el clock del micro.
const float k0=9.7419;


bool estado=0;

// #   #   #   # Variable Basura
int Bandera=0; // bandera para administrar procesos fuera de interrupciones
float b;
unsigned long cuenta;

//################

// #   #   #   # Variables
unsigned long ticc,tocc;
unsigned char trama_activa=0;//Lo pongo en unsigned char para que ocupe 1 byte (int ocupa 2)
bool online;//Me indica si poner o no el identificador para la transmisión (online=true no lo transmite para ahorrar tiempo)
bool tx_activada;//Me indica si transmitir o no.
bool iniciar=false; // le dice al micro cuando puede iniciar.
int PWMA;int PWMB;//Acá guardo los valores de nuevos de PWM que me mada Matlab para que actualice. La actualización efectiva se hace cuando tengo estos dos valores.

/*
int TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
int TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.
int cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.
*/
volatile unsigned long cantOVerflow=0;//Variable que almacena la cantidad de veces que se desbordó el timer2 hasta que vuelvo a tener interrupción por pin de entrada. Esto permite realizar la medición de
                  //tiempos entre aujeros del encoder.
volatile unsigned long TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
volatile unsigned long TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.
volatile unsigned long cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.
unsigned long aux[6];  // este es un buffer para enviar datos en formato trama, corresponde a la funcion "EnviarTX"
float  bufferVel[2*cantMarcasEncoder];//buffer donde almaceno las últimas velocidades calculadas. // ANtes era unsigned long
bool BanderadelBuffer=true;
bool Motores_ON=false;
float velAngular=0;//última velocidad angular calculada. Lo separo del vector bufferVel para que no haya problemas por interrumpir en medio de una actualización de éste.
float velDeseada=0;//Empieza detenido el motor.
int soft_prescaler=0;
// Variables del PID
float u[3]; // historia del error cometido y la historia de las salidas de control ejecutadas.
float error[3];
float set_point=0; // Set_point esta en RPM
float Parametros[]={ 0.162651, 0, 0, 0, -0.000000};//{1.261400, -2.522133, 1.260733, 1.999500, -0.999500};

volatile float freq;
int windup_top=1000,windup_bottom=100;


// # # # # # Variables prueba

int * puntero;
int indice=0;


// #   #   #   # Declaracion de Funciones

void medirVelocidad(unsigned char);
//void EnviarTX(int cantidad,char identificador, unsigned long *datos);
//void EnviarTX_online(unsigned long var);
//void timer_interrupt(void);
//void interrupt_flanco(void);
/*
void PID_offline(void);

void tic(void);
void toc(void);
*/


void setup() { // $2
  interruptOFF; // se desactivan las interrupciones para configurar.

Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)
//Serial.begin(1000000);

while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
 controlados1.configPinesMotores();
 controlados1.modoStop();
 controlados1.configTimerMotores();
 controlados1.configTimer2Contador(FsEncoders,preescaler,1);//Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.
 controlados1.actualizarDutyCycleMotores(0,0);
 controlados1.modoAdelante();
 _OCR2A=OCR2A;
  interruptON;//Activo las interrupciones
  pinMode(A0, INPUT);
  pinMode(SalidaTest,OUTPUT);
  pinMode(SalidaTest2,OUTPUT);
  pinMode(SalidaTest3,OUTPUT);
  tic();
}

void loop() { //$3
if (bitRead(Bandera,0)){ bitWrite(Bandera,0,0);// timer 2 overflow

  }
  if (bitRead(Bandera,1)){ bitWrite(Bandera,1,0); // Entra cuando no registra cambio en la entrada
  // Serial.println  (Bandera,BIN);

  medirVelocidad(0);

  }
  if (bitRead(Bandera,2)){  bitWrite(Bandera,2,0);// se registra cambio en la entrada
  medirVelocidad(1);
  }
   if (bitRead(Bandera,3)){ bitWrite(Bandera,3,0); // Se midio un tiempo de 15mS, se realiza el calculo del PID
  //EnviarTX_online(freq);
  //EnviarTX_online((float)tocc);
  PID_offline(); // $VER, analizar esto, porque va a entrar varias veces (entre 8 y 9 o mas) antes de tener una nueva medida de las RPM
  // Si no me equivoco lo mejor seria tomar muestras a 66Hz (considerando 500RPM como minimo) eso da 15mS de Ts.
  }
}


void medirVelocidad(unsigned char interrupcion)
{ // $7
  //digitalWrite(SalidaTest3,HIGH);
long suma=0;
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
    interruptOFF; // Es importante poner esto antes de hacer el calculo para evitar que modifique las variables en la interrupcion. Se observaron problemas de medicion.
   /*  t=cantOVerflow_actual*_OCR2A;
     tmh=TCNT2actual-TCNT2anterior+t; // Ver problemas de variables
    bufferVel[2*cantMarcasEncoder-1]=long(preescaler)*(tmh);
*/
     bufferVel[2*cantMarcasEncoder-1]=(long)(preescaler)*(TCNT2actual-TCNT2anterior+cantOVerflow_actual*_OCR2A);
    suma=suma+ bufferVel[2*cantMarcasEncoder-1];
    freq=float((F_CPU*60.0)/(suma));
    interruptON;
     BanderadelBuffer=true;

  }
  else{
     bufferVel[2*cantMarcasEncoder-1]=0;
     BanderadelBuffer=false;
     suma=suma+ bufferVel[2*cantMarcasEncoder-1];
     freq=0; // Hay que calcularla aca porque sino da cualquier valor.
  }

/*
if (abs(bufferVel[2*cantMarcasEncoder-1]+bufferVel[2*cantMarcasEncoder-2]-160000)>1000)
{
aux[0]=bufferVel[2*cantMarcasEncoder-1];
aux[1]=_OCR2A;
aux[2]=cantOVerflow_actual;
aux[3]=TCNT2anterior;
aux[4]=TCNT2actual;
 EnviarTX(5,'a',aux);
}
*/
// Pruebas. Para proximas modificaciones poner la version para la que funcionan.


  //aux[0]=freq;
  //aux[1]=tocc;//Envío el tiempo en el que se tomó la muestra
  //EnviarTX(2,'a',aux);
//EnviarTX_online(freq);
//EnviarTX_online((float)tocc);
//EnviarTX_online(bufferVel[2*cantMarcasEncoder-1]);

//digitalWrite(SalidaTest3,LOW);
 //indice=int(freq);
EnviarTX_online(freq);
//EnviarTX_online(suma);

}
