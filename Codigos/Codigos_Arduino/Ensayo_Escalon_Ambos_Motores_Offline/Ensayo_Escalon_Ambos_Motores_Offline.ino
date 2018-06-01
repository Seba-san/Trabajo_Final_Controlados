#define D 300 //cantidad de muestras para generar un delay
#define N 200  //cantidad de muestras a tomar en el ensayo al escalón
#define n0 20 //cantidad de muestras a la que se hace el cambio de velocidad (van a haber n0 muestras a PWM1 y la n0+1 será parte del transitorio)


#include <EEPROM.h>
#include "includes.h"
Controlados controlados1;

// #   #   #   # Constantes $1
const int cantMarcasEncoder = 9; //Es la cantidad de huecos que tiene el encoder de cada motor.
const int FsEncoders = 400;//400;//2000;//8000 2000 // Esto significa Overflow cada 2Khz
const int preescaler = 1024;//1024;//32;//8 32 64
const int cota = 200;//75;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
unsigned long _OCR2A;
int controlador=1;
// F_CPU es el valor con el que esta trabajando el clock del micro.

// #   #   #   # Variable Basura
int Bandera=0; // bandera para administrar procesos fuera de interrupciones

//################

//Variables para el Ensayo al Escalón:
float w[N];//Vector para guardar las mediciones del ensayo
int contador=0,contador2=0;
int enviar_datos=0;//Bandera con la que Matlab le indica al nano que le devuelva el resultado del último ensayo al escalón
int PWM1=40;//Valor inicial del escalón
int PWM2=80;//Valor final del escalón
int Escribir=0;//Le indico que escriba en la eeprom con un 1 y que lea con un 0
float inicio=400;//Velocidad inicial en rpm cuando el controlador está activado
float fin=800;//Velocidad final en rpm cuando el controlador está activado


// #   #   #   # Variables
unsigned char trama_activa=0;//Lo pongo en unsigned char para que ocupe 1 byte (int ocupa 2)
bool online;//Me indica si poner o no el identificador para la transmisión (online=true no lo transmite para ahorrar tiempo)
bool tx_activada;//Me indica si transmitir o no.
bool iniciar=false; // le dice al micro cuando puede iniciar.
int PWMA;int PWMB;//Acá guardo los valores de nuevos de PWM que me mada Matlab para que actualice. La actualización efectiva se hace cuando tengo estos dos valores.
volatile unsigned long cantOVerflowA=0,cantOVerflowB=0;//Variable que almacena la cantidad de veces que se desbordó el timer2 hasta que vuelvo a tener interrupción por pin de entrada. 
                                                       //Esto permite realizar la medición de tiempos entre agujeros del encoder.
volatile unsigned long TCNT2anteriorA=0,TCNT2anteriorB=0;//Valor anterior del contador (para corregir la medición)
volatile unsigned long TCNT2actualA=0,TCNT2actualB=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.
volatile unsigned long cantOVerflow_actualA=0,cantOVerflow_actualB=0;//Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.
unsigned long aux[6];  // este es un buffer para enviar datos en formato trama, corresponde a la funcion "EnviarTX"
float  bufferVelA[2*cantMarcasEncoder],bufferVelB[2*cantMarcasEncoder];//buffer donde almaceno las últimas velocidades calculadas.
bool Motores_ON=false;
int soft_prescaler=0;
// Variables del PID
float uA[3],uB[3]; // historia del error cometido y la historia de las salidas de control ejecutadas.
float errorA[3],errorB[3];
float set_pointA=inicio,set_pointB=inicio; // Set_point esta en RPM
float ParametrosA[]={0.12012,-0.11435,0,1,0};//PID andando medio pedorro={0.76184,-1.2174,0.48631,0,1};//PI andando={0.10679,-0.099861,0,1,0};
float ParametrosB[]={0.12012,-0.11435,0,1,0};

volatile float freqA;
volatile float freqB;
int windup_top=100,windup_bottom=10;

unsigned char estadoEncoder=0;//En esta variable guardo el valor de las entradas de los encoders para identificar cuando se genera la interrupción cuál de los dos motores se movió

// #   #   #   # Declaracion de Funciones
void medirVelocidadA(unsigned char);
void medirVelocidadB(unsigned char);


void setup() { // $2
  interruptOFF; // se desactivan las interrupciones para configurar.

  Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println("Inicio");//$$$BORRAR

  //Config pines de encoders:
  pinMode(14, INPUT);//A0 = pin 14 del nano
  pinMode(15, INPUT);//A1 = pin 15 del nano
  
 controlados1.configPinesMotores();
 controlados1.modoStop();
 controlados1.configTimerMotores();
 controlados1.configTimer2Contador(FsEncoders,preescaler,1);//Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.
 controlados1.actualizarDutyCycleMotores(PWM1,PWM1);
 
 //Antes de prender los motores guardo el valor de los encoderes en la variable estadoEncoder para que cuando se genere la primer interrupción por
 int encoderAux;
 encoderAux=bitRead(PINC,0);
 bitWrite(estadoEncoder,0,encoderAux);
 encoderAux=bitRead(PINC,1);
 bitWrite(estadoEncoder,0,encoderAux);

  
  _OCR2A=OCR2A;
  interruptON;//Activo las interrupciones
  
  pinMode(13, INPUT);//Uso el pin del LED para ver qué hacer, si escribir o no
  if (digitalRead(13)){ Escribir=1;delay(1000);}
  if(Escribir){controlados1.modoAdelante();}//Sólo prendo el motor si voy a escibir las mediciones en la EEPROM
}

void loop() { //$3
  if (bitRead(Bandera,0)){bitWrite(Bandera,0,0);// timer 2 overflow
  }
  if (bitRead(Bandera,1)){bitWrite(Bandera,1,0); // Entra cuando no registra cambio en la entrada A
  medirVelocidadA(0);
  }
  if (bitRead(Bandera,2)){bitWrite(Bandera,2,0);// se registra cambio en la entrada A
  medirVelocidadA(1);
  }
  if (bitRead(Bandera,3)){bitWrite(Bandera,3,0); // Entra cuando no registra cambio en la entrada B
  medirVelocidadB(0);
  }
  if (bitRead(Bandera,4)){bitWrite(Bandera,4,0);// se registra cambio en la entrada B
  medirVelocidadB(1);
  }
  if (bitRead(Bandera,5)){bitWrite(Bandera,5,0);
  PID_offline();
  if(contador2<D){
    contador2++;
  }
  else{
    if (contador<N){
      //w[contador]=freqA;//Guardo la medición de velocidad del motor A
      w[contador]=freqB;//Guardo la medición de velocidad del motor B
      contador++;//Aumento el índice de las muestras
      if(contador==n0){//Inicio el escalón
        if(controlador){
          set_pointA=fin;
          set_pointB=fin;
        }
        else{
        controlados1.actualizarDutyCycleMotores(PWM2,PWM2);
        }
      }
    }
    else{
      controlados1.modoStop();//Paro los motores
      delay(1000);
      //Grabo en la EEPROM
      if(Escribir){
        int addr=0;
        for(int ind;ind<N;ind++){
          EEPROM.put(addr, w[ind]);//put escribe cualquier cosa, incluido float
          addr = addr + 4;//La siguiente escritura debería estar 4 bytes después (porque un float ocupa 4 bytes)
          //Ver si necesito esto:
          //if (addr == EEPROM.length()){addr = 0;}
        }
      }
    }
  }
  }
  if(enviar_datos==1){//La compu me pidió que le envíe los resultados del ensayo
    enviar_datos=0;//Para que lo transmita una sola vez
    int eeAddress=0;
        for(int ind;ind<N;ind++){
          //Piso la variable auxiliar w con lo que haya escrito en la eeprom
          EEPROM.get(eeAddress,w[ind]);//Para leer cualquier tipo de variable uso get en vez de read
          eeAddress = eeAddress + 4;//La siguiente escritura debería estar 4 bytes después (porque un float ocupa 4 bytes)
        }
    online=false;
    tx_activada=true; //Para ahorrar problemas fuerzo las banderas, así no lo tengo que hacer desde Matlab
    EnviarTX(N,'A',w);
  }
}

void medirVelocidadA(unsigned char interrupcionA)
{
long suma=0;
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:
  for(int k=0;k<(2*cantMarcasEncoder-1);k++)
  {
    bufferVelA[k]=bufferVelA[k+1];//Desplazamiento a la derecha de los datos del buffer
    suma=suma+bufferVelA[k];
  }
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if(interrupcionA){
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
    interruptOFF; // Es importante poner esto antes de hacer el calculo para evitar que modifique las variables en la interrupcion. Se observaron problemas de medicion.
    bufferVelA[2*cantMarcasEncoder-1]=(long)(preescaler)*(TCNT2actualA-TCNT2anteriorA+cantOVerflow_actualA*_OCR2A);
    suma=suma+ bufferVelA[2*cantMarcasEncoder-1];
    freqA=float((F_CPU*60.0)/(suma));
    interruptON;
  }
  else{
     bufferVelA[2*cantMarcasEncoder-1]=0;
     suma=suma+ bufferVelA[2*cantMarcasEncoder-1];
     freqA=0; // Hay que calcularla aca porque sino da cualquier valor.
  }
}

void medirVelocidadB(unsigned char interrupcionB)
{
long suma=0;
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:
  for(int k=0;k<(2*cantMarcasEncoder-1);k++)
  {
    bufferVelB[k]=bufferVelB[k+1];//Desplazamiento a la derecha de los datos del buffer
    suma=suma+bufferVelB[k];
  }
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if(interrupcionB){
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
    interruptOFF; // Es importante poner esto antes de hacer el calculo para evitar que modifique las variables en la interrupcion. Se observaron problemas de medicion.
    bufferVelB[2*cantMarcasEncoder-1]=(long)(preescaler)*(TCNT2actualB-TCNT2anteriorB+cantOVerflow_actualB*_OCR2A);
    suma=suma+ bufferVelB[2*cantMarcasEncoder-1];
    freqB=float((F_CPU*60.0)/(suma));
    interruptON;
  }
  else{
     bufferVelB[2*cantMarcasEncoder-1]=0;
     suma=suma+ bufferVelB[2*cantMarcasEncoder-1];
     freqB=0; // Hay que calcularla aca porque sino da cualquier valor.
  }
}
