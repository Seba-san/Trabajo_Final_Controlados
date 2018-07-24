//Ensayo al escalón
//Condiciones: frec de PID motores = 200 Hz, frec de PID total =200 Hz 

#define D 200 //cantidad de muestras para generar un delay
#define N 100  //cantidad de muestras a tomar en el ensayo al escalón
#define n0 40 //cantidad de muestras a la que se hace el cambio de dw=0 a dw=dW
#define dW 50  //Valor final del escalón

#include <EEPROM.h>
#include "includes.h"
Controlados controlados1;

// #   #   #   # Constantes $1
const int cantMarcasEncoder = 9; //Es la cantidad de huecos que tiene el encoder de cada motor.
const int FsEncoders = 400;//400;//2000;//8000 2000 // Esto significa Overflow cada 2Khz
const int preescaler = 1024;//1024;//32;//8 32 64
const int cota = 2000;//75;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
unsigned long _OCR2A;
// F_CPU es el valor con el que esta trabajando el clock del micro.

// #   #   #   # Variable Basura
int Bandera = 0; // bandera para administrar procesos fuera de interrupciones

//################

//Variables para el Ensayo al Escalón:
unsigned char sensor[N];//Mediciones de beta
float wA[N],wB[N];//Mediciones de velocidad ang deseada
int contador = 0, contador2 = 0, parar = 0,contadorStop=0,MaxStop=20;//MaxStop indica que si durante 20 muestras seguidas no detecta la línea tiene que detenerse
int enviar_datos = 0; //Bandera con la que Matlab le indica al nano que le devuelva el resultado del último ensayo al escalón
int Escribir = 0; //Le indico que escriba en la eeprom con un 1 y que lea con un 0
int controlador = 0, girar=0,grabar=0;
int softprescaler=0;//Lo uso para bajar la frec de muestreo de la señal de salida

// #   #   #   # Variables
unsigned char trama_activa = 0; //Lo pongo en unsigned char para que ocupe 1 byte (int ocupa 2)
bool online;//Me indica si poner o no el identificador para la transmisión (online=true no lo transmite para ahorrar tiempo)
bool tx_activada;//Me indica si transmitir o no.
bool iniciar = false; // le dice al micro cuando puede iniciar.
int PWMA; int PWMB; //Acá guardo los valores de nuevos de PWM que me mada Matlab para que actualice. La actualización efectiva se hace cuando tengo estos dos valores.
volatile unsigned long cantOVerflowA = 0, cantOVerflowB = 0; //Variable que almacena la cantidad de veces que se desbordó el timer2 hasta que vuelvo a tener interrupción por pin de entrada.
//Esto permite realizar la medición de tiempos entre agujeros del encoder.
volatile unsigned long TCNT2anteriorA = 0, TCNT2anteriorB = 0; //Valor anterior del contador (para corregir la medición)
volatile unsigned long TCNT2actualA = 0, TCNT2actualB = 0; //Almaceno el valor del timer para que no me jodan posibles actualizaciones.
volatile unsigned long cantOVerflow_actualA = 0, cantOVerflow_actualB = 0; //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.
unsigned long aux[6];  // este es un buffer para enviar datos en formato trama, corresponde a la funcion "EnviarTX"
float  bufferVelA[2 * cantMarcasEncoder], bufferVelB[2 * cantMarcasEncoder]; //buffer donde almaceno las últimas velocidades calculadas.
bool Motores_ON = false,controlador_motores=false;
int soft_prescaler = 0,cont_A,cont_B,delta_cont;
// Variables del PID
float uA[3], uB[3]; // historia del error cometido y la historia de las salidas de control ejecutadas.
float errorA[3], errorB[3];
float set_pointA, set_pointB; // Set_point esta en RPM
float wref = 350; //Velocidad lineal del centro del robot.
volatile float beta = 0; //Ángulo entre el eje central del robot y la línea (en radianes)
float dw[3] = {0, 0, 0}, errorBeta[3] = {0, 0, 0}; //Variación de velocidad angular.
volatile unsigned char byteSensor;//Byte del sensor de línea. Sirve para debuggear y para almacenar con menos bytes la información del sensor
byte vect_beta; // Variable de debuggeo
int bateria=0;
//Parametros PID: de las mediciones que habíamos hecho cuando hacíamos el ensayo con un sólo motor teníamos:
//PID andando medio pedorro={0.76184,-1.2174,0.48631,0,1};//PI andando={0.10679,-0.099861,0,1,0};

float ParametrosA[] = {0.053493,-0.050442,0,1,0};//Andando bien: {0.073817, -0.06814, 0, 1, 0};
float ParametrosB[] = {0.054997,-0.052071,0,1,0};//{0.077848, -0.072512, 0, 1, 0};
float Parametros[] = {1002.4094,9091.0267,526.9123,-36.3598,0};//{126.5571,-252.6673,126.1102,2,-0.99999};//{160.6358,-317.8668,157.2313,1.9999,-0.99992};//{159.3639,-317.8766,158.5127,2,-0.99998};//{181.7336,-360.803,179.0695,2,-0.99998};//{287.108,-573.8906,286.7826,2,-0.99999};//{191.8788,-383.6318,191.7531,2,-0.99997};////Este anda :D !!!!:{196.762,-393.4536,196.6916,1.9999,-0.99994};

volatile float freqA;
volatile float freqB;
int windup_top = 100, windup_bottom = 10;
int windup_top_dw = 500, windup_bottom_dw = -500; //Definir bien // Con 500 se garantiza un radio de giro de 25cm a 800 RPM

unsigned char estadoEncoder = 0; //En esta variable guardo el valor de las entradas de los encoders para identificar cuando se genera la interrupción cuál de los dos motores se movió

// #   #   #   # Declaracion de Funciones
void medirVelocidadA(unsigned char);
void medirVelocidadB(unsigned char);
void medirBeta(void);


void setup() { // $2
  interruptOFF; // se desactivan las interrupciones para configurar.

  Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  //Config pines de encoders:
  pinMode(14, INPUT);//A0 = pin 14 del nano
  pinMode(15, INPUT);//A1 = pin 15 del nano

  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();
  controlados1.configTimer2Contador(FsEncoders, preescaler, 1); //Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.
  controlados1.actualizarDutyCycleMotores(0, 0);

  //Antes de prender los motores guardo el valor de los encoderes en la variable estadoEncoder para la interrupción por flanco
  int encoderAux;
  encoderAux = bitRead(PINC, 0);
  bitWrite(estadoEncoder, 0, encoderAux);
  encoderAux = bitRead(PINC, 1);
  bitWrite(estadoEncoder, 1, encoderAux);

  set_pointA = wref;
  set_pointB = wref;
  _OCR2A = OCR2A;
  interruptON;//Activo las interrupciones

  pinMode(13, INPUT);//Uso el pin del LED para ver qué hacer, si escribir o no
  if (digitalRead(13)) {
    Escribir = 1;
    delay(1000);
  }
  if (Escribir) {
    controlados1.modoAdelante(); //Sólo prendo el motor si voy a escibir las mediciones en la EEPROM
    //Escribir=0;// $.$ PARA NO ESCRIBIR AL PEDO LA EEPROM SI NO LA VOY A LEER
  }
}

void loop() { //$3
  //Bateria(); // Chequeo el estado de la bateria, si es menor que un valor, se queda aca dentro.
  if (bitRead(Bandera, 0)) {
    bitWrite(Bandera, 0, 0); // timer 2 overflow
  }
  if (bitRead(Bandera, 1)) {
    bitWrite(Bandera, 1, 0); // Entra cuando no registra cambio en la entrada A
    medirVelocidadA(0);
  }
  if (bitRead(Bandera, 2)) {
    bitWrite(Bandera, 2, 0); // se registra cambio en la entrada A
    medirVelocidadA(1);
  }
  if (bitRead(Bandera, 3)) {
    bitWrite(Bandera, 3, 0); // Entra cuando no registra cambio en la entrada B
    medirVelocidadB(0);
  }
  if (bitRead(Bandera, 4)) {
    bitWrite(Bandera, 4, 0); // se registra cambio en la entrada B
    medirVelocidadB(1);
  }
  if (bitRead(Bandera, 5)) {
    bitWrite(Bandera, 5, 0);
    medirBeta();//Actualizo la medición de ángulo
    
//    softprescaler++;
//    if (softprescaler==1){//Ya no submuestreo//Tomo una de cada 4 muestras para una frec de muestreo del PID total de 50 Hz (PID motores a 200 Hz)
//        softprescaler=0;
//    }
//      if (controlador==1 && contador<n0 && girar==1) {
//        PID_total();
//        set_pointA = wref - dw[2]/2.0;
//        set_pointB = wref + dw[2]/2.0;
//      }
      //else if (controlador==1 && girar==0){//Si no quiero girar entonces siempre calcula el controlador
      if (controlador==1 ){//Si no quiero girar entonces siempre calcula el controlador
        PID_total();
        set_pointA = wref - dw[2]/2.0;//*(wref/350);//*(wref/500.0);
        set_pointB = wref + dw[2]/2.0;//*(wref/350);//*(wref/500.0);
      }
  //}
    //$.$
//    Serial.print(dw[2]);
//    Serial.print(" ");
  //Serial.println(byteSensor);
  
 // Serial.write(byteSensor);
 /*byte addi;
   addi= freqA*255.0/1000.0;
    Serial.write(addi);
    addi= freqB*255.0/1000.0;
    Serial.println(addi);
    */
//freqB=freqB+500;
//Serial.println(byteSensor,BIN);
//Serial.println(' ');
//Serial.println(byteSensor);
//Serial.println(' ');
//Serial.print(cont_A);
//Serial.print(' ');
//Serial.println(cont_B);
  EnviarTx_blue();
  /* byte addi=0;
  
  Serial.write(0xFF); // inicio
  Serial.write(byteSensor);
 addi= freqA*255.0/1000.0;
  Serial.write(addi);
 addi=freqB*255.0/1000.0;
  Serial.write(addi);
  */
  
//    
    PID_offline_Motores();
//    if (parar == 1) { //Perdí la línea
//      //controlados1.modoStop();//Paro los motores, pero sigo midiendo
//    }
//    if (contador2 < D) {
//      //if(beta<3){contador2++;}//Le pongo el if para que siga derecho hasta estar sobre la línea
//      contador2++;
//    }
//    else {  //Si controlador=1 empieza a tomar muestras sin delay
//      if (contador < N) {
//        //$.$ Lo dejo que tome muestras a 200Hz. Ojo que ahora softprescaler lo uso en el PID total!! No reactivar lo de abajo sin cambiar eso!!!
//        //softprescaler++;
//        //if (softprescaler==4){//Tomo una de cada 4 muetsras
//        //softprescaler=0;
//        sensor[contador] = byteSensor; //Guardo la medición de ángulo
//        //wA[contador] = set_pointA; //Guardo la medición de velocidad deseada del motor A
//        //wB[contador] = set_pointB; //Guardo la medición de velocidad deseada del motor B
//        wA[contador] = freqA; //Guardo la medición de velocidad real del motor A
//        wB[contador] = freqB; //Guardo la medición de velocidad real del motor B
//        //$.$
//        //wA[contador]=freqB-freqA;//Guardo la medición de velocidad del motor A
//        //wB[contador]=set_pointB-set_pointA;//Guardo la medición de velocidad del motor B
//        contador++;//Aumento el índice de las muestras          
//        //}
//        if (contador == n0 && girar==1){// && controlador == 0) {
//          
//          set_pointA = wref - dW;
//          set_pointB = wref + dW;
//        }
//      }
//      else {
//        //controlados1.modoStop();//Paro los motores//Esto creo que es redundante, pero por si acaso
//        if (Escribir==1 && grabar==1) {//Grabo en la EEPROM
//          Escribir=0;//No vuelve a grabar
//          int addr = 0;
//          if(controlador==0){//Sin controlador almaceno sólo el ángulo
//            for (int ind=0; ind < N; ind++) {
//              EEPROM.put(addr, sensor[ind]);//put escribe cualquier cosa, incluido float
//              addr = addr + 1;
//            }
//          }
//          else{//Con controlador almaceno el ángulo y los setpoints
//            
//            for (int ind=0; ind < N; ind++) {
//              EEPROM.put(addr, sensor[ind]);//put escribe cualquier cosa, incluido float
//              addr = addr + 1;
//              EEPROM.put(addr, wA[ind]);
//              addr = addr + 4;//Sumo 4 porque la variable es float
//              EEPROM.put(addr, wB[ind]);
//              addr = addr + 4;
//              Serial.println(ind,DEC);
//            }
//          }
//        }
//      }
//    }
//  }
//  if (enviar_datos == 1) { //La compu me pidió que le envíe los resultados del ensayo
//    enviar_datos = 0; //Para que lo transmita una sola vez
//    int eeAddress = 0;
//    online = false;
//    tx_activada = true; //Para ahorrar problemas fuerzo las banderas, así no lo tengo que hacer desde Matlab
//    for (int ind=0; ind < N; ind++) {
//      if(controlador==0){
//        //Piso la variable auxiliar sensor con lo que haya escrito en la eeprom
//        EEPROM.get(eeAddress, sensor[ind]); //Para leer cualquier tipo de variable uso get en vez de read
//        eeAddress = eeAddress + 1;
//      }
//      else{
//        EEPROM.get(eeAddress, sensor[ind]);
//        eeAddress = eeAddress + 1;
//        EEPROM.get(eeAddress, wA[ind]);
//        eeAddress = eeAddress + 4;
//        EEPROM.get(eeAddress, wB[ind]);
//        eeAddress = eeAddress + 4;        
//      }
//    }
//    EnviarTX(N, 'b', sensor);
//    if(controlador==1){
//        EnviarTX(N, 'A', wA);
//        EnviarTX(N, 'B', wB);
//    }
  }
}

void medirVelocidadA(unsigned char interrupcionA)
{
  long suma = 0;
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:
  for (int k = 0; k < (2 * cantMarcasEncoder - 1); k++)
  {
    bufferVelA[k] = bufferVelA[k + 1]; //Desplazamiento a la derecha de los datos del buffer
    suma = suma + bufferVelA[k];
  }
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if (interrupcionA) {
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
    interruptOFF; // Es importante poner esto antes de hacer el calculo para evitar que modifique las variables en la interrupcion. Se observaron problemas de medicion.
    bufferVelA[2 * cantMarcasEncoder - 1] = (long)(preescaler) * (TCNT2actualA - TCNT2anteriorA + cantOVerflow_actualA * _OCR2A);
    suma = suma + bufferVelA[2 * cantMarcasEncoder - 1];
    freqA = float((F_CPU * 60.0) / (suma));
    interruptON;
  }
  else {
    bufferVelA[2 * cantMarcasEncoder - 1] = 0;
    suma = suma + bufferVelA[2 * cantMarcasEncoder - 1];
    freqA = 0; // Hay que calcularla aca porque sino da cualquier valor.
  }
}

void medirVelocidadB(unsigned char interrupcionB)
{
  long suma = 0;
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:
  for (int k = 0; k < (2 * cantMarcasEncoder - 1); k++)
  {
    bufferVelB[k] = bufferVelB[k + 1]; //Desplazamiento a la derecha de los datos del buffer
    suma = suma + bufferVelB[k];
  }
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if (interrupcionB) {
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
    interruptOFF; // Es importante poner esto antes de hacer el calculo para evitar que modifique las variables en la interrupcion. Se observaron problemas de medicion.
    bufferVelB[2 * cantMarcasEncoder - 1] = (long)(preescaler) * (TCNT2actualB - TCNT2anteriorB + cantOVerflow_actualB * _OCR2A);
    suma = suma + bufferVelB[2 * cantMarcasEncoder - 1];
    freqB = float((F_CPU * 60.0) / (suma));
    interruptON;
  }
  else {
    bufferVelB[2 * cantMarcasEncoder - 1] = 0;
    suma = suma + bufferVelB[2 * cantMarcasEncoder - 1];
    freqB = 0; // Hay que calcularla aca porque sino da cualquier valor.
  }
}

void medirBeta(void) {
  float betaAux;
  betaAux = controlados1.leerSensorDeLinea(&byteSensor);
  //Si beta=3 es porque el sensor tiró un valor erróneo o perdió la línea.
  //En ese caso mantengo el valor anterior medido. Por eso sólo actualizo beta si la rutina NO devuelve un 3.
  if (betaAux == 3) {
    contadorStop++;
    if(contadorStop>MaxStop){parar=1;}//$.$
  }//Aviso que pare porque perdió la línea
  else{ //$.$ Nota: ahora no estaría marcando cuando pierde la línea, sino que mantiene la medición anterior
    contadorStop=0;
    beta = betaAux;
  }
}
