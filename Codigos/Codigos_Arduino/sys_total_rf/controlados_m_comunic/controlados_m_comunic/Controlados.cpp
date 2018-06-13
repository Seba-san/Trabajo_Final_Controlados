/******************************************************************************
Librería para el Trabajo Final de Controlados
******************************************************************************/
//Esto parece que va según una página de por ahí
//(https://www.arduino.cc/en/Hacking/LibraryTutorial),
//pero ni idea por qué ni si está bien:

#include "Controlados.h"
#include <Arduino.h>

//Formato del define: #define NombreVariable Valor
//PONER LOS NROS DE PINES!!!!
#define In1A 6
#define In2A 5
#define In1B 8
#define In2B 12
#define STBY 7
#define tope 0//Este valor va de 0 a 1 y me indica el dutycycle máx permitido
                //Como estoy trabajando en modo invertido los PWM, pongo _TOP*tope
				//como valor mínimo. Así, para un dutycycle máx de 80%, tope debe
				//valer 0.2.
//Pines de los receptores del sensor de línea:
//Obs: no supe cómo hacerlo como vector para que sea más elegante el código y no
//quise perder tiempo en eso. Probablemente pueda mejorarse.
//$REVISAR QUE ESTÉN BIEN
#define rx_1 16
#define port1 PINC
#define bit1 2
#define rx_2 17
#define port2 PINC
#define bit2 3
#define rx_3 18
#define port3 PINC
#define bit3 4
#define rx_4 19
#define port4 PINC
#define bit4 5
//ESTE PIN ES ANALÓGICO!!!$ (por eso le pongo PORTC)
#define rx_5 A6
#define port5 PORTC
#define bit5 6
#define rx_6 2
#define port6 PIND
#define bit6 2
#define rx_7 3
#define port7 PIND
#define bit7 3
#define rx_8 4
#define port8 PIND
#define bit8 4

Controlados::Controlados()
{
	_timerMotores=0;//Esto lo hago para que actualizarDutyCycleMotores no haga
	                // nada si todavía no configuré algún timer.

	//Defino como salidas los pines que controlan el modo de funcionamiento
	//de los motores:
	pinMode(In1A,OUTPUT);     pinMode(In1B,OUTPUT);
	pinMode(In2A,OUTPUT);     pinMode(In2B,OUTPUT);
}

void Controlados::configPinesMotores()
{
	pinMode(In1A,OUTPUT);
	pinMode(In2A,OUTPUT);
	pinMode(In1B,OUTPUT);
	pinMode(In2B,OUTPUT);
	pinMode(STBY,OUTPUT);
	digitalWrite(STBY,HIGH);//Dejo los motores habilitados
}

void Controlados::configTimerMotores()
{
	//Se configura el TIMER1 para trabajar en modo Phase correct PWM, con una
	//frecuencia fija de 15,656kHz, con un conteo inicial en 0, SIN
	//interrupciones.
	//El modo Phase Correct lo elijo porque el datasheet dice que es mejor para
	//controlar motores que el FastPWM (ni idea por qué), y que es casi lo mismo
	//que el Phase and Frequency Correct si el TOP va a quedar estático.

	//_timerMotores le indica a actualizarDutyCycleMotores cuál es el timer
	//que estoy usando para los motores. Usaremos siempre el timer 1, pero sirve
	//para indicar si ya se configuró o no el timer.
	_timerMotores=1;

	//Para que el PWM esté disponible en las salidas correspondientes,
	//habilito los pines como salida.
	pinMode(9, OUTPUT);//OC1A = pin 9 del nano
	pinMode(10, OUTPUT);//OC1B = pin 10 del nano

	//TC1 Control Register A: Los bits 4 a 7 configuran el modo de la salida,
	//pero el datasheet no es claro en cuáles son los bits (dice en todos
	//COM1, pero deberían ser COM1A1, COM1A0, COM1B1,COM1B0). Debido a eso,
	//pongo salida en modo invertido (todos en 1), ya que sin invertir sería
	//1010, pero no logro saber en qué orden.
	//Los bits 3 y 2 no hacen nada, mientras que los bit 1 y 0 son WGM11 y
	//WGM10 para seleccionar el modo del timer.
	//Por lo que entiendo, parece que puedo usar 8, 9 o 10 bits para la
	//modalidad que necesito. Yo selecciono el modo de trabajo nro 2 de
	//la tabla del datasheet, ya que me da más precisión sin ser tan lento.
	//Para seleccionar ese modo necesito WGM1[3:0]=0010.
	TCCR1A = 0b11110010;
	//Si uso 9 bits, TOP=0x01FF=511. Guardo este valor para que la función
	//actualizarDutyCycleMotores sepa el máximo permitido para OCR1x.
	_TOP=511;

	//TC1 Control Register B: bit7=ICNC1 y bit6=ICES1 están asociados al pin
	//de captura, por lo que no me interesan. El bit5 no hace nada.
	//Los bit 4 y 3 son WGM13 y WGM12, por lo que los pongo como 0 para usar
	//el modo 2 explicado antes.
	//Los bits 2 a 0 son CS1[2:0], que son los Clock Select. Selecciono para
	//usar clock source interno sin preescalador (N=1) tomando CS1[2:0]=001
	//(el datasheet parece indicar que 011 es lo mismo). La frec resultante
	//es:
	//              f=fclkio/(2*N*TOP)=16MHz/(2*511)=15,656kHz
	//A modo de referencia, con 10 bits la frec máxima posible es 7820,14Hz,
	//mientras que con 8 bits es 31,37kHz.
	TCCR1B=0b00000001;

	//Pongo el timer en 0 para el inicio del conteo:
	TCNT1 = 0x0000;

	//Dutycycles iniciales de 50% aprox:
	OCR1A=255;
	OCR1B=255;

	//Deshabilito las interrupciones por timer1:
	TIMSK1=0x00;
	//Bajo las banderas del timer1 (en realidad no es necesario si no hay
	//interrupciones habilitadas, pero que quede):
	TIFR1=0x00;

}

void Controlados::actualizarDutyCycleMotores(int dutyCycleA,int dutyCycleB)
{
	//Esta subrutina actualiza el dutyCyclex del timer indicado por la
	//variable global _timerMotores. Para ello las variables dutyCyclex deben
    //ser números entre 0 y 100. No obstante, si se exceden, la función los
    //trunca al límite excedido.
	//Se requiere previamente haber configurado el timer correspondiente con
	//configTimerMotores, por eso se usa la variable _timerMotores. Como ésta
	//última arranca en 0, si nunca se llamó a configTimerMotores,
	//actualizarDutyCycleMotores no hace nada, mientras que al llamar a
	//configTimerMotores se cambia _timerMotores al valor que corresponda.

	int valor;

	if (_timerMotores==1)
	{
		dutyCycleA=100-dutyCycleA;//Acomodo porque según la configuración
								  // estoy trabajando en modo inverso.
		dutyCycleB=100-dutyCycleB;

		//Para convertir uso la función map de arduino. La sintáxis de la
		//misma es: map(value,fromLow,fromHigh,toLow,toHigh), y mapea
		//linealmente (creo) cambiando del rango fromLow-fromHigh al rango
		//toLow-toHigh. La fc devuelve enteros por truncamiento (no redondeo)
		//y no se limita a devolver números dentro del rango. Por esto último
		//usamos después constrain.
		valor=map(dutyCycleA,0,100,0,_TOP);//OCR1x debe moverse de 0 a _TOP
		OCR1A=constrain(valor,_TOP*tope,_TOP);//Si me paso del rango, trunca el
										      //nro.
		valor=map(dutyCycleB,0,100,0,_TOP);
		OCR1B=constrain(valor,_TOP*tope,_TOP);
	}
}



//Tabla de funcionamiento del motor:
//In1  In2  PWM  Out1  Out2  Mode
// H    H    H/L   L     L   Short brake
// L    H    H     L     H   CCW (counterclock wise)
// L    H    L     L     L   Short brake
// H    L    H     H     L   CW (clock wise)
// H    L    L     L     L   Short brake
// L    L    H    OFF   OFF  Stop

//Defino CCW como adelante.
void Controlados::modoAdelante()
{
	//Esta rutina configura ambos motores para funcionar CCW de modo simultáneo
	digitalWrite(In1A,LOW);   digitalWrite(In1B,LOW);
	digitalWrite(In2A,HIGH);  digitalWrite(In2B,HIGH);
}
void Controlados::modoAtras()
{
	//Esta rutina configura ambos motores para funcionar CW de modo simultáneo
	digitalWrite(In1A,HIGH);  digitalWrite(In1B,HIGH);
	digitalWrite(In2A,LOW);   digitalWrite(In2B,LOW);
}
void Controlados::modoStop()
{
	//Esta rutina detiene ambos motores
	digitalWrite(In1A,LOW);  digitalWrite(In1B,LOW);
	digitalWrite(In2A,LOW);   digitalWrite(In2B,LOW);
}

void Controlados::configPinesSensorLinea()
{
	pinMode(rx_1,INPUT_PULLUP);
	pinMode(rx_2,INPUT_PULLUP);
	pinMode(rx_3,INPUT_PULLUP);
	pinMode(rx_4,INPUT_PULLUP);
	pinMode(rx_5,INPUT_PULLUP);
	pinMode(rx_6,INPUT_PULLUP);
	pinMode(rx_7,INPUT_PULLUP);
	pinMode(rx_8,INPUT_PULLUP);
}

float Controlados::leerSensorDeLinea(unsigned char* byteSensor)
{
	//Esta función lee el sensor de línea y determina en función de eso un valor
	//para el ángulo de error etre el robot y la línea. Para eso supone que el
	//centro del robot (punto medio entre las dos ruedas) está siempre encima de
	//la línea.
  //Si se perdió la línea, devuelve beta=3 para que desde afuera de la función
  //se lo tome como no valido. En este caso, desde el main se debe indicar que
  //hay que mantener el valor anterior.
  
  //Para definir beta vamos a tomar 15 casos posibles: que la línea esté justo
  //debajo de alguno de los fotodiodos o que este justo en el medio entre dos.
  //Para cada uno de esos casos vamos a asignar un valor de ángulo sacado de
  //un ensayo. Como el sensor actual no había sido diseñado para medir ángulos,
  //los resultados fueron bastante asimétricos y poco uniformes, con zonas
  //inestables y puntos ciegos. A futuro se debe mejorar el hardware del sensor.
  //Resultado del Ensayo:
  float betaEnsayo[]={-0.6981317,-0.53232542,-0.32288591,-0.20071286,-0.16580628,-0.06981317,0.052359878,0.161442956,0.253072742,0.327249235,0.327249235,0.327249235,0.410152374,0.567232007,0.772308194};
  //Obs: en éste ensayo no se distinguía entre las configuraciones 7-6, 6 y 6-5.

	uint8_t LED[8]; int aux, suma, sumaPonderada;float beta;
  
  LED[0]=bitRead(port1,bit1);
  LED[1]=bitRead(port2,bit2);
  LED[2]=bitRead(port3,bit3);
  LED[3]=bitRead(port4,bit4);
  //Acá hago algo distinto porque el pin es analógico
  aux=analogRead(A6);//Lectura del bit analógico
  if(aux>512){LED[4]=1;}
  else{LED[4]=0;}
  LED[5]=bitRead(port6,bit6);
  LED[6]=bitRead(port7,bit7);
  LED[7]=bitRead(port8,bit8);
  //Junto todos los bits en un byte:
  *byteSensor=0;
  for(int k=0;k<8;k++){
    bitWrite(*byteSensor,k,LED[k]);
  }
  suma=LED[0]+LED[1]+LED[2]+LED[3]+LED[4]+LED[5]+LED[6]+LED[7];
  sumaPonderada=2*(LED[0]+2*LED[1]+3*LED[2]+4*LED[3]+5*LED[4]+6*LED[5]+7*LED[6]+8*LED[7]);
  //Piso el valor de aux para hacer el switch case, total es int y ya no lo uso
  if(suma==0){beta=3;}//Si suma=0 es poque no detectó la línea, así que le doy el valor de error
  else{
    aux=sumaPonderada/suma;//Potencial fuente de error: la división de enteros. $.$ Ver si es problema
    beta=betaEnsayo[aux-2];
  }
  return beta;//Lo devuelvo como valor de salida
}

 void Controlados::configTimer2Contador(){
  int a=2000,b=32;
  Controlados::configTimer2Contador(a, b,0);
  }

void Controlados::configTimer2Contador(const int& Frecuencia, const int& Prescaler, bool interrupciones=0)
{
	/* Frecuencia es un valor que va desde un desaconsejable 500000 hasta 1997 
  *  El error en esta frecuencia esta dado por el redondeo
   * 
   * Esta función configura el timer2 para funcionar como contador.
	Para ello se debe usar el modo CTC, en el cual la frecuencia
	resultante según el datasheet es:
	                 fOCnx=fclkio/(2N(1+OCRnx))
  Ver :Clear Timer on Compare Match (CTC) Mode  (WGM2[2:0] = 2) (pag 195)
  
	Esta frecuencia es la que tendríamos si ponemos la salida OC2A
	en toogle, por lo tanto el tiempo entre interrupciones es la mitad
	del periodo indicado por esta ecuación. En limpio:
					   Tint=N(1+OCRnx)/fclkio
	Se selecciona, además, para trabajar con interrupción por timer2.
	La activación de las interrupciones globales se realiza por fuera
	de esta función.
	Como el timer2 se va a usar de forma compartida para determinar la
	frecuencia de muestreo y para hacer mediciones de tiempo, pongo
	el preescalador bajo para medir el menor tiempo posible, y el
	preescalador que necesitaría para la frecuencia de muestreo lo 
	hago por soft usando una variable auxiliar.
	*/
	TCCR2B=0x00; //Deshabilitamos el timer durante la configuración.
	/*Para elegir el modo de CTC hay que poner WGM2[2:0]=010. Por otro
	lado, para desactivar las salidas asociadas al timer2 hay que 
	poner COM2A[1:0]=COM2B[1:0]=00. Como el registro TCCR2A tiene los
	4 MSbits COM2A[1:0] y COM2B[1:0], seguidos de 2 bits no utilizados,
	terminando con WGM2[1:0], configuramos:
 */
	TCCR2A=0b00000010;
	/*Queremos obtener un periodo de muestreo que sea un múltiplo del
	valor que genera la interrupción por timer. Buscamos que ese valor
	sea de 10us y después ajustamos el valor final de tiempo de muestreo
	en el código principal cuando se genera la interrupción.
	Siendo fclkio=16.3MHz, tomando OCR2A=50 obtenemos Tint=0.1ms, 
	aproximadamente.
  */

	OCR2A=round(float(F_CPU)/(float(Prescaler)*float(Frecuencia)));

	/*Para habilitar las interrupciones por timer2 usamos el registro
	TIMSK2. El único bit que nos interesa es el bit 1, OCIEA que activa
	interrupciones por compare match con el registro OCR2A.
	Antes de activar la interrupción bajamos la bandera correspondiente
	en el registro TIFR2. Para bajarla hay que setearla (ver pag 212 datasheet).
  */ 
	TIFR2=1;
	bitWrite(TIMSK2,1,1); // TBit 1 – OCIEA: Timer/Counter2, Output Compare A Match Interrupt Enable
  /*
	En el registro TCCR2B los bits 7 y 6 fuerzan un compare match al
	escribírseles un 1;como no nos sirven los dejamos en 0. Los bits
	5 y 4 no se usan. El bit 3 es WGM22, por lo que debe valer 0. Los
	bits 2 a 0 son CS2[2:0] que seleccionan la fuente del clock. Para
	poner el clock source interno con preescalador 32 ponemos 011.
  */
  TCCR2B=0;
  switch(Prescaler){
    case 1:
    TCCR2B++;
    break;
    case 8:
    TCCR2B+=2;
    break;
    case 32:
    TCCR2B+=3;
    break;
    case 64:
    TCCR2B+=4;
    break;
    case 128:
    TCCR2B+=5;
    break;
    case 256:
    TCCR2B+=6;
    break;
    case 1024:
    TCCR2B+=7;
    break;
    default:
    TCCR2B+=3;
    break;   
    }

if (interrupciones){//Interrupciones por estado en pin para lectura de los encoders
   bitWrite(PCICR,PCIE1,1); // Pin Change Interrupt Control Register ; Bit 1 – PCIE1: Pin Change Interrupt Enable 1; PCINT[14:8]. Nosotros usamos A0=PCINT8 y A1=PCINT9
  bitWrite(PCMSK1,PCINT8,1); // PCINT8 corresponde a A0.
  bitWrite(PCMSK1,PCINT9,1); // PCINT9 corresponde a A1.
  bitWrite(PCIFR,PCIF1,1);// Limpio la bandera
  }
}
// ############################################### COMUNICACION
 Comunicacion::Comunicacion(int *_direccion){
  direccion_int=_direccion; tipo=0;
  }
void Comunicacion::modificar(int *valor)
{
*direccion_int=valor;
}
void Comunicacion::enviar()
{
  int a;float b;long c; unsigned char d;
  switch (tipo){
  case 0:
   a=*direccion_int;
 Serial.println(a,DEC);
 break;
 case 1:
  b=*direccion_flo;
 Serial.println(b,DEC);
 break;
 case 2:
  c=*direccion_long;
 Serial.println(c,DEC);
 break;
 case 3:
  d=*direccion_char;
 Serial.println(d,DEC);
 break;
 default:
 break;
}
}
void Comunicacion::iniciar()
{
int a;
  switch (tipo){
  case 0:
   a=direccion_int;
 Serial.println(a,DEC);
 break;
 case 1:
   a=direccion_flo;
 Serial.println(a,DEC);
 break;
 case 2:
   a=direccion_long;
 Serial.println(a,DEC);
 break;
 case 3:
   a=direccion_char;
 Serial.println(a,DEC);
 break;
 default:
 break;
  }
}

Comunicacion::Comunicacion(float *_direccion){
  direccion_flo=_direccion; tipo=1;
  }
Comunicacion::Comunicacion(long *_direccion){
  direccion_long=_direccion; tipo=2;
  }
  Comunicacion::Comunicacion(unsigned char *_direccion){
  direccion_char=_direccion; tipo=3;
  }

/*
void Comunicacion::modificar(float *valor)
{
*direccion_flo=valor;
}
void Comunicacion::modificar(long *valor)
{
*direccion_long=valor;
}
void Comunicacion::modificar(unsigned char *valor)
{
*direccion_char=valor;
}

*/



/*
void Comunicacion::enviar()
{

 float a=*direccion_flo;
 Serial.println(a,DEC);
}
void Comunicacion::iniciar()
{
  float a=direccion_flo;
 Serial.println(a,DEC);
}
*/


