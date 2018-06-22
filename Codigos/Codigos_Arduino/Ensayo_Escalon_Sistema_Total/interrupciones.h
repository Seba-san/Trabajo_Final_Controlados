#include "Arduino.h"
#include "Controlados.h"

// Instrucciones traducidas
#define ins_PWM 250 // instrucciones traducidas.
#define ins_trama 253
#define ins_online 252
#define ins_stop 251 // baja de transmitir online
#define ins_test 249
#define ins_setpoint 248
#define ins_resultado_ensayo 245 //Devuelve de forma secuencial los datos del ensayo al escalón almacenados en w[N]

//Variables Ensayo Escalón Offline
extern int enviar_datos;

// Variables para la comunicacion
extern bool Motores_ON,tx_activada, online,controlador_motores;
extern unsigned char trama_activa;
extern int PWMA,PWMB;
extern float uA[3],uB[3];
extern volatile float freqA,freqB;
extern float set_pointA,set_pointB;
extern float ParametrosA[5],ParametrosB[5];
extern unsigned char estadoEncoder;


// Variables para interrupcion por Overflow
extern int soft_prescaler,Bandera,cont_A,cont_B;
extern const int cota;
extern volatile unsigned long cantOVerflowA,cantOVerflowB;
extern volatile unsigned long TCNT2anteriorA,TCNT2anteriorB;//Valor anterior del contador (para corregir la medición)
extern volatile unsigned long TCNT2actualA,TCNT2actualB;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.
extern volatile unsigned long cantOVerflow_actualA,cantOVerflow_actualB;

// funciones
extern Controlados controlados1;
extern void toc(void);
void timer_interrupt(void);
extern void EnviarTX_online(float);
