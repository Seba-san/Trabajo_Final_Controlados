#include "Arduino.h"
#include "Controlados.h"

// Instrucciones traducidas
#define ins_PWM 250 // instrucciones traducidas.
#define ins_trama 253
#define ins_online 252
#define ins_stop 251 // baja de transmitir online
#define ins_test 249
#define ins_setpoint 248
#define ins_parametros 247
#define ins_u_control 246

// Pines para debuguear
#define SalidaTest 2
#define SalidaTest2 3
#define SalidaTest3 4
// Variables para la comunicacion
extern bool Motores_ON,tx_activada, online;
extern unsigned char trama_activa;
extern int PWMA,PWMB;
extern float u[3];
extern float set_point;
extern float Parametros[5];

//Variables del controlador:
extern float ent0;

// Variables para interrupcion por Overflow


extern int soft_prescaler,Bandera;
extern const int cota;
extern const float k0;
extern bool estado;


extern volatile unsigned long cantOVerflow;
extern volatile unsigned long TCNT2anterior;//Valor anterior del contador (para corregir la medición)
extern volatile unsigned long TCNT2actual;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.
extern volatile unsigned long cantOVerflow_actual;


// Variables de pruebas

extern volatile float freq;
// funciones

extern Controlados controlados1;
extern void toc(void);
void timer_interrupt(void);
extern void EnviarTX_online(float);
