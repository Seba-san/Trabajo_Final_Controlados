#include "Arduino.h"        //Contiene todas las declaraciones de funciones y registros de arduino

// variables de tic y toc
extern unsigned long ticc,tocc;

//variables del PID
extern volatile float  freq;
extern float u[3]; // historia del error cometido y la historia de las salidas de control ejecutadas.
extern float error[3];
extern float set_point; // Set_point esta en RPM
extern float Parametros[5];//{1.261400, -2.522133, 1.260733, 1.999500, -0.999500};
extern float sal0;
extern int soft_prescaler;
extern int windup_top,windup_bottom;
// Variables comunicaciones
extern bool online, tx_activada;


extern void tic(void);
extern void toc(void);
extern void PID_offline(void);
extern void EnviarTX(int cantidad,const char identificador, unsigned long *datos);
extern void EnviarTX_online(float);
extern void EnviarTX_online(int);
extern void EnviarTX_online(long);
