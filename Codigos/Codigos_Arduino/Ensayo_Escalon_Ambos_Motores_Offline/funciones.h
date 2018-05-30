#include "Arduino.h"        //Contiene todas las declaraciones de funciones y registros de arduino

// variables de tic y toc
extern unsigned long ticc,tocc;

//variables del PID
extern volatile float  freqA,freqB;
extern float uA[3],uB[3];// historia del error cometido y la historia de las salidas de control ejecutadas.
extern float errorA[3],errorB[3];
extern float set_pointA,set_pointB;// Set_point esta en RPM
extern float ParametrosA[5],ParametrosB[5];
extern int soft_prescaler;
extern int windup_top,windup_bottom;

// Variables comunicaciones
extern bool online, tx_activada;


extern void tic(void);
extern void toc(void);
extern void PID_offline(void);
extern void EnviarTX(int cantidad,const char identificador, float *datos);//Nota 30/05/18: Cambie *datos de unsigned long a float. Esperemos que funcione
extern void EnviarTX_online(float);
extern void EnviarTX_online(int);
extern void EnviarTX_online(long);
