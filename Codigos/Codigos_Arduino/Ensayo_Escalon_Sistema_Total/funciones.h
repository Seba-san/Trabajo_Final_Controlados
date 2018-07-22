#include "Arduino.h"        //Contiene todas las declaraciones de funciones y registros de arduino

// variables de tic y toc
extern unsigned long ticc,tocc;

//variables del PID
extern volatile float  freqA,freqB,beta;
extern float uA[3],uB[3],dw[3];// historia del error cometido y la historia de las salidas de control ejecutadas.
extern float errorA[3],errorB[3],errorBeta[3];
extern float set_pointA,set_pointB;// Set_point esta en RPM
extern float ParametrosA[5],ParametrosB[5],Parametros[5];
extern int soft_prescaler,cont_A,cont_B,PWMA,PWMB,controlador;
extern int windup_top,windup_bottom,windup_top_dw,windup_bottom_dw;
extern volatile unsigned char byteSensor;
// Variables comunicaciones
extern bool online, tx_activada;
extern byte vect_beta; // Variable de testeo


extern void tic(void);
extern void toc(void);
extern void PID_offline_Motores(void);
extern void PID_total(void);
extern void EnviarTX(int cantidad,const char identificador, float *datos);
extern void EnviarTX(int cantidad,const char identificador, unsigned char *datos);
extern void EnviarTX_online(float);
extern void EnviarTX_online(int);
extern void EnviarTX_online(long);
extern void EnviarTx_blue(void);
extern void Bateria(void);
