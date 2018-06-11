#include "Arduino.h"        //Contiene todas las declaraciones de funciones y registros de arduino

// variables de tic y toc
extern unsigned long ticc,tocc;

//variables del PID
extern volatile float  freqA,freqB,beta;
<<<<<<< HEAD
extern float uA[3],uB[3],dw[3],wref;;// historia del error cometido y la historia de las salidas de control ejecutadas.
=======
extern float uA[3],uB[3],dw[3];// historia del error cometido y la historia de las salidas de control ejecutadas.
>>>>>>> 4e42941948c5447524447255cc4a7370f5dad129
extern float errorA[3],errorB[3],errorBeta[3];
extern float set_pointA,set_pointB;// Set_point esta en RPM
extern float ParametrosA[5],ParametrosB[5],Parametros[5];
extern int soft_prescaler;
extern int windup_top,windup_bottom,windup_top_dw,windup_bottom_dw;

// Variables comunicaciones
extern bool online, tx_activada;


extern void tic(void);
extern void toc(void);
extern void PID_offline_Motores(void);
extern void PID_total(void);
extern void EnviarTX(int cantidad,const char identificador, float *datos);
extern void EnviarTX(int cantidad,const char identificador, unsigned char *datos);
extern void EnviarTX_online(float);
extern void EnviarTX_online(int);
extern void EnviarTX_online(long);
