#include "funciones.h"
#include "Arduino.h"  

void PID_offline_Motores (void){ //$9 PID
/* La idea de esta funcion es que realice los calculos que no requieren que esten sincronizados.
 Luego, en la interrupcion por overflow, que se hagan solo los calculos necesarios y se aplique la señal de control.
*/
  for(int k=0;k<2;k++)
    {
     errorA[k]=errorA[k+1];//Desplazamiento a la derecha de los datos del buffer
     uA[k]=uA[k+1];
     errorB[k]=errorB[k+1];
     uB[k]=uB[k+1];
    }  
  errorA[2]=((float)(set_pointA)-freqA);
  uA[2]=ParametrosA[0]*errorA[2]+ParametrosA[1]*errorA[1]+ParametrosA[2]*errorA[0]+ParametrosA[3]*uA[1]+ParametrosA[4]*uA[0];
  if (uA[2]>windup_top){uA[2]=windup_top;}
  if (uA[2]<windup_bottom){uA[2]=windup_bottom;}
  errorB[2]=((float)(set_pointB)-freqB);
  uB[2]=ParametrosB[0]*errorB[2]+ParametrosB[1]*errorB[1]+ParametrosB[2]*errorB[0]+ParametrosB[3]*uB[1]+ParametrosB[4]*uB[0];
  if (uB[2]>windup_top){uB[2]=windup_top;}
  if (uB[2]<windup_bottom){uB[2]=windup_bottom;}
}

void PID_total(void){//PID del sistema en su conjunto
  for(int k=0;k<2;k++)
    {
     errorBeta[k]=errorBeta[k+1];//Desplazamiento a la derecha de los datos del buffer
     dw[k]=dw[k+1];
    }
  //errorBeta[2]=((float)(0)-beta);//El ángulo deseado es siempre 0  
  //errorBeta[2]=((float)(0.052359878)-beta);//El ángulo deseado no es 0 porque el sensor nunca mide cero
  errorBeta[2]=((float)(-0.039269908)-beta);//El ángulo deseado no es 0 porque el sensor nunca mide cero
  dw[2]=Parametros[0]*errorBeta[2]+Parametros[1]*errorBeta[1]+Parametros[2]*errorBeta[0]+Parametros[3]*dw[1]+Parametros[4]*dw[0];
  if (dw[2]>windup_top_dw){dw[2]=windup_top_dw;} //Cambiar nombres de windup_top y _bottom $$$$$
  if (dw[2]<windup_bottom_dw){dw[2]=windup_bottom_dw;}
}

 //  #####################  Modulos de comunicaciones 
 
// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, float *datos){ //  $8 ojo, datos es un puntero
 if (online==false && tx_activada==true){ 
  Serial.println(0xFF,DEC); // inicio
  Serial.println(cantidad,DEC);// cantidad de datos
  Serial.println(identificador);// identificador, se puede sacar, pero para fines debuggeros puede ayudar
  float a;
  for (int i=0;i<cantidad;i++){
    a=*(datos+i); // como datos es un puntero, necesito guardar en "a" el apuntador.
    Serial.println(a,DEC);
  }
  Serial.println(0xFE,DEC); // fin
 }
}

void EnviarTX(int cantidad,const char identificador, unsigned char *datos){ //  $8 ojo, datos es un puntero
 if (online==false && tx_activada==true){ 
  Serial.println(0xFF,DEC); // inicio
  Serial.println(cantidad,DEC);// cantidad de datos
  Serial.println(identificador);// identificador, se puede sacar, pero para fines debuggeros puede ayudar
  unsigned char a;
  for (int i=0;i<cantidad;i++){
    a=*(datos+i); // como datos es un puntero, necesito guardar en "a" el apuntador.
    Serial.println(a,DEC);
  }
  Serial.println(0xFE,DEC); // fin
 }
}

void EnviarTX_online(float var){
  if (online==true && tx_activada==true){
  Serial.println(var,DEC);
  }
}


void EnviarTX_online(int var){ //  $8 funcion de envio de datos de corta duracion. No se envia en formato trama, solo verifica una bandera.
  if (online==true && tx_activada==true){
  Serial.println(var,DEC);
  }
}

void EnviarTX_online(long var){
  if (online==true && tx_activada==true){
  Serial.println(var,DEC);
  }
}

// ##################### Estas 2 funciones hacen los mismo que en matlab :-)
void tic(){
    ticc=micros();
  }
void toc(){
    tocc=micros()-ticc;
  }
