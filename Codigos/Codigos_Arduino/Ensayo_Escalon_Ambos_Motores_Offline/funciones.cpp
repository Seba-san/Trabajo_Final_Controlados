#include "funciones.h"
#include "Arduino.h"  

void PID_offline (void){ //$9 PID
/* La idea de esta funcion es que realice los calculos que no requieren que esten sincronizados.
 Luego, en la interrupcion por overflow, que e hagan solo los calculos necesarios y se aplique la señal de control.
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


 //  #####################  Modulos de comunicaciones 
 
// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, float *datos){ //  $8 ojo, datos es un puntero   //Nota 30/05/18: Cambie *datos de unsigned long a float. Esperemos que funcione
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
