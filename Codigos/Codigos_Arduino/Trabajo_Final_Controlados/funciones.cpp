#include "funciones.h"
#include "Arduino.h"  




void PID_offline (void){ //$9 PID

  /* La idea de esta funcion es que realice los calculos que no requieren que esten sincronizados.
   Luego, en la interrupcion por overflow, que e hagan solo los calculos necesarios y se aplique la señal de control.
   */
 
for(int k=0;k<2;k++)
  {
   error[k]=error[k+1];//Desplazamiento a la derecha de los datos del buffer
   u[k]=u[k+1];
  }  
error[2]=((float)(set_point)-freq+sal0);
u[2]=Parametros[0]*error[2]+Parametros[1]*error[1]+Parametros[2]*error[0]+Parametros[3]*u[1]+Parametros[4]*u[0];
if (u[2]>windup_top){u[2]=windup_top;}
if (u[2]<windup_bottom){u[2]=windup_bottom;}
//EnviarTX_online(error[2]);
//EnviarTX_online(error[1]);
//EnviarTX_online(error[0]);
//EnviarTX_online(u[2]);

}


 //  #####################  Modulos de comunicaciones 
 
// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, unsigned long *datos){ //  $8 ojo, datos es un puntero
  
 // [inicio][cantidad de datos][identificador][Datos][fin] Agregar un CRC y ACK???

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
  void tic()
  {
    ticc=micros();
  }

void toc()
  {
   tocc=micros()-ticc;
    }
