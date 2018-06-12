/******************************************************************************
Librería para el Trabajo Final de Controlados - Header
******************************************************************************/


#ifndef CONTROLADOS_h
#define CONTROLADOS_h

#include <Arduino.h>

class Controlados
{
  public:
    // Constructor
    Controlados();
    //El constructor es el encargado de setear la librería.	
	
	//Parecería que también hay un destructor, que sería, si mal no entendí,
	//~Controlados(), pero como la librería que usé de base no lo tiene, lo 
	//dejo así por ahora.

	//Configuración del timer1 para los motores:
	void configPinesMotores();
	void configTimerMotores();
  void actualizarDutyCycleMotores(int dutyCycleA,int dutyCycleB);  
	void modoAdelante();
	void modoAtras();
	void modoStop();
	
	void configPinesSensorLinea();
	float leerSensorDeLinea(unsigned char* byteSensor);
	
	//Configuración timer2 como contador:
	void configTimer2Contador(const int& Frecuencia, const int& Prescaler,bool interrupciones);
  void configTimer2Contador();

 // Funciones agregadas por comodidad:
 //unsigned long tic(void);
//unsigned long toc(unsigned long);
  
	
  private: //Funciones y variables internas
	int _timerMotores;//Para indicar cuál timer fue configurado para los motores.
	int _TOP;//Para indicarle a la fc actualizarDutyCycleMotores cuál es el
	         //máximo valor que puede tomar el OCRnx (ciclo de 100%), que
			 //depende de la cant de bits seleccionada en configTimerMotores

};


class Comunicacion
{

  private:
  int direccion;

  public:
  void Iniciar(int direccion);
  void modificar(int valor);
  void modificar(float valor);
  void modificar(long valor);
  void enviar();
  
  
  };

#endif



