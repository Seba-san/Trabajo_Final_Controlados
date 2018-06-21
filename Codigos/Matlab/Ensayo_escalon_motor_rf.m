%% Inicio
s=InicializacionSerial_rf('/dev/ttyUSB1',115200);%Velocidad: 115200 baudios
%addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
%cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%% Ensayo al escalon
pause(5)
PWM=10;
Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
N=200; %Cantidad de muestras
%RPMA=zeros(1,N);
%RPMB=zeros(1,N);
Fs=200;%Frec de env�o de datos
tiempo=0:1/Fs:(N-1)/Fs;%Genero a mano el vector de tiempos (asumo que el nano trabaja a frec cte sin problema)
wA=[];
wB=[];
%Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
PWMA=[];   
PWMB=[]; 
PWM=400;
%Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
Env_instruccion(s,'setpoint',[PWM PWM])
pause(1);
Comunic_test_rf(s)
Env_instruccion(s,'online');
flushinput(s);  %Vacia el buffer de entrada
for i=1:N %Ojo con el tope que pone el nano
    if i==50
        PWM=800;
        Env_instruccion(s,'setpoint',[PWM PWM]);
    end
    datos=DatoRx_rf(s);
    %flushinput(s);
    %beta=ConversionSensor(datos(1),0) ;
    w=datos(2); wA=[wA w]; 
    w=datos(3); wB=[wB w];    
    PWMA=[PWMA PWM];PWMB=[PWMB PWM];
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='respuesta_escalon_motores_ccontrolador';
name2='40_80_';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'tiempo','PWMA','wA','PWMB','wB') 
figure(1)
yyaxis left
plot(tiempo,wB,'r.',tiempo,wA,'b.');ylim([100 1200])
ylabel('Vel Ang (rpm)'); legend('Motor B','Motor A','Location','northwest')
yyaxis right
plot(tiempo,PWMB,'.');ylim([0 1000])
%legend('Señal de PWM');
title('Respuesta escalón ambos Motores');
xlabel('tiempo (s)'); %Revisar la unidad!! $
grid on;