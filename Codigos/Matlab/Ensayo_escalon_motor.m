%% Inicializacion de serial
clear all;clc;close all;
% cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%% Ensayo al escalon
N=300;%Cantidad de muestras a tomar
Fs=200;%Frec de env�o de datos
tiempo=0:1/Fs:(N-1)/Fs;%Genero a mano el vector de tiempos (asumo que el nano trabaja a frec cte sin problema)
wA=[];
%Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
PWMA=[];                   
PWM=10;
Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
pause(3);
Comunic_test(s)
Env_instruccion(s,'online');
flushinput(s);  %Vacia el buffer de entrada
for k=1:N %Ojo con el tope que pone el nano
    if k==80
        PWM=80;
        Env_instruccion(s,'PWM',[PWM PWM]);
    end
    w=str2double(fscanf(s));
    wA=[wA w];
    PWMA=[PWMA PWM];
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='respuesta_escalon_motorB_';
name2='10_80_';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'tiempo','PWMA','wA') 
figure(1);plot(tiempo,PWMA,tiempo,wA,'.');%ylim([0 1500])
legend('Señal de PWM','Señal de vel ang');
title('Respuesta del Motor A');
xlabel('tiempo (us)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $