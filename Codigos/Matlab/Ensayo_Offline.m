% Este script permite recuperar el vector de mediciones almacenado en el
% nano como resultado de un ensayo.
clear all;close all;clc
%% Inicio
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
InfoHard=instrhwinfo('serial');%Busco el puerto que tengo conectado
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Pido que me devuelva el ensayo al escalón de ambos motores
Comunic_test(s)
Env_instruccion(s,'devolver ensayo');
[MotorA]=DatoRx(s);[MotorB]=DatoRx(s);
wA=MotorA.datos;wB=MotorB.datos;
N=120;%cantidad de muestras tomadas en el ensayo al escalón
n0=5;%nro de muestra después de la cual arranca el escalón
w1=400;w2=600;
setpointA=[w1*ones(1,n0) w2*ones(1,N-n0)];setpointB=setpointA;
Fs=200;t=0:1/Fs:(N-1)/Fs;
ParametrosA={0.073817,-0.06814,0,1,0};
ParametrosB={0.077848,-0.072512,0,1,0};
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='_ensayo_escalon_ambos_motores';
name=strcat(direccion,name,name1,'.mat');
%save(name,'wA','wB','setpointA','setpointB','t','ParametrosA','ParametrosB') 
%% Gráfico del ensayo al escalón de ambos motores
figure(1);plot(t,wA,'.',t,wB,'.',t,setpointA,t,setpointB);
legend('wA','wB','setpointA','setpointB');
title('Ensayo al Escalón para Ambos Motores con Carga')
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')