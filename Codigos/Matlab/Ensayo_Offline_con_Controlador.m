% Este script permite recuperar el vector de mediciones almacenado en el
% nano como resultado de un ensayo.
%% Inicio
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
clear all;close all;clc
InfoHard=instrhwinfo('serial');%Busco el puerto que tengo conectado
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Le pido las 3 medidas:
Comunic_test(s)
Env_instruccion(s,'devolver ensayo');
[Dato1]=DatoRx(s)
[Dato2]=DatoRx(s)
[Dato3]=DatoRx(s)
%% Grafico de 3 medidas
byteSensor=Dato1.datos;
wA=Dato2.datos;
wB=Dato3.datos;
for k=1:length(byteSensor)
    beta(k)=ConversionSensor(byteSensor(k),0);
end
Fs=200/4;
t=0:1/Fs:(length(byteSensor)-1)/Fs;
figure(2);subplot(211);plot(t,beta,'.');%hold on;plot(Datos1.datos,'.');hold off
subplot(212);plot(t,wA,t,wB);
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')