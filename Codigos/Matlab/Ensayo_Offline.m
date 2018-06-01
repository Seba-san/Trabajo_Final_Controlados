% Este script permite recuperar el vector de mediciones almacenado en el
% nano como resultado de un ensayo.
%% Inicio
s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
InfoHard=instrhwinfo('serial');%Busco el puerto que tengo conectado
%s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%% Pido que me devuelva el ensayo al escal�n
Comunic_test(s)
Env_instruccion(s,'devolver ensayo');
[Dato]=DatoRx(s)
dato=DatoRx_online(s)
%% Gr�fico
figure(1);plot(Dato.datos,'.')