% Este script permite recuperar el vector de mediciones almacenado en el
% nano como resultado de un ensayo.
%% Inicio
% s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
InfoHard=instrhwinfo('serial');%Busco el puerto que tengo conectado
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Pido que me devuelva el ensayo al escalón
Comunic_test(s)
Env_instruccion(s,'devolver ensayo');
[Dato]=DatoRx(s)
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='resp_escalon_mB_con_carga_';
name2='40_80';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'Dato') 
%% Gráfico
figure(1);plot(Dato.datos,'.')
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')