% Este script permite recuperar el vector de mediciones almacenado en el
% nano como resultado de un ensayo.
%% Inicio
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
InfoHard=instrhwinfo('serial');%Busco el puerto que tengo conectado
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Pido que me devuelva el ensayo al escal�n
Comunic_test(s)
Env_instruccion(s,'devolver ensayo');
[Dato]=DatoRx(s)
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='resp_escalon_mB_con_carga_';
name2='400_800_PI';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'Dato')
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
Fs=200;
t=0:1/Fs:(length(byteSensor)-1)/Fs;
figure(2);subplot(211);plot(t,beta,'.');%hold on;plot(Datos1.datos,'.');hold off
subplot(212);plot(t,wA,t,wB);
%% Gr�fico
figure(1);plot(Dato.datos,'.');%hold on;plot(Datos1.datos,'.');hold off
ang=1;
if(ang==1)
    byteSensor=Dato.datos;
    for k=1:length(byteSensor)
        beta(k)=ConversionSensor(byteSensor(k),3);
    end
    figure(2);plot(beta,'.');%hold on;plot(Datos1.datos,'.');hold off
end
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='ensayo_escalon_angulo';
name2='wref_600_dw_100_PI';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'beta') 
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')