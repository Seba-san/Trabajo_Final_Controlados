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
byteSensor=Dato1.datos;
wA=Dato2.datos;
wB=Dato3.datos;
%% Grafico de 3 medidas
indice=find(byteSensor==0);%Cuando pierde la l�nea el sensor queda en 0
K=length(wA);
if length(indice)>0
%     K=indice(1)-1;%Elimino los valores donde perdi� la l�nea
end
byteSensor=byteSensor(1:K);
wA=wA(1:K);
wB=wB(1:K);
for k=1:length(byteSensor)
    beta(k)=ConversionSensor(byteSensor(k),0);
end
Fs=200/4;
t=0:1/Fs:(length(beta)-1)/Fs;
figure(2);subplot(211);plot(t,beta,'.');grid on
subplot(212);plot(t,wA,t,wB);grid on
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='resp_escalon_sistema_total';
name2='';
name=strcat(direccion,name,name1,name2,'.mat');
% Parametros=table(500,20,50,c,200/4,'VariableNames',{'wref';'dW';'n0';'controlador';'Fs'})
%save(name,'t','beta','wA','wB','Parametros')