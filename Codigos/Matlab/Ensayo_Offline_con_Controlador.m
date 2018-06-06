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
indice=find(byteSensor==0);%Cuando pierde la línea el sensor queda en 0
K=length(wA);
if length(indice)>0
%     K=indice(1)-1;%Elimino los valores donde perdió la línea
end
byteSensor=byteSensor(1:K);
wA=wA(1:K);
wB=wB(1:K);
for k=1:length(byteSensor)
    beta(k)=ConversionSensor(byteSensor(k),0);
end
Fs=200;
t=0:1/Fs:(length(beta)-1)/Fs;
% figure(2);subplot(211);plot(t,beta,'.');grid on
% subplot(212);plot(t,wA,t,wB);grid on;legend('wA','wB')
figure(1); yyaxis right;plot(t,beta,'.');grid on
yyaxis left;plot(t,wB,t,wA);grid on;legend('wB-wA deseado','wB-wA medido')
%% Fin
fclose(instrfindall);%cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/Respuesta_Escalon_060618/';
name1='resp_escalon_sistema_total';
name2='';
name=strcat(direccion,name,name1,name2,'.mat');
Parametros=table(350,10,300,100,30,[200,0,0,0,0],200,...
    'VariableNames',{'wref';'n0';'D';'N';'dW';'controlador';'Fs'})
%save(name,'t','byteSensor','beta','wA','wB','Parametros')