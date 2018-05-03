%% Prueba con planta ficticia
% Este código prueba las funciones a usar en la estimación del motor con
% una planta ficticia para depurar errores.
clear all;close all;clc
%% Generación de la planta ficticia
num=[100e3];
den=poly([-58 -55]);
Planta=tf(num,den);
[s,t]=step(Planta);
Info=stepinfo(s,t)
%% Ensayo al escalón (contínuo)
cd('C:\Users\Tania\Documents\ING\Carrera de Grado\Controlados\Trabajo Final con Seba\Git con Seba\Trabajo_Final_Controlados_git\Codigos\Matlab')
load('../../Mediciones/180423182356_resp_escalon.mat')
taux=tiempo*1e-6;%Acomodo unidad
taux=taux(16:length(taux))-taux(16);%Acomodo el cero

%Armo el vector de tiempos "continuo" en fucnión del del ensayo real
t=linspace(0,max(taux),1000*length(taux));
s=step(Planta,t)';
figure(2);plot(t,s);
%% Muestreo de la respuesta "contínua"
clear tiempo
for k=1:length(taux)
    %Busco el valor de tiempo contínuo más próximo
    [~,indice]=min(abs(taux(k)-t));
    tiempo(k)=t(indice);
    salida(k)=s(indice);
end
figure(2);plot(t,s);hold on
stem(tiempo,salida);
%% Acomodo las muestras para que estén equiespaciadas
entrada=ones(1,length(salida));
Fs=2e3;
Ts=1/Fs;
t=min(tiempo):1/Fs:max(tiempo);
w_interp = interp1(tiempo,salida,t); % Interpola lo datos de salida
% Interpola los datos de entrada, para evitar que los haga con una
% pendiente, los fabrico a mano.
inicia=find(entrada==max(entrada),1);
%Tania: El primer valor al que la entrada es máxima es cuando el escalón
%empieza
% Busca cuando empezo el pulso
inicia_seg=find(abs(tiempo(inicia)-t)==min(abs(tiempo(inicia)-t)));

pwm_interp=zeros(1,length(t));
pwm_interp(inicia_seg:length(t))=max(entrada);pwm_interp(1:(inicia_seg-1))=min(entrada);
%pwm_interp=interp1(tiempoSeg,entrada,t);
figure
plot(tiempoSeg,salida,'o',t,w_interp,':.');
title('Interpolacion Lineal de w_salida');
figure
plot(tiempoSeg,entrada,'o',t,pwm_interp,':.');
title('Interpolacion Lineal de pwm_entrada');