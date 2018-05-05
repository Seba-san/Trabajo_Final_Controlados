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
figure(1);plot(t,s);
%% Muestreo de la respuesta "contínua"
clear tiempo
for k=1:length(taux)
    %Busco el valor de tiempo contínuo más próximo
    [~,indice]=min(abs(taux(k)-t));
    tiempo(k)=t(indice);
    salida(k)=s(indice);
end
entrada=ones(1,length(salida));
figure(2);plot(t,s);hold on
stem(tiempo,salida);
%% Acomodo las muestras para que estén equiespaciadas
Fs=2e3;
Ts=1/Fs;
t=min(tiempo):1/Fs:max(tiempo);
w_interp = interp1(tiempo,salida,t); % Interpola lo datos de salida
% Interpola los datos de entrada, para evitar que los haga con una
% pendiente, los fabrico a mano.
inicia=find(entrada==max(entrada),1);

%Tania: El primer valor al que la entrada es máxima es cuando el escalón
%empieza. Acá la idea es buscar ese valor de tiempo en el nuevo vector para
%después generar el vector de la señal de entrada.

% Busca cuando empezo el pulso
inicia_seg=find(abs(tiempo(inicia)-t)==min(abs(tiempo(inicia)-t)));

pwm_interp=zeros(1,length(t));
pwm_interp(inicia_seg:length(t))=max(entrada);pwm_interp(1:(inicia_seg-1))=min(entrada);
figure(3);subplot(211);stem(tiempo,salida);hold on; plot(t,w_interp,'.');
title('Interpolacion Lineal de w-salida');legend('Muestras Originales','Interpolación');
subplot(212);stem(tiempo,entrada);hold on; plot(t,pwm_interp,'.');
title('Interpolacion Lineal de pwm-entrada');legend('Muestras Originales','Interpolación');
%% Estimación de la Planta
data = iddata(w_interp',pwm_interp',Ts);
np=3;%Le indico el nro de polos
nz=[2];%Le indico el nro de ceros
iodelay=[];%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts)
% Cambio el tiempo de muestreo del sistema:
Ts2=0.015;
sys_2=d2d(sys,Ts2);

%Comparo las respuestas a la entrada usada antes:
salidaEstimada=filter(sys.num,sys.den,pwm_interp);
figure(4);plot(t,w_interp,t,salidaEstimada);
title('Respuesta del Sistema Orignal y del Estimado (Ts inicial)');
legend('Respuesta del Sistema Original','Respuesta del Sistema Estimado');

%Ídem para el sistema con menor Fs:
figure(5);step(Planta);hold on;step(sys_2)
title('Respuesta del Sistema Orignal y del Estimado (Fs reducida)');
legend('Respuesta del Sistema Original','Respuesta del Sistema Estimado');
%% PID Ziegler-Nichols (Ogata) - Contínuo
[Kp,Ki,Kd]=ControlZN('P',entrada,salida,tiempo,0);
% Con los valores de las constantes armo el PID:
Tf=0;%No sé qué poner en Tf porque ZN no me lo da.
P = pid(Kp,Ki,Kd,Tf);%Como no especifiqué Ts me da un controlador contínuo.
sysControladoP= feedback(series(P,Planta),1);
[Kp,Ki,Kd]=ControlZN('PI',entrada,salida,tiempo,0);
PI = pid(Kp,Ki,Kd,Tf);
sysControladoPI= feedback(series(PI,Planta),1);
[Kp,Ki,Kd]=ControlZN('PID',entrada,salida,tiempo,0);
PID = pid(Kp,Ki,Kd,Tf);
sysControladoPID= feedback(series(PID,Planta),1);
figure(7);%subplot(211);step(Planta);title('Sistema sin controlar')
% subplot(212);
step(sysControladoP);hold on;step(sysControladoPI);
step(sysControladoPID);title('Sistema controlado');
legend('P','PI','PID');
%% PID Contínuo con PID tuner

%CONTINUARA!! TAN, TAN, TAN!!

%% Conversion a Discreto
