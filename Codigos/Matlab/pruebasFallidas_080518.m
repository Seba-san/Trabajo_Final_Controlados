%% Ensayo al escalón - Cargo los datos
clear all;close all;clc;
cd('C:\Users\Tania\Documents\ING\Carrera de Grado\Controlados\Trabajo Final con Seba\Git con Seba\Trabajo_Final_Controlados_git\Codigos\Matlab')
load('../../Mediciones/respuesta_escalon_180503213903.mat')
% load('../../Mediciones/respuesta_escalon_180503210331.mat')
tiempo=tiempo*1e-6;%Acomodo la unidad del tiempo.
%% Ajuste del nombre de las variables
N=176;%Corto la señal para sacar valores ruidosos
entrada=PWMA(1:N);
salida=wA(1:N);
tiempo=tiempo(1:N);
entrada=entrada-min(entrada);
salida=salida-min(salida);
[~,indice]=max(entrada);%Acomodo el timepo para que el escalón arranque en t=0
tiempo=tiempo-tiempo(indice);
figure(1);
plot(tiempo,entrada,tiempo,salida,'.');
legend('Señal de PWM','Señal de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
%%
k=max(entrada);%Para que sea la rpta el escalón tradicional debería dividir entrada y salida por este número.
% Eqn = 'a/(b*c)+a*exp(-b*x)*/(b^2-b*c)+a*exp(-c*x)*/(c^2-b*c)'
Eqn = 'a+b*exp(-c*x)+d*exp(-e*x)'
% Eqn = 'a/(b*c)'
startPoints = [10 10 20 10 20];  
x=tiempo(indice:length(tiempo));
y=salida(indice:length(tiempo))/k;
f1 = fit(x',y',Eqn,'Start', startPoints)
%%
a=f1.a;
b=f1.b;
c=f1.c;
d=f1.d;
e=f1.e;
f=a+b*exp(-c.*x)+d*exp(-e.*x);
figure();plot(x,y,x,f)
