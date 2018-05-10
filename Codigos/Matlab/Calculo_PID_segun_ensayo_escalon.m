%% Cálculo de PID
% Este script carga los datos del ensayo al escalón para uno de los motores
% y calcula a partir de estos un controlador PID. Se estima el controlador
% usando el método de Ziegler Nichols. Para estudiar el controlador
% diseñado se hace una estimación de la planta y se verifica el efecto del
% controlador sobre esta estimación de la planta.
%% Ensayo al escalón - Cargo los datos
clear all;close all;clc;
cd('C:\Users\Tania\Documents\ING\Carrera de Grado\Controlados\Trabajo Final con Seba\Git con Seba\Trabajo_Final_Controlados_git\Codigos\Matlab')
load('../../Mediciones/respuesta_escalon_180503213903.mat')
% load('../../Mediciones/respuesta_escalon_180503210331.mat')
tiempo=tiempo*1e-6;%Acomodo la unidad del tiempo.
%% Gráfico de la Respuesta al escalón
figure(1);
plot(tiempo,PWMA,tiempo,wA,'.');
legend('Señal de PWM','Señal de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
%% Ajuste del nombre de las variables
N=176;%Corto la señal para sacar valores ruidosos
entrada=PWMA(1:N);
salida=wA(1:N);
tiempo=tiempo(1:N);
entrada=entrada-min(entrada);
salida=salida-min(salida);
figure(1);
plot(tiempo,entrada,tiempo,salida,'.');
legend('Señal de PWM','Señal de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
%% PID Discreto con Ziegler - Nichols
Ts2=0.015;
tipo={'P';'PI';'PID'};
clear A B C D E F;
for k=1:3
    [Kp,Ki,Kd]=ControlZN(tipo{k},entrada,salida,tiempo,0,1);
    Tf=0;%No sé qué poner en Tf porque ZN no me lo da.
    control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts2,'tustin')
    [A(k,1),B(k,1),C(k,1),D(k,1),E(k,1)]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),tipo{k});
end
ctes=table(A,B,C,D,E,'RowNames',tipo)

%Para pasarle la información a Seba:
ctesP=['{' num2str(ctes.A(1)) ',' num2str(ctes.B(1)) ',' num2str(ctes.C(1)) ',' num2str(ctes.D(1)) ',' num2str(ctes.E(1)) '}'];
ctesPI=['{' num2str(ctes.A(2)) ',' num2str(ctes.B(2)) ',' num2str(ctes.C(2)) ',' num2str(ctes.D(2)) ',' num2str(ctes.E(2)) '}'];
ctesPID=['{' num2str(ctes.A(3)) ',' num2str(ctes.B(3)) ',' num2str(ctes.C(3)) ',' num2str(ctes.D(3)) ',' num2str(ctes.E(3)) '}'];
% Transferencia del controlador: H(z)=(A+Bz^-1+Cz^-2)/(1-Dz^-1-Ez^-2)
%Armo los controladores:
controlP=tf([ctes.A(1)],[1],Ts2);
controlPI=tf([ctes.A(2) ctes.B(2)],[1 -ctes.D(2)],Ts2);
controlPID=tf([ctes.A(3) ctes.B(3) ctes.C(3)],[1 -ctes.D(3) -ctes.E(3)],Ts2);
%% Estimación de la Planta (sólo para verificación)
% Acomodo las muestras para que estén equiespaciadas;
Fs=2e3;Ts=1/Fs;
t=min(tiempo):1/Fs:max(tiempo);
w_interp = interp1(tiempo,salida,t); % Interpola lo datos de salida
inicia=find(entrada==max(entrada),1);
%Tania: El primer valor al que la entrada es máxima es cuando el escalón
%empieza. Acá la idea es buscar ese valor de tiempo en el nuevo vector para
%después generar el vector de la señal de entrada.
% Busca cuando empezo el pulso:
inicia_seg=find(abs(tiempo(inicia)-t)==min(abs(tiempo(inicia)-t)));
pwm_interp=zeros(1,length(t));
pwm_interp(inicia_seg:length(t))=max(entrada);pwm_interp(1:(inicia_seg-1))=min(entrada);
% figure(2);subplot(211);stem(tiempo,salida);hold on; plot(t,w_interp,'.');
% title('Interpolacion Lineal de w-salida');legend('Muestras Originales','Interpolación');
% subplot(212);stem(tiempo,entrada);hold on; plot(t,pwm_interp,'.');
% title('Interpolacion Lineal de pwm-entrada');legend('Muestras Originales','Interpolación');

% Estimación de la Planta:
data = iddata(w_interp',pwm_interp',Ts);
np=3;%Le indico el nro de polos
nz=[];%Le indico el nro de ceros
iodelay=[];%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts);
syscont = tfest(data,np,nz,[]);
% Cambio el tiempo de muestreo del sistema:
sys2=d2d(sys,Ts2);
% Acomodo la señal de entrada a la nueva frec de muestreo:
t2=linspace(min(t),max(t),(max(t)-min(t))/Ts2);
[~,indice]=max(pwm_interp);%Busco el punto donde la señal cambia
%Busco el índice del valor de t2 más próximo al valor de tiempo donde se da
%el mínimo de pwm_interp:
[~,indice]=min(abs(t2-t(indice)));
%Armo a mano el vector de entrada para que sea un escalón perfecto con
%cambio en el tiempo encontrado antes:
pwm2(indice:length(t2))=max(pwm_interp)*ones(1,length(t2)-indice+1);
pwm2(1:indice-1)=min(pwm_interp)*ones(1,indice-1);

%Comparo las respuestas a la entrada usada antes:
salidaEstimada1=filter(sys.num,sys.den,pwm_interp);
salidaEstimada2=filter(sys2.num,sys2.den,pwm2);
figure();plot(t,w_interp,t,salidaEstimada1,t2,salidaEstimada2);
title('Respuesta del Sistema Orignal y del Estimado');
legend('Rpta del Sist Original','Rpta del Sist Estimado','Rpta del Sist Estimado con Fs reducida');

% figure();step(syscont);hold on;step(sys);step(sys2);
% title('Respuesta al Escalón');
% legend('Versión Contínua','Versión discreta 1','Versión discreta 2');
%% Diagrama de polos y ceros para los sistemas estimados y los controladores
% figure();subplot(121);iopzmap(syscont);subplot(122);iopzmap(sys);hold on;iopzmap(sys2);
figure();subplot(121);iopzmap(syscont);subplot(122);iopzmap(c2d(syscont,Ts2,'tustin'));
figure();subplot(121);iopzmap(controlPI);subplot(122);iopzmap(d2c(controlPI,'tustin'));
figure();subplot(121);iopzmap(controlPID);subplot(122);iopzmap(d2c(controlPID,'tustin'));
%% Desempeño del controlador para la planta estimada
%Armo los sistemas realimentados:
sysControladoP= feedback(series(d2c(controlP,'tustin'),syscont),1);
sysControladoPI= feedback(series(d2c(controlPI,'tustin'),syscont),1);
sysControladoPID= feedback(series(d2c(controlPID,'tustin'),syscont),1);
% %Calculo las salidas de los sistemas realimentados:
% salidacontroladaP=filter(cell2mat(sysControladoP.num),cell2mat(sysControladoP.den),pwm2);
% salidacontroladaPI=filter(cell2mat(sysControladoPI.num),cell2mat(sysControladoPI.den),pwm2);
% % salidacontroladaPID=filter(cell2mat(sysControladoPID.num),cell2mat(sysControladoPID.den),pwm2);
% figure(4);plot(t2,salidacontroladaP,t2,salidacontroladaPI,t2,salidacontroladaPID);
% title('Respuesta del Sistema Controlado');
% legend('Control P','Control PI','Control PID');

%Calculo las respuestas al escalón de los sistemas realimentados:
figure();step(sysControladoP);hold on;step(sysControladoPI);%step(sysControladoPID);
legend('Control P','Control PI','Control PID');