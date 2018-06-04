%% 
% Se realizan las estimaciones de la planta con las mediciones del sistema
% con carga.
clc;clear all; close all;
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%%
% Cargando Datos.
Fs=200;
PWM_A=zeros(2,200);PWM_B=PWM_A;
n0=20;
load('../../Mediciones/180531222349resp_escalon_mA_con_carga_10_80.mat');
RPM_A(1,:)=Dato.datos; PWM_A(1,1:n0)=10;PWM_A(1,(n0+1):end)=80;
load('../../Mediciones/180531221835resp_escalon_mA_con_carga_40_80.mat');
RPM_A(2,:)=Dato.datos;PWM_A(2,1:n0)=40;PWM_A(2,(n0+1):end)=80;
load('../../Mediciones/180531222559resp_escalon_mB_con_carga_10_80.mat');
RPM_B(1,:)=Dato.datos;PWM_B(1,1:n0)=10;PWM_B(1,(n0+1):end)=80;
load('../../Mediciones/180531222925resp_escalon_mB_con_carga_40_80.mat');
RPM_B(2,:)=Dato.datos;PWM_B(2,1:n0)=40;PWM_B(2,(n0+1):end)=80;
load('../../Mediciones/180601203314ensayo_escalon_angulowref_600_dw_100_PI.mat')
%load('../../Mediciones/ensayo_sabado.mat')
load('../../Mediciones/ensayo_sabado_filtrado.mat')
% Por comodidad supongo que en la muestra 100 comienza el escalon
dw=zeros(1,length(beta));dw(101:end)=100;
%load('../../Mediciones/modelos.mat')
%save('../../Mediciones/modelos.mat','sisupA','sisupB')

%%
load('../../Mediciones/modelos.mat')
% Ajusto el PID en continuo
sys=sisupA;
C0 = pid(1,1,0);  
opt = pidtuneOptions('DesignFocus','disturbance-rejection','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys,C0,opt); %% Hay que ajustar ambos sistemas con los mismos requisitos
T_pi = feedback(C*sys, 1);
figure ();
%hold on
step(T_pi)
%hold off
title ('Controlado')
figure (2);
%hold on
step(sys)
%hold off
title ('Sin controlar')

%%
Kp=C.Kp;Kd=C.Kd;Ki=C.Ki;
tipo='PI';
Fsnano=200;
% Ts2=0.015;
Ts2=1/Fsnano;
clear A B C D E F;
Tf=0;%No s� qu� poner en Tf porque ZN no me lo da.
control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts2,'tustin'); 
[A,B,C,D,E]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),tipo);
ctes=table(A,B,C,D,E);
ctesPID=['{' num2str(ctes.A) ',' num2str(ctes.B) ',' num2str(ctes.C) ',' num2str(ctes.D) ',' num2str(ctes.E) '}']
% Transferencia del controlador: H(z)=(A+Bz^-1+Cz^-2)/(1-Dz^-1-Ez^-2)




%%
% ### Lo paso a la forma apropiada para programarlo
mm=C0;
Primero=tf(mm.Kp,1);
Segundo=tf([mm.Ts*mm.Ki 0],[1 -1],mm.Ts);
if (PIDF==1)
Tercero=tf(mm.kd*[1 -1],mm.Tf*[1 -1]+mm.Ts*[1 0],mm.Ts);
else
Tercero=tf(mm.kd*[1 -1],mm.Ts*[1 0],mm.Ts); 
end
Pid=Primero+Segundo+Tercero;pipi=Pid; % Supongo que esta en la forma PID paralelo
num=pipi.Numerator{1};
den=pipi.Denominator{1};
num=num/den(1);den=den/den(1); % hay que dividir por den(1) porque es el coeficiente de u(k).
disp('A B C D E')
CUCU=[num -den(2:size(den,2))];
sprintf('%f, %f, %f, %f, %f',CUCU)
%%
%Obtengo el modelo de los motores implementados

load('../../Mediciones/modelos.mat')
sys=sisupA;
C0 = pid(1,1,0);  
opt = pidtuneOptions('DesignFocus','disturbance-rejection','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys,C0,opt); %% Hay que ajustar ambos sistemas con los mismos requisitos
T_pi = feedback(C*sys, 1);
[b,a]=ss2tf(T_pi.A,T_pi.B,T_pi.C,T_pi.D);
Ca=C;sysA=sys;
C0 = pid(1,1,0);  
opt = pidtuneOptions('DesignFocus','disturbance-rejection','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys,C0,opt); %% Hay que ajustar ambos sistemas con los mismos requisitos
T_pi = feedback(C*sys, 1);
Cb=C;sysB=sys;
save('../../Mediciones/modelos_controlados.mat','sysA','sysB','Cb','Ca')
%% fft
X=wA(110:end)-mean(wA(110:end));
Fs = 200;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(X);             % Length of signal
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
%%
%Diseño de predictor de smith
% PI
ancho_de_banda=50;%hz
wc=2*pi*ancho_de_banda;
Cpi = pidtune(sys,pidstd(1,1),wc)

Tpi = feedback([sys*Cpi,1],1,1,1);  % closed-loop model [ysp;d]->y
Tpi.InputName = {'ysp' 'd'};

step(Tpi), grid on

%%
load('../../Mediciones/180601203314ensayo_escalon_angulowref_600_dw_100_PI.mat')
% Acondiciono datos
n0=100; % A partir de cuando empieza el escalon
Fs=200;
% Busco los puntos de cambio
cambio=diff(beta);
tiempo2=0:1/200:54/200;
[valor,indice]=find(cambio);
y=[beta(100) beta(indice(1:end-1)+1)];
tiempo=([100 indice(1:end-1)]-n0+1)*1/Fs;

figure(1)
plot(tiempo,y,tiempo2,beta(100:154)')
title('Puntos relevantes')
% Fiteo curva

PmR='K*exp(a*x)';
%PmR='K*x';
figure(2)
[f1,gof] = fit(tiempo(1:end)',y(1:end)',PmR)
plot(f1,tiempo,y)
title('respuesta temporal')

% Modifico los datos para usarlos en sistem-identification
beta_interp = interp1(tiempo,y,tiempo2,'Spline');
figure(3)
plot(tiempo,y,'o',tiempo2,beta_interp,':.');
title('(Default) Linear Interpolation');
% Datos obtenidos por fit
Kp = f1.K ;Tp1 = 1/f1.a ;
sys2=tf(Kp,[Tp1 -1]);
figure(4)
step(sys2,tiempo2)
% Modelo estimado mediante aproximacion polinomica discreta
%save('modelo_beta.mat','arx111')


%%
% Simulo los datos del ajuste de la curva exponencial
k=f1.K;a=f1.a;
y2=k*exp(a.*tiempo2);
in=ones(1,length(y2))*100;
plot(tiempo2,y2,'.')
% Coeficientes del sistema k(1-exp(-a*t)); usando la curva fiteada.
Kp=0.01;Tp1=0.10561;
sys3=tf(Kp,[Tp1 -1]);
step(sys3,tiempo2)




%%
clear all;close all
%load('../../Mediciones/ensayo_sabado.mat')
load('../../Mediciones/modelos_controlados.mat')
load('../../Mediciones/ensayo_sabado_filtrado.mat') % Controlador P con Kp=200
load('parametros.mat')
wref=500; % Acordate de esto! 
Kp=200;
%dW=wA-wB;

% Obtengo las RPM desde los sistemas controlados
sysA_c=feedback(sysA*Ca,1);
rpm_A=lsim(sysA_c,wA,t);
sysB_c=feedback(sysB*Cb,1);
rpm_B=lsim(sysB_c,wB,t);
dW=rpm_B-rpm_A; % A -B o B - A???!!? segun codigo arduino B-A OJO!!
figure(5)
subplot(311)
yyaxis left
plot(t,rpm_A,t,wA);ylabel('rpm A estimadas')
yyaxis right
plot(t,beta);ylabel('beta medido')
subplot(312)
yyaxis left
plot(t,rpm_B,t,wB);ylabel('rpm B estimadas')
yyaxis right
plot(t,beta);ylabel('beta medido')
subplot(313)
yyaxis left
plot(t,dW);ylabel('delta RPM estimadas')
yyaxis right
plot(t,beta);ylabel('beta medido')




figure(1)
yyaxis left
plot(t,dW)
ylabel('delta W en RPM')
yyaxis right
plot(t,beta)
ylabel('beta en radianes')
title('Datos medidos')


sys3=feedback(sys2*200,1);
figure(2)
tiempo2=0:1/200:54/200;
step(sys2,tiempo2)
title('Sin controlar')
figure(3)
step(sys3,tiempo2)
title('Controlado')


% Prueba de rendimiento
figure(4)
yyaxis right
di=lsim(sys3,dW/100,t);
plot(t,di)
ylabel('beta simulado')
yyaxis left
plot(t,beta)
ylabel('beta medido')
title('Datos medidos vs Datos simulados')






%%
% Esto es para los datos con el sistema estable (ver celda de arriba )

% Acondiciono datos
Fs=200/4;
% Busco los puntos de cambio
cambio=diff(beta); %plot(beta);hold on;plot(cambio);hold off
tiempo2=0:1/Fs:(length(beta)-1)*1/Fs;
[valor,indice]=find(cambio~=0); %plot(tiempo2,beta);hold on;plot(tiempo2(indice),beta(indice),'o');hold off
k=1;
for i=1:length(indice)-1
    
    if (indice(i)+1)==indice(i+1)
        indice_n(k)=indice(i);
        k=k+1;
    else
        indice_n(k)=(indice(i)-1);
        k=k+1;
        indice_n(k)=(indice(i)+1);
        k=k+1;
    end
end
indice_n(1)=[];
y=  beta(indice_n);
tiempo=(indice_n)*1/Fs;

figure(1)
plot(tiempo,y,tiempo2,beta)
title('Puntos relevantes')
% Modifico los datos para usarlos en sistem-identification
beta_interp = interp1(tiempo,y,tiempo2,'Spline');
figure(3)
plot(tiempo2,beta,tiempo2,beta_interp,':.');
title('(Default) Linear Interpolation');

%%
%lsim(sys_sr,PWMA3(20:end),tiempo3(20:end))
load('parametros.mat')
sys3=feedback(sys2*200,1);
figure(1)
tiempo2=0:1/200:54/200;
step(sys2,tiempo2)
title('Sin controlar')
figure(2)
step(sys3,tiempo2)
title('Controlado')


% Prueba de rendimiento
figure(2)
yyaxis right
di=lsim(sys3,dW/100,t);
plot(t,di)
ylabel('beta simulado')
yyaxis left
plot(t,beta)
ylabel('beta medido')

%%
clear all
%load('../../Mediciones/180603192250resp_escalon_sistema_total.mat')
load('../../Mediciones/180603192511resp_escalon_sistema_total.mat')
[indice]=find(beta==3);t=t(1:(indice(1)-1));beta=beta(1:(indice(1)-1));
wA=wA(1:(indice(1)-1));wB=wB(1:(indice(1)-1));
dW=wB-wA;
sin_control_t=50;
subplot(311)
yyaxis left
plot(t,wA,t(50),wA(50),'o');ylabel('rpm A')
yyaxis right
plot(t,beta,t(50),beta(50),'o');ylabel('beta medido')
subplot(312)
yyaxis left
plot(t,wB,t(50),wB(50),'o');ylabel('rpm B ')
yyaxis right
plot(t,beta,t(50),beta(50),'o');ylabel('beta medido')
subplot(313)
yyaxis left
plot(t,dW,t(50),dW(50),'o');ylabel('delta RPM ')
yyaxis right
plot(t,beta,t(50),beta(50),'o');ylabel('beta medido')

load('../../Mediciones/modelos_controlados.mat')

dW_est=lsim(sysb_dw,beta,t);
plot(t,dW_est,t,dW);legend('a','b')



% Datos sin control

dWsc=dW(51:end);dWcc=dW(1:50);
betasc=beta(51:end);betacc=beta(1:50);
figure(1)
yyaxis left
plot(dWsc)
yyaxis right
plot(betasc)
figure(2)
yyaxis left
plot(dWcc)
yyaxis right
plot(betacc)
%%
Kp=sysb_dw.Kp;Tp1=sysb_dw.Tp1;
sys=tf(Kp,[Tp1 1]); % Entra beta sale dW
sys_inv=inv(sys);% Entra dW sale beta






