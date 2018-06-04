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
figure(1)
subplot(311)
yyaxis left
plot(t,rpm_A,t,wA);ylabel('rpm A estimadas')
title('Ensayo obtenido a partir de dW setpoint')
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
dW1=dW;beta1=beta;
% Otro ensayo

%load('../../Mediciones/180603192250resp_escalon_sistema_total.mat')
load('../../Mediciones/180603192511resp_escalon_sistema_total.mat')
[indice]=find(beta==3);t=t(1:(indice(1)-1));beta=beta(1:(indice(1)-1));
wA=wA(1:(indice(1)-1));wB=wB(1:(indice(1)-1));
dW=wB-wA;
sin_control_t=50;
figure(2)
subplot(311)
yyaxis left
plot(t,wA,t(50),wA(50),'o');ylabel('rpm A')
title('todo el ensayo')
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



% Separo  Datos sin control y con control

dWsc=dW(51:end);dWcc=dW(1:50);
betasc=beta(51:end);betacc=beta(1:50);
figure(3)
yyaxis left
plot(dWsc)
yyaxis right
plot(betasc)
title('Sin control')
figure(4)
yyaxis left
plot(dWcc)
yyaxis right
plot(betacc)
title('Con control')