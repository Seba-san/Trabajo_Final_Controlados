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
%%
% Otro ensayo

%load('../../Mediciones/180603192250resp_escalon_sistema_total.mat')
load('../../Mediciones/180603192511resp_escalon_sistema_total.mat')
%load('../../Mediciones/180604194417ensayo_a_vel_cte_con_PI.mat') % Solo
%control
%load('../../Mediciones/180604195315resp_escalon_sistema_total.mat') % con
%bolantazo
%load('../../Mediciones/180604195905resp_escalon_sistema_total.mat') % Con bolantazo
try
[indice]=find(beta==3);t=t(1:(indice(1)-1));beta=beta(1:(indice(1)-1));
wA=wA(1:(indice(1)-1));wB=wB(1:(indice(1)-1));
end
dW=wB-wA;
tsc=50;
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

dWsc=dW(51:end);dWcc=dW(1:50);tcc=t(1:50);tsc=t(51:end);
betasc=beta(51:end);betacc=beta(1:50);
figure(3)
yyaxis left
plot(tsc,dWsc) ; ylabel('dW [RPM]')
yyaxis right
plot(tsc,betasc); ylabel('beta')
title('Sin control')
figure(4)
yyaxis left
plot(tcc,dWcc); ylabel('dW [RPM]')
yyaxis right
plot(tcc,betacc); ylabel('beta')
title('Con control')



%%
ls ../../Mediciones
%%
% sys_t=sysA_c;
% A=sys_t.A;B=sys_t.B;C=sys_t.C;D=sys_t.D;E=sys_t.E;
% [b,a]=ss2tf(A,B,C,D,E,1);sis=tf(a,b)

sys=sysA;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2));%,'OutputDelay',sys.Td);
C=Ca;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
% Sin delay
sys_cm=control*sys_*inv(1+control*sys_);
% Con delay
%sys_cmd=control*sysA*inv(1+control*sysA)

sys_t=sys_cm*sys_cinematico








