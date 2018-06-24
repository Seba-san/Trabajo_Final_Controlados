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
parametros.beta=beta;
parametros.delay=0;
parametros.Ts=1/50;
parametros.k=0.034;
parametros.sys=sys_cinematico;
%parametros.sys=feedback(sys_cinematico*Kp,1);%sys_cinematico;
beta_estimado=est_beta(t,dW,beta(1),parametros,'discreto');
%beta_estimado=est_beta(t,dW,beta(1),parametros,'continuo');
figure(2)
yyaxis left
plot(t,beta,'.');ylabel('beta medido')
yyaxis right
plot(t,beta_estimado,'.');ylabel('beta estimado')
%figure(2)
% plot(t,beta,'.',t,beta_estimado,'.');legend('beta','beta_{estimado}')
% figure(3)
% plot(t,beta-beta_estimado)

%%
clear all
% Otro ensayo
%load('../../Mediciones/180603192250resp_escalon_sistema_total.mat')
%load('../../Mediciones/180603192511resp_escalon_sistema_total.mat')
%load('../../Mediciones/180604194417ensayo_a_vel_cte_con_PI.mat') % Solo
%control
%load('../../Mediciones/180604195315resp_escalon_sistema_total.mat') % con
%bolantazo
%load('../../Mediciones/180604195905resp_escalon_sistema_total.mat') % Con bolantazo
%load('../../Mediciones/sin_clasificar/180606144919.mat') % Prueba de pista


%load('../../Mediciones/Respuesta_Escalon_060618/180606185509resp_escalon_sistema_total.mat')
%load('../../Mediciones/Respuesta_Escalon_060618/180606185628resp_escalon_sistema_total.mat')
%load('../../Mediciones/Respuesta_Escalon_060618/180606190241resp_escalon_sistema_total.mat')
load('../../Mediciones/Respuesta_Escalon_060618/180606190409resp_escalon_sistema_total.mat')

try
[indice]=find(beta==3);t=t(1:(indice(1)-1));beta=beta(1:(indice(1)-1));
wA=wA(1:(indice(1)-1));wB=wB(1:(indice(1)-1));
end
dW=wB-wA;
try
t0=Parametros.n0;Ts=1/Parametros.Fs;
catch
    t0=50;Ts=1/50;
end
figure(2)
subplot(311)
yyaxis left
plot(t,wA,t(t0),wA(t0),'o');ylabel('rpm A')
title('Ensayo respuesta escalón sistema total');
yyaxis right
plot(t,beta,t(t0),beta(t0),'o');ylabel('beta medido');grid on; grid minor
subplot(312)
yyaxis left
plot(t,wB,t(t0),wB(t0),'o');ylabel('rpm B ')
yyaxis right
plot(t,beta,t(t0),beta(t0),'o');ylabel('beta medido');grid on; grid minor
subplot(313)
yyaxis left
plot(t,dW,t(t0),dW(t0),'o');ylabel('delta RPM ')
yyaxis right
plot(t,beta,t(t0),beta(t0),'o');ylabel('beta medido');grid on; grid minor
xlabel('Tiempo(segundos)')


% Separo  Datos sin control y con control

dWsc=dW((t0+1):end);dWcc=dW(1:t0);tcc=t(1:t0);tsc=t((t0+1):end);
betasc=beta((t0+1):end);betacc=beta(1:t0);
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

% figure(5)
% 
% beta_simulado=lsim(sys_cinematico,dW,t);
% yyaxis left
% plot(t,beta_simulado); ylabel('beta simulado')
% yyaxis right
% plot(t,beta); ylabel('beta medido')
% title('beta medido vs beta simulado')
%%
ls ../../Mediciones
%%
% sys_t=sysA_c;
% A=sys_t.A;B=sys_t.B;C=sys_t.C;D=sys_t.D;E=sys_t.E;
% [b,a]=ss2tf(A,B,C,D,E,1);sis=tf(a,b)

sys=sysA;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2),'OutputDelay',sys.Td);
C=Ca;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
% Sin delay
sys_cm=control*sys_*inv(1+control*sys_);
% Con delay
%sys_cmd=control*sysA*inv(1+control*sysA)

sys_t=2*sys_cm*sys_cinematico2


%%
% Reajuste de los controladores de los motores
load('../../Mediciones/modelos_controlados2.mat')
sys=sysA;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2),'OutputDelay',sys.Td);
C=Cb;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
sys_cm=control*sys_*inv(1+control*sys_);
%figure(1)
step(sys_cm)
hold on
sys=sysA;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2),'OutputDelay',sys.Td);
C=Ca; % 0.3184 0.75 robustes
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
sys_cm=control*sys_*inv(1+control*sys_);
%figure(2)
step(sys_cm)
hold off

%save('../../Mediciones/modelos_controlados2.mat','sysA','sysB','Cb','Ca','sys_cinematico2','sys_cinematico','Cct')

%%
C=Ca;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
z=control.num{1}(2)/control.num{1}(1);
p=0;
k=1/control.num{1}(1);
control_=zpk(z,p,k)






