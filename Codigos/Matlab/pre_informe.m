%% Antes de empezar poner esto
addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%%
% Generacion de las imagenes para el Pre-informe
clc;clear all;
% Respuesta escalon motores CON carga
Fs=200;
i=1;
load('../../Mediciones/180622181548respuesta_escalon_motores_scontrolador20_60_');
RPM_A.sc(i,:)=wA; PWM_A.sc(i,:)=PWMA;RPM_B.sc(i,:)=wB; PWM_B.sc(i,:)=PWMB;tiempo_.sc(i,:)=tiempo;
i=i+1;
load('../../Mediciones/180622181441respuesta_escalon_motores_scontrolador40_80_');
RPM_A.sc(i,:)=wA; PWM_A.sc(i,:)=PWMA;RPM_B.sc(i,:)=wB; PWM_B.sc(i,:)=PWMB;tiempo_.sc(i,:)=tiempo;

i=1;
load('../../Mediciones/180622174710respuesta_escalon_motores_ccontrolador40_80_');
RPM_A.cc(i,:)=wA; PWM_A.cc(i,:)=PWMA;RPM_B.cc(i,:)=wB; PWM_B.cc(i,:)=PWMB;tiempo_.cc(i,:)=tiempo;
% FALTA ESTE ENSAYO
% i=i+1;
% load('../../Mediciones/180621184925respuesta_escalon_motores_ccontrolador20_60_');
% RPM_A.cc(i,:)=wA; PWM_A.cc(i,:)=PWMA;RPM_B.cc(i,:)=wB; PWM_B.cc(i,:)=PWMB;tiempo_.cc(i,:)=tiempo;
%%
% muestro datos
x=tiempo_.sc(1,:);
y1=PWM_A.sc(1,:);y1_=PWM_B.sc(1,:);
y2=RPM_A.sc(1,:);
y3=RPM_B.sc(1,:);
figure(1);subplot(211); title('Ensayo con carga')
plot(x,y1,'b.',x,y1_,'r.');legend('señal de control');grid on;xlabel('tiempo (s)');ylabel('% de PWM')
subplot(212);
plot(x,y2,'.b',x,y3,'.r');legend('motor A','motor B');xlabel('tiempo (s)');ylabel('F_{ang} (RPM)')
grid on



%%
% sistema
sys=sysB;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2),'OutputDelay',sys.Td);

% Controladores
C=Ca;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
z=control.num{1}(2)/control.num{1}(1);
p=0;
k=1/control.num{1}(1);
control_=zpk(z,p,k)
% Sistema completo
sys=sysB;
sys_=zpk([],[-1/sys.Tp1 -1/sys.Tp2],sys.Kp/(sys.Tp1*sys.Tp2),'OutputDelay',sys.Td);
C=Cb;
Kp=C.Kp;Ki=C.Ki;Kd=C.Kd;Tf=0;
control=tf(pid(Kp,Ki,Kd,Tf)); 
sys_cm=control*sys_*inv(1+control*sys_)
[a,t]=step(sys_cm);
[a,t]=lsim(sys_cm,PWM_A(1,:),x);%,PWM_B(1,2));
%%
%Muestra de imagenes
x=tiempo_.cc(1,:);%(1:1:length(PWM_A(1,:)))*1/Fs;
y1=PWM_A.cc(1,:);
y2=RPM_A.cc(1,:);
y3=RPM_B.cc(1,:);
%yyaxis left
%plot(x,y1,'.');ylabel('RPM deseado');%ylim([-10 110]);
%yyaxis right
plot(x,y1,x,y2,'r.',x,y3,'b.');ylabel('RPM');legend('Señal de control','Motor A','Motor B','Location','southeast')
xlabel('tiempo (s)')
grid on;grid minor
title('Respuesta al escalón: Motores con carga  con controlador')
%% 
% Respuesta escalon sistema controlado
x=tiempo_.cc(1,:);%(1:1:length(PWM_A(1,:)))*1/Fs;
y1=RPM_A.sc(2,:);
y2=RPM_A.cc(1,:);%,RPM_A(1,:);
y3=PWM_A.sc(2,:);
yyaxis left
plot(x,y1,'-.',x,y2,'.');%ylim([-10 110]);
ylabel('RPM');
yyaxis right
plot(x,y3,'.');ylabel('Señal de control');legend('Motor B sin control','Motor B controlado','Señal de control','Location','southeast')
xlabel('s')
grid on
grid minor
title('Respuesta al escalón simulada Motor B controlado')
%%
% Prueba de linealidad de motores
load('../../Mediciones/180423122752_respuesta_motorB.mat');

x=PWMA;
y1=wA;

[f1,g,o]=fit(x(130:end)',y1(130:end)','poly1');
plot(f1,x,y1);ylabel('RPM');xlabel('% de duty')
legend('motor B','Recta ajustada')
grid on
grid minor
title('Barrido de PWM; Prueba de linealidad')


