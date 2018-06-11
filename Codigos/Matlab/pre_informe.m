% Generacion de las imagenes para el Pre-informe


% Respuesta escalon motores CON carga
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
x=(1:1:length(PWM_A(1,:)))*1/Fs;
y1=PWM_A(1,:);
y2=RPM_A(1,:);
y3=RPM_B(1,:);
yyaxis left
plot(x,y1,'.');ylim([-10 110]);ylabel('Entrada en % de duty')
yyaxis right
plot(x,y2,'r.',x,y3,'b.');ylabel('Salida en RPM');legend('Motor A','Motor B','Location','southeast')
xlabel('s')
grid on;grid minor
title('Respuesta al escalón Motores con carga')
%% 
% Respuesta escalon sistema controlado
x=t;%(1:1:length(PWM_A(1,:)))*1/Fs;
y1=PWM_A(1,:);
y2=a;%,RPM_A(1,:);
y3=RPM_A(1,:);
yyaxis left
plot(x,y1,'-',x,y2,'.');%ylim([-10 110]);
ylabel('RPM');
yyaxis right
plot(x,y3,'.');ylabel('Salida en RPM');legend('Señal de control','Motor B controlado','Motor B sin control','Location','southeast')
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


