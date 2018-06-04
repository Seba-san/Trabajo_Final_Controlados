%% 
% Se realizan las estimaciones de la planta con las mediciones del sistema
% con carga.
clc;clear all; close all;
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
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

l=1:1:200;
figure()
plot(l,RPM_A(2,:),'.',l,RPM_B(2,:),'.')

%%
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
% Cargo todas las mediciones
clc;clear all; close all;
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')


load('../../Mediciones/respuesta_escalon_motorB_10_40_180529195140.mat');
wAm(:,1)=wA;PWMAm(:,1)=PWMA;tiempom(:,1)=tiempo;
load('../../Mediciones/respuesta_escalon_motorB_40_80_180529195226.mat');
wAm(:,2)=wA;PWMAm(:,2)=PWMA;tiempom(:,2)=tiempo;
load('../../Mediciones/respuesta_escalon_motorB_10_80_180529195249.mat');
wAm(:,3)=wA;PWMAm(:,3)=PWMA;tiempom(:,3)=tiempo;
% Simulando planta
load('planta3.mat')
%P2D 3es el sistema
sys=idtf(P2D3); % Se puede tunear usando el PID Tuner en continuo y luego se dicretiza

for i=1:3
figure()
 lsim(sys,PWMAm(:,i)',tiempom(:,i)')
    hold on
    plot(tiempom(:,i),wAm(:,i),'.')
    hold off
%C = pidtune(sys_sr,'PI');
end
%%
% Para pasar el controlador de discreto a continuo


Kp=C0.Kp;Kd=C0.Kd;Ki=C0.Ki;
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

%% DE ACA PARA ABAJO ES BASURA!!! #### (no borrar)
sys_d=c2d(sys,1/200,'tustin')
sys_d2=feedback(sys_d*0.1,1);
  lsim(sys_d,PWMA(20:end),tiempo(20:end))
  hold on
  plot(tiempo(20:end),wA(20:end))
  hold off
step(sys_d)

step(P2D)


PIDF=0; % en 1 es si, en 0 es no
C0 = pid(1,1,1,PIDF,'Ts',1/200,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
%C = pidtune(sys,C0);
opt = pidtuneOptions('DesignFocus','reference-tracking','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys_d,C0,opt);
%[C,info] = pidtune(sys,C0);
T_pi = feedback(C*sys_d, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys)
title ('Sin controlar')

%%

%Sistema sin retardo
sys_sr=tf(7.228e04,[1 165.3 6827]);
sys_srd=c2d(sys_sr,1/200,'tustin');
PIDF=0; % en 1 es si, en 0 es no
C0 = pid(1,1,1,PIDF,'Ts',1/200,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
%C = pidtune(sys,C0);
opt = pidtuneOptions('DesignFocus','reference-tracking','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys_srd,C0,opt);
%[C,info] = pidtune(sys,C0);
T_pi = feedback(C*sys_srd, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys_srd)
title ('Sin controlar')



%%
step(sys_sr)
sys=feedback(sys_sr*0.5,1)
 lsim(sys_sr,PWMA3(20:end),tiempo3(20:end))

step(sys)



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
Kp=C0.Kp;Kd=C0.Kd;Ki=C0.Ki;
Fsnano=200;
% Ts2=0.015;
Ts2=1/Fsnano;
clear A B C D E F;
Tf=0;%No s� qu� poner en Tf porque ZN no me lo da.
control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts2,'tustin'); 
[A,B,C,D,E]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),'PI');
ctes=table(A,B,C,D,E);


ctesPID=['{' num2str(ctes.A) ',' num2str(ctes.B) ',' num2str(ctes.C) ',' num2str(ctes.D) ',' num2str(ctes.E) '}']
% Transferencia del controlador: H(z)=(A+Bz^-1+Cz^-2)/(1-Dz^-1-Ez^-2)
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
% Filtro
n=10;
Wn=0.01;
b = fir1(n,Wn,'low')
freqz(b,1,512)
outhi = filter(b,1,wA);

subplot(2,1,1)
plot(tiempo,wA,'.')
title('Original Signal')
%ys = ylim;

subplot(2,1,2)
plot(tiempo,outhi)
title('Highpass Filtered Signal')
xlabel('Time (s)')
%ylim(ys)
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

%PmR='K*exp(a*x)';
PmR='K*x';
figure(2)
[f1,gof] = fit(tiempo(1:end-1)',y(1:end-1)',PmR)
plot(f1,tiempo,y)
title('respuesta temporal')

% Modifico los datos para usarlos en sistem-identification
beta_interp = interp1(tiempo,y,tiempo2,'Spline');
figure(3)
plot(tiempo,y,'o',tiempo2,beta_interp,':.');
title('(Default) Linear Interpolation');
% Datos obtenidos por sistem_identification
Kp = 43.791 ;Tp1 = 1.9377 ;
sys=tf(Kp,[Tp1 -1]);
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

% Pruebo cosas locas

k=f1.K;a=f1.a;
sys4=tf(k,[1/a -1]);
step(sys4,tiempo2)


%%
clear all;close all
%load('../../Mediciones/ensayo_sabado.mat')
load('../../Mediciones/ensayo_sabado_filtrado.mat') % Controlador P con Kp=200
load('parametros.mat')
wref=500; % Acordate de esto! 
Kp=200;
dW=wA-wB;
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



