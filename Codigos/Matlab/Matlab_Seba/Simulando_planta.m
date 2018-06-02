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
C=-100;
T_pi = feedback(C*sys3, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys3)
title ('Sin controlar')

