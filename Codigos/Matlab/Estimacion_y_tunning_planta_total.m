% Este script estima el sistema total del robot a partir de los resultados
% del ensayo al escalón. Para eso usa el modelo obtenido en "Ideas para el 
% modelado del robot", cuya ecuación es: d(beta)/dt=c deltaW, donde c es un
% parámetro constante del sistema.
% Update: No, eso no funciona. Después veo qué onda
clear all;close all;clc
%% Cargo el ensayo
load('../../Mediciones/180601203314ensayo_escalon_angulowref_600_dw_100_PI.mat ');
%% Armo el vector de tiempo y el vector de entrada
% Para ello uso la información de qué parámetros tenía el ensayo al momento
% % de realizarse: Fs=200Hz, 200 muestras totales: 100 a dw=0, 100 a
% dw=100rpm, wref=400 rpm.
Fs=200;
N=200;
n0=100;
t=0:1/Fs:(N-1)/Fs;
dw=[zeros(1,n0) 100*ones(1,N-n0)];

%Ajuste trucho: no sé qué hacer con el offset que tengo inicialmente por no
%tener nunca una medición de beta==0, así que lo voy a forzar a mano a que
%sea cero cuando vale 0.05 :/
beta(1:100)=0;
%Para este ensayo la línea se perdió en la muestra 154, así que de ahí en
%adelante las descarto:
beta=beta(1:153);t=t(1:153);dw=dw(1:153);

figure();plot(t,beta,'.');title('Resultado del Ensayo al Escalón para el Sistema Total')
% legend('Entrada','Salida');
xlabel('t (seg)');ylabel('angulo (rad)');grid on
%% Discretizo el PID
Fsnano=200;
% Ts2=0.015;
Ts2=1/Fsnano;
tipo={'P';'PI';'PID'};
clear A B C D E F;
figura=[1,0,0];
for k=1:3
    %[Kp,Ki,Kd]=ControlZN(tipo{k},'CC2',entrada,salida,tiempo,0,figura(k));
    Kp=CPID.Kp;Ki=CPID.Ki;Kd=CPID.Kd;
    Tf=0;%No sï¿½ quï¿½ poner en Tf porque ZN no me lo da.
    control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts2,'tustin');
    [A(k,1),B(k,1),C(k,1),D(k,1),E(k,1)]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),tipo{k});
end
ctes=table(A,B,C,D,E,'RowNames',tipo)

%Para pasarle la informaciï¿½n a Seba:
ctesP=['{' num2str(ctes.A(1)) ',' num2str(ctes.B(1)) ',' num2str(ctes.C(1)) ',' num2str(ctes.D(1)) ',' num2str(ctes.E(1)) '}'];
ctesPI=['{' num2str(ctes.A(2)) ',' num2str(ctes.B(2)) ',' num2str(ctes.C(2)) ',' num2str(ctes.D(2)) ',' num2str(ctes.E(2)) '}'];
ctesPID=['{' num2str(ctes.A(3)) ',' num2str(ctes.B(3)) ',' num2str(ctes.C(3)) ',' num2str(ctes.D(3)) ',' num2str(ctes.E(3)) '}'];