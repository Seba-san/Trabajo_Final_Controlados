%% Inicio
% s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
%Env_instruccion(s,'stop')
Comunic_test(s)
Env_instruccion(s,'PWM',[0 0]); 
setpointA=350;setpointB=setpointA;
Env_instruccion(s,'setpoint',[setpointA,setpointB]); 
N=400;
beta=zeros(1,N);wA=zeros(1,N);wB=zeros(1,N);
i=1;
RPMn=zeros(1,N);%RPMn(1)=RPM;
a=1;veces=1;
pause(1)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
    close(1)
end
figure(1)
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
flushinput(s);
veces_=0;
while (veces_<veces)
   %beta(i)=ConversionSensor(str2double(fscanf(s)),0);
   
   beta(i)=str2double(fscanf(s));
    wA(i)=str2double(fscanf(s));
    wB(i)=str2double(fscanf(s));
   flushinput(s); 
    
    m1=1:1:i;m2=i+1:1:N;
    ax1=subplot(311);
    cla(ax1);
    plot(m1,beta(1:i),'.','color',[~a 0 a]); hold on;plot(m2,beta(i+1:N),'.','color',[a 0 ~a]); hold off;ylim([-1 3])
    ylabel('beta')
    %     ylim([limite_inf limite_sup]);
    ax2=subplot(312);
    cla(ax2);
    plot(m1,wA(1:i),'.','color',[~a 0 a]); hold on;plot(m2,wA(i+1:N),'.','color',[a 0 ~a]); hold off;ylim([0 1000])
    ax3=subplot(313);
    cla(ax3);
    plot(m1,wB(1:i),'.','color',[~a 0 a]); hold on;plot(m2,wB(i+1:N),'.','color',[a 0 ~a]); hold off;ylim([0 1000])
%     plot(m1,wA(1:i),'.b'); hold on;plot(m1,wB(1:i),'.k');plot(m2,wA(i+1:N),'.','color',[a 0 ~a]);
%    ylabel('wA - wB');plot(m2,wB(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([0 900]);
%    legend('wA','wB')
    pause(0.0005)
    i=i+1;
    if (i>(N-1) ) 
        i=2;a=~a;
        veces_=veces_+1;
    end    
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',[0,0]); 
% pause(1)
% close all
%Env_instruccion(s,'PWM',[0 0]); 

%% Espectro de la medicion
Fs=200;
x=(16e6*60./beta);
% x=x-mean(x);
t=0:1/Fs:(N-1)/Fs;
figure();plot(t,x,'.');

y=fftshift(fft(x))/length(x);
f=linspace(-Fs/2,Fs/2,length(y));
% figure();plot(f,abs(y),'.');
%% Separo los tiempos en dos partes
t1=[];
ind1=[];
t2=[];
ind2=[];
for k=1:length(beta)
    if beta(k)>7e5
        t1=[t1 beta(k)];
        ind1=[ind1 k];
    else
        t2=[t2 beta(k)];
        ind2=[ind2 k];
    end
end
figure();plot(ind1,t1,'r.',ind2,t2,'b.');
%%
M=18*3;
figure();subplot(211);plot(beta(1:M),'o');hold on;plot(beta(1:M));
subplot(212);plot(beta(M+1:2*M),'o');hold on;plot(beta(M+1:2*M));
%%
suma=[];
for k=1:length(beta)/2
    suma=[suma beta(2*k-1)+beta(2*k)];
end
figure();plot(suma/16e6,'.')