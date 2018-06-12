%% Inicio
% s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
%Env_instruccion(s,'stop')}
Comunic_test(s)
% Env_instruccion(s,'PWM',[50 100]); 
% pause(1)

RPM=400; 
RPM2=400;RPM2=RPM;
Env_instruccion(s,'setpoint',[RPM,RPM]); 
N=36*10;
medicion=zeros(1,N);
control=zeros(1,N);
i=1;
RPMn=zeros(1,N);RPMn(1)=RPM;
a=1;veces=1;
pause(1)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
%     close(1)
end
figure(1)
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
flushinput(s);
veces_=0;
while (veces_<veces)
    %figure(1)
    
    
    medicion(i)=str2double(fscanf(s));
    control(i)=str2double(fscanf(s));
    %medicion(i)=16e6*60/suma;
    flushinput(s);
%     flushinput(s);
%    e2=str2double(fscanf(s))
%     e1=str2double(fscanf(s))
%     e0=str2double(fscanf(s))
    % setpoint=str2double(fscanf(s))
    
    m1=1:1:i;m2=i+1:1:N;
    %figure(1)
    subplot(211)
    plot(m1,medicion(1:i),'.','color',[~a 0 a]); hold on;plot(m2,medicion(i+1:N),'.','color',[a 0 ~a]); hold off;%ylim([0 1200])
    ylabel('RPM')
    %     ylim([limite_inf limite_sup]);
 % figure(2)
    subplot(212)
    plot(m1,control(1:i),'.','color',[~a 0 a]); hold on;plot(m2,control(i+1:N),'.','color',[a 0 ~a]); hold off; %ylim([0 110]);
   ylabel('U_{control}')
    pause(0.001)
    i=i+1;
    if (i>(N-1) ) 
        i=2;a=~a;
        veces_=veces_+1;
        
    end
    
%     if i==N/2
%         RPMn(i)=RPM2;
%         Env_instruccion(s,'setpoint',[RPM2,RPM2]); 
%     else
%          RPMn(i)= RPMn(i-1);
%     end
    
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',[0,0]); 
% pause(1)
% close all
%Env_instruccion(s,'PWM',[0 0]); 

%% Espectro de la medicion
Fs=200;
x=(16e6*60./medicion);
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
for k=1:length(medicion)
    if medicion(k)>7e5
        t1=[t1 medicion(k)];
        ind1=[ind1 k];
    else
        t2=[t2 medicion(k)];
        ind2=[ind2 k];
    end
end
figure();plot(ind1,t1,'r.',ind2,t2,'b.');
%%
M=18*3;
figure();subplot(211);plot(medicion(1:M),'o');hold on;plot(medicion(1:M));
subplot(212);plot(medicion(M+1:2*M),'o');hold on;plot(medicion(M+1:2*M));
%%
suma=[];
for k=1:length(medicion)/2
    suma=[suma medicion(2*k-1)+medicion(2*k)];
end
figure();plot(suma/16e6,'.')