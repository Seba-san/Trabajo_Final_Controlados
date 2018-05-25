%% Inicio
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
s=InicializacionSerial('COM5',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
Comunic_test(s)
Env_instruccion(s,'PWM',[0 5]); 
% pause(1)
% Env_instruccion(s,'setpoint',[0,100]); 
N=36*10;
medicion=zeros(1,N);
control=zeros(1,N);
i=1;
a=1;veces=0;
pause(1)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
%     close(1)
end
figure()
 flushinput(s);
while (veces<1)
    %figure(1)
    
    
    medicion(i)=str2double(fscanf(s));
   % control(i)=str2double(fscanf(s));
    %medicion(i)=16e6*60/suma;
    flushinput(s);
%     flushinput(s);
%    e2=str2double(fscanf(s))
%     e1=str2double(fscanf(s))
%     e0=str2double(fscanf(s))
    % setpoint=str2double(fscanf(s))
    
    m1=1:1:i;m2=i+1:1:N;
    %figure(1)
    plot(m1,medicion(1:i),'.','color',[~a 0 a]); hold on;plot(m2,medicion(i+1:N),'.','color',[a 0 ~a]); hold off;
%     ylim([limite_inf limite_sup]);
 % figure(2)
   % plot(m1,control(1:i),'.','color',[~a 0 a]); hold on;plot(m2,control(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([0 110]);
   
    pause(0.001)
    i=i+1;
    if (i>N ) 
        i=1;a=~a;
        veces=veces+1;
    end
    
    
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',[0,0]); 
% pause(1)
% close all
Env_instruccion(s,'PWM',[0 0]); 

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
M=18*10;
figure();subplot(211);plot(medicion(1:M),'o');hold on;plot(medicion(1:M));
subplot(212);plot(medicion(M+1:2*M),'o');hold on;plot(medicion(M+1:2*M));