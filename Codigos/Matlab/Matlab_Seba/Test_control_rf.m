%% Inicio
s=InicializacionSerial_rf('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
%s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
% Probar si esta clase de instrucciones funcionan, porque el terminador
% cambio:
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
Comunic_test(s)
% Env_instruccion(s,'PWM',[50 100]); 
% pause(1)

RPM=400; 
RPM2=400;RPM2=RPM;
Env_instruccion(s,'setpoint',[RPM,RPM]); 

N=36*10;
angulo=zeros(1,N);
RPMA=zeros(1,N);
RPMB=zeros(1,N);
i=1;

a=1;veces=1;
pause(1)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
     close(1)
end
figure(1)
 flushinput(s);
 veces_=0;
while (veces_<veces)
   % Supongo [angulo][RPMA][RPMB]
    datos=DatoRx_rf(s); 
    angulo(i)=datos(1);RPMA(i)=datos(2);RPMB(i)=datos(3);
    
    m1=1:1:i;m2=i+1:1:N;
    subplot(311)
    plot(m1,medicion(1:i),'.','color',[~a 0 a]); hold on;plot(m2,medicion(i+1:N),'.','color',[a 0 ~a]); hold off;%ylim([0 1200])
   % ylabel('RPM')
    subplot(312)
    plot(m1,control(1:i),'.','color',[~a 0 a]); hold on;plot(m2,control(i+1:N),'.','color',[a 0 ~a]); hold off; %ylim([0 110]);
 %  ylabel('U_{control}')
    subplot(313)
   plot(m1,control(1:i),'.','color',[~a 0 a]); hold on;plot(m2,control(i+1:N),'.','color',[a 0 ~a]); hold off; %ylim([0 110]);
%   ylabel('U_{control}')
    pause(0.001)
    
    i=i+1;
    if (i>(N-1) ) 
        i=2;a=~a;
        veces_=veces_+1;
        
    end
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',[0,0]); 


