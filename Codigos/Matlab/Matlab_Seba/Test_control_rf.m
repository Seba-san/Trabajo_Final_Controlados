%% Inicio
%s=InicializacionSerial_rf('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
%addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
%cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial_rf(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios


%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%%
% Probar si esta clase de instrucciones funcionan, porque el terminador
% cambio:
% Condicion inicial
solo_sens=0;
N=36*5;
wref=500;
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'control_off');
pause(0.1);
 flushinput(s);
pause(2)
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
Comunic_test_rf(s)
 PWM=20;
 if (~solo_sens); Env_instruccion(s,'PWM',[PWM PWM]); end %Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
 
% pause(1)

% RPM=400; 
% RPM2=400;RPM2=RPM;
% Env_instruccion(s,'setpoint',[RPM,RPM]); 
% 

angulo=zeros(1,N);
RPMA=zeros(1,N);
RPMB=zeros(1,N);
i=1;

a=1;veces=1;
%pause(0.5)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
     close(1)
end
PWM=wref;
%wref=PWM;
 if (~solo_sens);Env_instruccion(s,'setpoint',[PWM PWM]);end
%pause(0.5)
 if (~solo_sens);Env_instruccion(s,'control_on');end
figure(1)
 flushinput(s);
 veces_=0;
 t=tic;
while (veces_<veces)
   % Supongo [angulo][RPMA][RPMB]
    datos=DatoRx_rf(s);
    flushinput(s);
     if (solo_sens)
    beta=datos(1);
    dec2bin(datos(1))
     end
    beta=ConversionSensor(datos(1),0) ;
    angulo(i)=beta;RPMA(i)=datos(2);RPMB(i)=datos(3);
    
    m1=1:1:i;m2=i+1:1:N;
    subplot(311)
    plot(m1,angulo(1:i),'.','color',[~a 0 a]); hold on;plot(m2,angulo(i+1:N),'.','color',[a 0 ~a]); hold off;ylim([-2 4])
    ylabel('angulo (rad)')
    subplot(312)
    plot(m1,RPMA(1:i),'.','color',[~a 0 a]); hold on;plot(m2,RPMA(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([0 1000]);
   %ylabel('U_{control}')
   ylabel('RPMA')
    subplot(313)
   plot(m1,RPMB(1:i),'.','color',[~a 0 a]); hold on;plot(m2,RPMB(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([0 1000]);
%   ylabel('U_{control}')
    ylabel('RPMB')
    pause(0.001)
    
    i=i+1;
    if (i>(N-1) ) 
        i=2;a=~a;
        veces_=veces_+1;
        
    end
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'control_off');
toc(t)
b=1;
while (b)
    datos=DatoRx_rf(s);
    if datos(2)<100 && datos(3)<100
        Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
        disp('FIN')
        b=0;
    end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'control_off');
pause(0.1);
    
end
dWw=RPMB-RPMA;
figure(2)
plot(dWw)
dWwMax=max(dWw)
Vmed=mean((RPMB+RPMA)/2)
pierde_linea=find(angulo==3);
l=length(angulo);n=1;
for i=1:l
    
    if (angulo(i)~=3)
    
    angulo2(n)=angulo(i);
    n=n+1;
    end
end

var_angulo=var(angulo2)
%save('../../Mediciones/Rendimiento/controlador_5.mat','angulo','RPMB','RPMA','wref')
%% Frenar
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
disp('FIN')
%%
dWw=RPMB-RPMA;
figure(2)
plot(dWw)
