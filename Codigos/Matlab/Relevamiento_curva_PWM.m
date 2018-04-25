%% Inicializaci�n de serial
clear all;clc;close all;
cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%% Barrido de PWM motor A
% Le digo al micro que ponga el PWM en un dado valor y que mida la
% velocidad obtenida. Hago esto para varios valores para obtener una curva
% de respuesta.
% Para eso uso Env_instruccion, envi�ndole: Env_instruccion(s,'PWM',[PWMA,
% PWMB]), donde PWMA y PWMB son los PWM de cada motor.
R=20;%Cantidad de muestras a tomar por cada valor de PWM
wA=[];
PWMA=[];
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
                            %sin identificador de trama
for PWM=0:1:100 %Ojo con el tope que pone el nano
    Env_instruccion(s,'PWM',[PWM 0]);
    clc;disp(['PWM= ' num2str(PWM)])
    pause(0.1);
    for k=1:R
        dato=zeros(16,1);%Borro lo que hab�a en datos
        for m=1:16
            dato(m)=DatoRx_online(s);
        end
        tiempo=sum(dato)/8;
        wA=[wA 16e6/tiempo];%Frec angular en Hz
        PWMA=[PWMA PWM];
        %Ac� no s� si ponerle un delay $
    end
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
name=strcat(name,'.mat');
%save(name,'wA','PWMA') 
%% Gr�fico de la Respuesta del motor B
figure(1);
plot(PWMA,wA,'.');
title('Respuesta del Motor B');
xlabel('RPM');ylabel('frec de giro') %Revisar la unidad!! $
