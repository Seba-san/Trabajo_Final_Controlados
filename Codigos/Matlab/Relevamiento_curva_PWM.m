%% Inicializaciï¿½n de serial
clear all;clc;close all;
% cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cuï¿½l puerto tengo conectado con esta instrucciï¿½n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%% Barrido de PWM motor A
% Le digo al micro que ponga el PWM en un dado valor y que mida la
% velocidad obtenida. Hago esto para varios valores para obtener una curva
% de respuesta.
% Para eso uso Env_instruccion, enviï¿½ndole: Env_instruccion(s,'PWM',[PWMA,
% PWMB]), donde PWMA y PWMB son los PWM de cada motor.
R=20;%Cantidad de muestras a tomar por cada valor de PWM
wA=[];
PWMA=[];
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
                            %sin identificador de trama
for PWM=0:1:100 %Ojo con el tope que pone el nano
    Env_instruccion(s,'PWM',[0 PWM]);
    clc;disp(['PWM= ' num2str(PWM)])
    pause(0.1);
    for k=1:R
        dato=zeros(16,1);%Borro lo que habï¿½a en datos
        for m=1:16
            dato(m)=DatoRx_online(s);
        end
        wA=[wA dato];%Frec angular en Hz
        PWMA=[PWMA PWM];
        %Acï¿½ no sï¿½ si ponerle un delay $
    end
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
name=strcat('curva_ent_sal_motorB',name,'.mat');
%save(name,'wA','PWMA') 
%% Gráfico de la Respuesta del motor
figure();
a=201;b=1602;%Para limitar el PWM al rango de 10 a 80%
f=fit(PWMA(a:b)',mean(wA(:,a:b))','poly1');%Ajuste recta
plot(f,'r');hold on;plot(PWMA,wA,'b.');
legend('Recta Ajustada','Datos Medidos')
title('Respuesta del Motor A');
xlabel('PWM');ylabel('frec de giro (rpm)')