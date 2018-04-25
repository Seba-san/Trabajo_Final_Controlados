%% Inicializacion de serial
clear all;clc;close all;
cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%% Ensayo al escalon
N=200;%Cantidad de muestras a tomar
wA=[];
tiempo=[];
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
PWMA=[];                   
PWM=10;
Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
pause(1);
flushinput(s);  %Vacia el buffer de entrada
for k=1:N %Ojo con el tope que pone el nano
    if k==16
        PWM=100;
        Env_instruccion(s,'PWM',[PWM PWM]);
    end
    dato=str2double(fscanf(s));%Actualmente el nano est� enviando la vel en rpm y el tiempo en us.
    wA=[wA dato];
     dato=str2double(fscanf(s));
    tiempo=[tiempo dato];
    PWMA=[PWMA PWM];
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
name=strcat(name,'.mat');
save(name,'tiempo','PWMA','wA') 
%% Gr�fico de la Respuesta del motor B
figure(1);
plot(tiempo,PWMA,tiempo,wA,'.');
%legend('Se�al de PWM','Se�al de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo (us)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
%% Acomodando Ts para estimacion del sistema
% Como la estimacion de Matlab se hace para una senal muestreada a Fs cte,
% generamos una version que guarde el valor anterior mas cercano para cada
% valor del vector de tiempos constante.
Fs=2e3;
Ts=1/Fs;
tiempoSeg=tiempo*1e-6;
t=min(tiempoSeg):1/Fs:max(tiempoSeg);
for k=1:length(t)
    [~,indice]=min(abs(tiempoSeg-t(k)));%Busco la muestra mas cercana al valor de tiempo en el que estoy mirando
    if tiempoSeg(indice)-t(k)>0 && indice>1
        indice=indice-1;%Me aseguro que siempre se quede con el valor mas cercano pero anterior, no futuro
    end
    w(k)=wA(indice);%Guardo el valor de wA anterior mas proximo al valor de tiempo que estoy considerando
    PWM(k)=PWMA(indice);
end
figure(1);plot(t,w,'.',tiempoSeg,wA);
%figure(1);plot(t,PWM,'.',tiempoSeg,PWMA);
%% Estimacion Sist
% transformo la informacion al formato iidata:
% data = iddata(w',PWM',Ts);

data = iddata((w-w(1))',(PWM-PWM(1))',Ts);%Le corro el eje y para que arranque en 0.
%Nota: no funcion� :( . Para 2 polos 1 cero parece que da exactamente el
%mismo sistema que la versi�n no desplazada.

% data = iddata(y,u,Ts) creates an iddata object containing a time-domain
% output signal y and input signal u, respectively. Ts specifies the sample
% time of the experimental data.
% Obs: tienen que ser vectores columna!! Sino lo toma como 5millones de
% entradas/salidas.
np=2;%Le indico el nro de polos
nz=[];%Le indico el nro de ceros
iodelay=0;%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts);
%% Prueba del sist estimado
we=filter(sys.Numerator,sys.Denominator,PWM);
figure();plot(t,we,t,w,'.');
%% PID