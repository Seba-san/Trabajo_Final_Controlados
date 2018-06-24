%% Inicio
s=InicializacionSerial_rf('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
%addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
%cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%s=InicializacionSerial('COM6',115200);%Velocidad: 115200 baudios
%% Fin
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%clear all;close all;clc
disp('Puerto Cerrado')
%% Ensayo al escalon
Comunic_test_rf(s)
pause(4)
Env_instruccion(s,'control_on');
PWM=2;
Env_instruccion(s,'PWM',[PWM PWM])
%Env_instruccion(s,'control_off'); 
RPM=400;
for i=1:RPM/10
Env_instruccion(s,'setpoint',[i*10 i*10]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
pause(0.1);
end
%Env_instruccion(s,'control_off'); 
%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
N=250; %Cantidad de muestras
%RPMA=zeros(1,N);
%RPMB=zeros(1,N);
Fs=200;%Frec de env�o de datos
tiempo=0:1/Fs:(N-1)/Fs;%Genero a mano el vector de tiempos (asumo que el nano trabaja a frec cte sin problema)
wA=[];
wB=[];
betas=[];
%Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
control=[];   
PWMB=[]; 
PWM=400;
dW=100;
%Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
% for i=1:PWM/5
% Env_instruccion(s,'setpoint',[i*5 i*5])
% pause(0.05)
% end
%pause(0.1);
Env_instruccion(s,'setpoint',[PWM PWM]);
Env_instruccion(s,'control_on');

Comunic_test_rf(s)

flushinput(s);
Env_instruccion(s,'online');
for i=1:N %Ojo con el tope que pone el nano
  % s.BytesAvailable
    if i==80
        %PWM=60;
       % Env_instruccion(s,'setpoint',[PWM PWM]); 
      flushinput(s);  %Vacia el buffer de entrada
      Env_instruccion(s,'control_off'); 
      Env_instruccion(s,'setpoint',[PWM-dW/2 PWM+dW/2]); 
    end
    [datos,inicio]=DatoRx_rf(s);
    if inicio==255
         control=[control 0];
    else
   control=[control 1];
    end
    %flushinput(s);
    beta=ConversionSensor(datos(1),0) ;
    w=datos(2); wA=[wA w]; 
    w=datos(3); wB=[wB w];    
    %PWMA=[PWMA datos(1)];PWMB=[PWMB datos(1)];
    betas=[betas beta];
   
end
% Conversion de datos
%Parametros.PWMB=wA;
Parametros.Fs=200;
Parametros.dWdes=dW;
Parametros.Wref=PWM;
Parametros.Controlador={200,0,0,0,0};

Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'setpoint',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
direccion='../../Mediciones/';
name1='respuesta_escalon_systot_scontrolador';
name2='_';
name=strcat(direccion,name,name1,name2,'.mat');
%save(name,'tiempo','wA','wB','control','betas','Parametros') 
figure(1)
yyaxis left
plot(tiempo,wB,'r.',tiempo,wA,'b.');ylim([100 1200])
ylabel('Vel Ang (rpm)'); legend('Motor B','Motor A','Location','northwest')
yyaxis right
plot(tiempo,betas,'r.',tiempo,control,'b');ylim([-1 3.1])
%legend('Señal de PWM');
title('Respuesta escalón ambos Motores');
xlabel('tiempo (s)'); %Revisar la unidad!! $
grid on;