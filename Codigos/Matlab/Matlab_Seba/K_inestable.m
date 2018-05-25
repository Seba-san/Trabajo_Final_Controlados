%% K inestable

%% Configuracion inicial
cd /media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba
%cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Codigos\Matlab')%compu Tania

addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab');
%% Inicio
% uart = serial('COM2','BaudRate',1200,'DataBits',7);
%s = serial('COM5');
% Hay que agregar el path!!
%InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.

s=InicializacionSerial('/dev/ttyUSB1',115200);%Velocidad: 115200 baudios
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')

%%


Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
Comunic_test(s)
 %Env_instruccion(s,'PWM',[10 100]); 
% pause(1)
Env_instruccion(s,'setpoint',[500 500]); 
N=50;
medicion=zeros(1,N);
control=zeros(1,N);
i=1;
a=1;veces=0;
pause(1)
limite=1200; %limite del grafico
try
    close(1)
end
figure(1)
k=0.1+0.15;
 kmas=0.15;
 Env_instruccion(s,'Kmas',kmas); %valor maximo  0.255
while  (1)%(veces<1)
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
    plot(m1,medicion(1:i),'.','color',[~a 0 a]); hold on;plot(m2,medicion(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([0 limite]);
    pause(0.001)
    i=i+1;
    if (i>N ) 
        i=1;a=~a;
        veces=veces+1;
        kmas=0.001;
        Env_instruccion(s,'Kmas',kmas); %valor maximo  0.255
        k=k+kmas
        %pause()
    end
    
    
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',[0 0]); 
disp('listo')
