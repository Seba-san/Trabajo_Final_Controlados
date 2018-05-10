%% Inicio
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios
s=InicializacionSerial('COM5',115200);%Velocidad: 115200 baudios
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')
%%
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama
%Env_instruccion(s,'stop')}
Comunic_test(s)
 %Env_instruccion(s,'PWM',[30 30]); 
% pause(1)
Env_instruccion(s,'setpoint',500); 
N=400;
medicion=zeros(1,N);
control=zeros(1,N);
i=1;
a=1;veces=0;
pause(1)
limite_sup=1e3; %limite del grafico
limite_inf=0;
try
    close(1)
end
figure(1)
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
    plot(m1,medicion(1:i),'.','color',[~a 0 a]); hold on;plot(m2,medicion(i+1:N),'.','color',[a 0 ~a]); hold off; ylim([limite_inf limite_sup]);
    pause(0.001)
    i=i+1;
    if (i>N ) 
        i=1;a=~a;
        veces=veces+1;
    end
    
    
end
Env_instruccion(s,'stop'); 
Env_instruccion(s,'setpoint',0); 
pause(1)
close all
