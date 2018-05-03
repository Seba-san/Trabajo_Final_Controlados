
%% Chequeo de control
error=[0,0,0];
u=[0,0,0];
Parametros=[0.007, 0,0, 1,0];
for m=1:length(medicion)
    freq=medicion(m);
for(k=1:2)%int k=0;k<2;k++)
   error(k)=error(k+1);%Desplazamiento a la derecha de los datos del buffer
   u(k)=u(k+1); 
end
error(3)=((set_point)-freq);
u(3)=Parametros(1)*error(3)+Parametros(2)*error(2)+Parametros(3)*error(1)+Parametros(4)*u(2)+Parametros(5)*u(1);
if (u(3)>100)
    u(3)=100;
elseif (u(3)<10)
    u(3)=10;
end
sen_control(m)=u(3);
end
%%

% plot(medicion,'.')
%s=InicializacionSerial('/dev/ttyUSB0',115200);%Velocidad: 115200 baudios

s=InicializacionSerial('/dev/ttyUSB0',1000000);
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')
%%
set_point=700;
error=[0,0,0];
u=[0,0,0];
N=1000;

Parametros=zeros(1,5);
%Parametros=[0.4673 ,   0.1130  ,  0.0282, 1,0];
%Parametros(1)=0.389381;% Z-N P
%Parametros=[0.350537, -0.350442, 0 ,1.000000,0]; % Z-N PI
Parametros=[2.352285, -4.233923, 1.883333, 1.000000, -0.000000]; %  Z-N
%PID
%Parametros=[56.967313, -113.467257, 56.500000, 1.000000, -0.000000]; % Z-N Con Fs=2kHz.
Con_Z_N=0;
Parametros=[0.020719, 0.008004, -0.012715, 1.000000, -0.000000]; % Parametros con PID-TUNE
% Parametros=[0.026080, -0.031646, 0.007538, 1.868783, -0.868783]; %Con PID-TUNE y PIDF

Env_instruccion(s,'online')
frecuencia=zeros(1,N);
ucontrol=zeros(1,N);
error_hist=zeros(1,N);
flushinput(s);
for mu=1:N
    tu=tic;
     freq=str2double(fscanf(s));   
     flushinput(s);
for(k=1:2)%int k=0;k<2;k++)
   error(k)=error(k+1);%Desplazamiento a la derecha de los datos del buffer
   u(k)=u(k+1); 
end
error(3)=((set_point)-freq);
u(3)=Parametros(1)*error(3)+Parametros(2)*error(2)+Parametros(3)*error(1)+Parametros(4)*u(2)+Parametros(5)*u(1);
%Seba dice que quizás ZN está trabajando como si ...
if Con_Z_N
    if u(3)>1000
        u(3)=1000;
    elseif (u(3)<100)
        u(3)=100;
    end
    pwm=round(u(3)/k0);
else
    if (u(3)>100)
        u(3)=100;
    elseif (u(3)<10)
        u(3)=10;
    end
    pwm=u(3);
end
    Env_instruccion(s,'Ucontrol',pwm);
    frecuencia(mu)=freq;
    ucontrol(mu)=u(3);
    error_hist(mu)=error(3);
    %toc(tu)
end
Env_instruccion(s,'stop');
Env_instruccion(s,'Ucontrol',10);
figure(1)
subplot(311);plot(frecuencia,'.');ylim([0 1000]);
title('Velocidad medida (en rpm)')
subplot(312);plot(ucontrol,'.');
if Con_Z_N
ylim([0 1000]);
title('Señal de control (en rpm)')
else
    ylim([0 100]);
    title('Seal de control (en % de PWM)')
end
subplot(313);plot(error_hist,'.');
title('Historial del error')
xlabel('Muestras')