% s=InicializacionSerial('/dev/ttyUSB0',1000000);
s=InicializacionSerial('/dev/ttyUSB1',115200);
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')
%%
% Prueba para medir la presicion del metodo. Hay que estandarizarla. 
N=10000;
Env_instruccion(s,'online');
Env_instruccion(s,'Ucontrol',10);
freq=zeros(1,N);
tocc=zeros(1,N);
flushinput(s);
for i=1:N
    %flushinput(s);
    freq(i)=str2double(fscanf(s));
    tocc(i)=str2double(fscanf(s));
    if i==1
        ta=tic;
    end
    
end
tm_tt=toc(ta)
Env_instruccion(s,'stop'); 
Env_instruccion(s,'Ucontrol',10);
tiemposs=1./freq;
tm_tt_u=sum(tiemposs)
a=tocc-tocc(1);
max(a)*1e-6
sum(freq)*(16e6)^-1
plot(freq,'.')



%%

%Prueba para medir que tan bien anda el controlador.



%Env_instruccion(s,'Ucontrol',10);
Env_instruccion(s,'setpoint',100);
pause(5)
N=1000;
freq=zeros(1,N);
Comunic_test(s)
Env_instruccion(s,'setpoint',500);
Env_instruccion(s,'online');

flushinput(s);
for i=1:N
    %flushinput(s);
    freq(i)=str2double(fscanf(s));  
end
plot(freq,'.')
Env_instruccion(s,'stop');
Env_instruccion(s,'setpoint',100);