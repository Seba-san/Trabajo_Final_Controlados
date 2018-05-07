% s=InicializacionSerial('/dev/ttyUSB0',1000000);
s=InicializacionSerial('/dev/ttyUSB1',115200);
%% Fin
fclose(s)
%clear s;
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


%%

%Mido la frecuencia de 100Hz.
N=10;
freq=zeros(1,N);
suma=zeros(1,N);
Env_instruccion(s,'trama');
flushinput(s);
i=0;
clc
cuenta=0;
while( i<N)
    %flushinput(s);
    %freq(i)=str2double(fscanf(s));  
    %suma(i)=str2double(fscanf(s)); 
    Dato=DatoRx(s);
    r=32*(Dato.datos(3)*Dato.datos(2)+Dato.datos(5)-Dato.datos(4));
    if r~=Dato.datos(1)
        (r-Dato.datos(1))/(32*Dato.datos(2))
        Dato.datos
        cuenta
        cuenta=0;
        
       % Dato
         i=i+1;
    else
        cuenta=cuenta+1;
    end
    

end
Env_instruccion(s,'stop');
%figure(1);plot(freq,'b.')
%figure(2);plot(16e6*60*1./suma,'b.')

%%

%Mido la frecuencia de 100Hz.
N=1000;
freq=zeros(1,N);
suma=zeros(1,N);
Env_instruccion(s,'online');
flushinput(s);
i=0;
clc
cuenta=0;
for i=1:N
    %flushinput(s);
    freq(i)=str2double(fscanf(s));  
end
Env_instruccion(s,'stop');
figure(1);plot(freq,'b.')
%figure(2);plot(16e6*60*1./suma,'b.')

%%
aux[0]=suma;
aux[1]=_OCR2A;
aux[2]=cantOVerflow_actual;
aux[3]=TCNT2anterior;
aux[4]=TCNT2actual;

%%

for i=1: 10
32*(Dato(i).datos(3)*Dato(i).datos(2)+Dato(i).datos(5)-Dato(i).datos(4))
Dato(i).datos(1)
end