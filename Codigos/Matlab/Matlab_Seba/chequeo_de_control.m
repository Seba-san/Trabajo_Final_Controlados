
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
figure(1);plot(sen_control,'.')
% plot(medicion,'.')

%%
set_point=800;
error=[0,0,0];
u=[0,0,0];
Parametros=[0.007, 0,0, 1,0];
Env_instruccion(s,'online')
frecuencia=zeros(1,1000);
ucontrol=zeros(1,1000);
flushinput(s);
for mu=1:1000
     freq=str2double(fscanf(s));   
     flushinput(s);
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
    Env_instruccion(s,'Ucontrol',round(u(3)))
    frecuencia(mu)=freq;
    ucontrol(mu)=round(u(3));
end
Env_instruccion(s,'stop');
 Env_instruccion(s,'Ucontrol',0);
 figure(1)
plot(frecuencia,'.');ylim([0 900])
figure(2)
plot(ucontrol,'.');ylim([0 100])