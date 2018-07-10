%% Antes de empezar poner esto
addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%%
% Simulacion Robot diferencial

L=0.16;%[m]
r=0.015;%[m]
tita=0;

% matriz de cinematica directa: [dV dtita]=A*[wr wl]
A=[r*cos(tita)/2 r*cos(tita)/2;...
    r*sin(tita)/2 r*sin(tita)/2;...
    r/L -r/L ];
% matriz de cinematica inversa: [wr wl]=A*[dV dtita]; A+=(A'*A)^-1 *AT
Ainv=[cos(tita)/r sin(tita)/r L/r;...
    cos(tita)/r sin(tita)/r -L/r];

robot.x=[0 1 1 0];
robot.y=[0 0 1 1];
plot(robot.x, robot.y)
%%
%load('/tmp/180607213207resp_escalon_sistema_total.mat')
%load('/home/seba/Dropbox/Facultad/Trabajo_Final_Controlados_git/180607231105resp_escalon_sistema_total.mat')
%load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180622220141respuesta_escalon_systot_scontrolador_.mat')
% Este procesamiento es necesario para las ultimas mediciones
%indice=find(control==0);betas2=betas(indice);wA2=wA(indice);wB2=wB(indice);tiempo2=tiempo(indice);
%indice=find(betas2<3);betas2=betas2(indice);wA2=wA2(indice);wB2=wB2(indice);tiempo2=tiempo2(indice);
%beta=betas2;wA=wA2;wB=wB2;
% #########################################
% Ejemplo de como seria si se conociera la orientacion en todo momento (usando odometria por ej.)
%close all
L=0.16;%[m]
%r= 0.01;%Valor de prueba
r= 0.015;%Valor estimado con el system identification.
%r=0.015;%[m]
T=0.075; %[m] Distancia del centro al sensor.
tita=0;
Ts=1/200;%Ts=1/Parametros.Fs;
% ## Resolucion por partes
%w=[wB; wA];
try
    close (1)
end


N=300;
% Calculo de velocidades y dtita
Vel=zeros(3,N);
Pos=zeros(3,N);
Pos_lin=zeros(2,N);

% Condiciones iniciales
wA0=300;dW=0;
wB0=300;
tita=10;
tita=tita*pi/180; Pos(3,1)=tita; Pos(3,2)=tita; % Condicion inicial
w=[wB0; wA0];w_v=[];
kp=10;ki=0.1;
for i=1:N-1
    
    % Actualizo el valor de A
    A=[r*cos(Pos(3,i))/2 r*cos(Pos(3,i))/2;...
    r*sin(Pos(3,i))/2 r*sin(Pos(3,i))/2;...
    r/L -r/L ];
    % Calculo de las velocidades
    Vel(:,i)=A*w;%(:,i);   
    % Calculo de las posiciones
    Pos(:,i+1)=Vel(:,i)*Ts+Pos(:,i); 
    % Acoto el angulo, para que no diverja
   dd=fix(Pos(3,i+1)/(2*pi));
   % if dd>0
        Pos(3,i+1)=Pos(3,i+1)-2*pi*dd;
   % end
     
   % Pos(3,i+1)=asin(Pos(2,i)/T)+Pos(3,i); % Obtengo la medicion del beta
    dW=-kp*Pos(3,i+1)+ki*dW; % Accion de control
    % Antiwindup
    if (dW>300 ); dW=300;  end
    if dW<-300 ;  dW=-300;  end
    ddW(i)=dW;
    w_v=[w_v w];
    w=[w(1)+dW/2;w(2)-dW/2];
    % Grafico posicion
    %subplot(211)
   
    plot(Pos(1,1:i-1),Pos(2,1:i-1),'.k',Pos(1,i),Pos(2,i),'rx');hold on;
    % Grafico orientacion
    Ox=0.1*cos(Pos(3,i)) ;Oy=0.1*sin(Pos(3,i));
    quiver(Pos(1,i),Pos(2,i),Ox,Oy);hold off
    ylim([-0.3 0.3]);ylabel('[m]'); xlim([0 5]);xlabel('[m]');
    title ('Posicion espacial mas orientacion')
%     subplot(212)
%     plot(ddW(1:i),'.')
    Ang=[Pos(3,i) Vel(3,i)]*180/pi
    pause(0.5)
  
end
 %plot(Pos(1,:),Pos(2,:),'b');%hold on;%ylim([-0.1 0.1])
 %Pos(3,:)
%plot(Pos(1,:),Pos(2,:),'r.',Pos_lin(1,:),Pos_lin(2,:),'b.');%ylim([-0.3 0.3])
% hold on
% quiver(Pos(1,:),Pos(2,:),Vel(1,:),Vel(2,:))
% hold off
%%
% #########################################
% Ejemplo de como seria si se conociera la orientacion en todo momento (usando odometria por ej.)
close all;
clear all; clc
L=0.16;%[m]
%r= 0.01;%Valor de prueba
r= 0.015;%Valor estimado con el system identification.
%r=0.015;%[m]
T=0.075; %[m] Distancia del centro al sensor.
tita=0;
Ts=1/200;%Ts=1/Parametros.Fs;
% ## Resolucion por partes
%w=[wB; wA];
try
    close (1)
end


N=300;
% Calculo de velocidades y dtita
Vel=zeros(3,N);
Pos=zeros(3,N);
Pos_lin=zeros(2,N);

% Condiciones iniciales
wA0=300;dW=0;
wB0=300;
tita=-1;
tita=tita*pi/180; Pos(3,1)=tita; Pos(3,2)=tita; % Condicion inicial
w=[wB0; wA0];w_v=[wB0 wB0; wA0 wA0];
kp=200;ki=0;kd=0;
d=1;bb=1;
for i=1:N-1
    
    %  ### Actualizo el valor de A
    A=[r*cos(Pos(3,i))/2 r*cos(Pos(3,i))/2;...
    r*sin(Pos(3,i))/2 r*sin(Pos(3,i))/2;...
    r/L -r/L ];
    % ### Calculo de las velocidades
    Vel(:,i)=A*w;%(:,i);   
    % ### Calculo de las posiciones FUTURAS
    Pos(:,i+1)=Vel(:,i)*Ts+Pos(:,i); 
    % ### Acoto el angulo, para que no diverja
 %  dd=fix(Pos(3,i+1)/(2*pi));   
 %  Pos(3,i+1)=Pos(3,i+1)-2*pi*dd;
   
   % ### Obtengo medicion del sensor
   y0=Pos(2,i);tit0=Pos(3,i);
   % No vale para todos los casos
   if y0*tit0>0 % tienen el mismo signo
       m=(T*sin(tit0)+y0)*1/cos(tit0);
   else
       if y0<0; bb=-1;else; bb=1; end 
       m=bb*(y0^2-T^2+((T-y0*sin(tit0))*(1/cos(tit0)))^2)^0.5;
   end
   if y0*tit0>0 && d<0 || y0*tit0<0 && d>0
       d=d*-1;
       disp('cambio de algoritmo '); d
   end
   %m=(y0+T/sin(tit0))*(1/cos(tit0)); % hacer la trigonometria para entenderlo
   
    % Paso de m a beta
   beta=atan(m/T);
   
   % ### Control
    dW=-kp*beta+ki*dW+kd*(-(w_v(1,i)-w_v(2,i))+(w_v(1,i+1)-w_v(2,i+1)))/Ts; % Accion de control
   % ### Antiwindup
    if (dW>300 ); dW=300;  end
    if dW<-300 ;  dW=-300;  end
    ddW(i)=dW;
    w_v=[w_v w];
    w=[wB0+dW/2;wA0-dW/2];
    
    
    
    % ### Grafico posicion
    %subplot(211)
   
    plot(Pos(1,1:i-1),Pos(2,1:i-1),'.k',Pos(1,i),Pos(2,i),'rx',[0 10],[0 0],'-b');hold on;
    % ### Grafico orientacion
    Ox=0.1*cos(Pos(3,i)) ;Oy=0.1*sin(Pos(3,i));
    quiver(Pos(1,i),Pos(2,i),Ox,Oy);hold off;
    offy=0.5;offx=5;
    ylim([Pos(2,i)-offy Pos(2,i)+offy]); xlim([Pos(1,i)-offx*4/5 Pos(1,i)+offx/5]);
    ylabel('[m]');xlabel('[m]');
    title ('Posicion espacial mas orientacion')
%     subplot(212)
%     plot(ddW(1:i),'.')
    %Ang=[Pos(3,i) Vel(3,i)]*180/pi
    posi=[m Pos(2,i) dW Pos(3,i)*180/pi i]
    m2(i)=m;
    pause(0.05)
  
end
%%
figure
plot(m2,'.')
title('m')
figure
plot(w_v','.')
title('vel angular')
figure
plot(Pos(3,:),'.')
title('pos angular')

%%
% #########################################
close all;
clear all; clc
% En este ejemplo en vez de seguir una linea recta, sigue una trayectoria
% definida por lin(x,y).
N=300;
aux=0:0.01:10;
n=length(aux);
lin=zeros(2,n);
% Armo la circunferencia centrada en x=2 y de radio=0.4
radio=2;x0=2;
ind=find(aux==x0,1);lin(1,:)=aux;
lin(2,ind:ind+300)=(radio^2-(lin(1,ind:ind+300)-x0).^2).^(0.5)-radio;
L=0.16;%[m]
%r= 0.01;%Valor de prueba
r= 0.015;%Valor estimado con el system identification.
%r=0.015;%[m]
T=0.075; %[m] Distancia del centro al sensor.
tita=0;
Ts=1/200;%Ts=1/Parametros.Fs;
% ## Resolucion por partes
%w=[wB; wA];
try
    close (1)
end



% Calculo de velocidades y dtita
Vel=zeros(3,N);
Pos=zeros(3,N);
Pos_lin=zeros(2,N);

% Condiciones iniciales
wA0=300;dW=0;
wB0=300;
tita=-1;
tita=tita*pi/180; Pos(3,1)=tita; Pos(3,2)=tita; % Condicion inicial
w=[wB0; wA0];w_v=[wB0 wB0; wA0 wA0];
kp=100;ki=0;kd=0;
d=1;bb=1;
for i=1:N-1
    
    %  ### Actualizo el valor de A
    A=[r*cos(Pos(3,i))/2 r*cos(Pos(3,i))/2;...
    r*sin(Pos(3,i))/2 r*sin(Pos(3,i))/2;...
    r/L -r/L ];
    % ### Calculo de las velocidades
    Vel(:,i)=A*w;%(:,i);   
    % ### Calculo de las posiciones FUTURAS
    Pos(:,i+1)=Vel(:,i)*Ts+Pos(:,i); 
    % ### Acoto el angulo, para que no diverja
 %  dd=fix(Pos(3,i+1)/(2*pi));   
 %  Pos(3,i+1)=Pos(3,i+1)-2*pi*dd;
   
   % ### Obtengo medicion del sensor
   x0=Pos(1,i);
   ind=find(lin(1,:)>x0,1);
   y0=Pos(2,i)-real(lin(2,ind)); tit0=Pos(3,i);
   % No vale para todos los casos
   if y0*tit0>0 % tienen el mismo signo
       m=(T*sin(tit0)+y0)*1/cos(tit0);
   else
       if y0<0; bb=-1;else; bb=1; end 
       m=bb*(y0^2-T^2+((T-y0*sin(tit0))*(1/cos(tit0)))^2)^0.5;
   end
   if y0*tit0>0 && d<0 || y0*tit0<0 && d>0
       d=d*-1;
       disp('cambio de algoritmo '); d
   end
   %m=(y0+T/sin(tit0))*(1/cos(tit0)); % hacer la trigonometria para entenderlo
   
    % Paso de m a beta
   beta=atan(m/T);
   
   % ### Control
    dW=-kp*beta+ki*dW+kd*(-(w_v(1,i)-w_v(2,i))+(w_v(1,i+1)-w_v(2,i+1)))/Ts; % Accion de control
   % ### Antiwindup
    if (dW>300 ); dW=300;  end
    if dW<-300 ;  dW=-300;  end
    ddW(i)=dW;
    w_v=[w_v w];
    w=[wB0+dW/2;wA0-dW/2];
    
    
    
    % ### Grafico posicion
    %subplot(211)
   
    plot(Pos(1,1:i-1),Pos(2,1:i-1),'.k',Pos(1,i),Pos(2,i),'rx',lin(1,:),lin(2,:),'-b');hold on;
    % ### Grafico orientacion
    Ox=0.1*cos(Pos(3,i)) ;Oy=0.1*sin(Pos(3,i));
    quiver(Pos(1,i),Pos(2,i),Ox,Oy);hold off;
    offy=0.5;offx=5;
    ylim([Pos(2,i)-offy Pos(2,i)+offy]); xlim([Pos(1,i)-offx*4/5 Pos(1,i)+offx/5]);
    ylabel('[m]');xlabel('[m]');
    title ('Posicion espacial mas orientacion')
%     subplot(212)
%     plot(ddW(1:i),'.')
    %Ang=[Pos(3,i) Vel(3,i)]*180/pi
    posi=[m Pos(2,i) dW Pos(3,i)*180/pi i]
    m2(i)=m;
    pause(0.05)
  
end