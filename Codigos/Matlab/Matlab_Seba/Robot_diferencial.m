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

%%
%load('/tmp/180607213207resp_escalon_sistema_total.mat')
%load('/home/seba/Dropbox/Facultad/Trabajo_Final_Controlados_git/180607231105resp_escalon_sistema_total.mat')
load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180622220141respuesta_escalon_systot_scontrolador_.mat')
% Este procesamiento es necesario para las ultimas mediciones
indice=find(control==0);betas2=betas(indice);wA2=wA(indice);wB2=wB(indice);tiempo2=tiempo(indice);
indice=find(betas2<3);betas2=betas2(indice);wA2=wA2(indice);wB2=wB2(indice);tiempo2=tiempo2(indice);
beta=betas2;wA=wA2;wB=wB2;

%close all
L=0.16;%[m]
%r= 0.01;%Valor de prueba
r= 0.0045;%Valor estimado con el system identification.
%r=0.015;%[m]
T=0.05; %[m] Distancia del centro al sensor.
tita=0;
Ts=1/200;%Ts=1/Parametros.Fs;
% ## Resolucion por partes
w=[wB; wA];
N=length(wA);

% Calculo de velocidades y dtita
Vel=zeros(3,N);
Pos=zeros(3,N);
Pos_lin=zeros(2,N);
Pos(3,1)=tita; % Condicion inicial
for i=2:N
    % Actualizo el valor de A
    A=[r*cos(Pos(3,i-1))/2 r*cos(Pos(3,i-1))/2;...
    r*sin(Pos(3,i-1))/2 r*sin(Pos(3,i-1))/2;...
    r/L -r/L ];
    % Calculo de las velocidades
    Vel(:,i)=A*w(:,i);   
    % Calculo de las posiciones
    Pos(:,i)=Vel(:,i)*Ts+Pos(:,i-1); 
      % BORRAR
    Pos(2,i)=-T*sin(beta(i)-Pos(3,i));
    Pos_lin(:,i)=[Pos(1,i)+T*cos(beta(i)-Pos(3,i));Pos(2,i)+T*sin(beta(i)-Pos(3,i))];
  
    
    
end
plot(Pos(1,:),Pos(2,:),'r.',Pos_lin(1,:),Pos_lin(2,:),'b.');%ylim([-0.3 0.3])
% hold on
% quiver(Pos(1,:),Pos(2,:),Vel(1,:),Vel(2,:))
% hold off

%%
% BORRAR

plot(tiempo2,beta,'b.',tiempo2,Pos(3,:),'r.')


