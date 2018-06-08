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
load('/home/seba/Dropbox/Facultad/Trabajo_Final_Controlados_git/180607231105resp_escalon_sistema_total.mat')
%close all
L=0.16;%[m]
r=0.015;%[m]
T=0.05; %[m] Distancia del centro al sensor.
tita=0;
Ts=1/Parametros.Fs;
% ## Resolucion por partes
w=[wA; wB];
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
    Pos_lin(:,i)=[Pos(1,i)+T*cos(beta(i)+Pos(3,i));Pos(2,i)+T*sin(beta(i)+Pos(3,i))];
    
    
    
end
plot(Pos(1,:),Pos(2,:),'r.',Pos_lin(1,:),Pos_lin(2,:),'b.');%ylim([-0.3 0.3])
%hold on
%quiver(Pos(1,:),Pos(2,:),Vel(1,:),Vel(2,:))
%hold off
