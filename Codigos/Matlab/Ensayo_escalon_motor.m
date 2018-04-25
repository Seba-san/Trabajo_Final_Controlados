%% Inicializacion de serial
clear all;clc;close all;
cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
%% Ensayo al escalon
N=200;%Cantidad de muestras a tomar
wA=[];
tiempo=[];
Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
PWMA=[];                   
PWM=10;
Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
pause(1);
flushinput(s);  %Vacia el buffer de entrada
for k=1:N %Ojo con el tope que pone el nano
    if k==16
        PWM=100;
        Env_instruccion(s,'PWM',[PWM PWM]);
    end
    dato=str2double(fscanf(s));%Actualmente el nano est� enviando la vel en rpm y el tiempo en us.
    wA=[wA dato];
     dato=str2double(fscanf(s));
    tiempo=[tiempo dato];
    PWMA=[PWMA PWM];
end
Env_instruccion(s,'PWM',[0 0]);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
name=strcat(name,'.mat');
save(name,'tiempo','PWMA','wA') 
%% Gr�fico de la Respuesta del motor B
 load('../../Mediciones/180423182356_resp_escalon.mat')
figure(1);
plot(tiempo,PWMA,tiempo,wA,'.');
%legend('Se�al de PWM','Se�al de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo (us)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
%% Acomodando Ts para estimacion del sistema
% Como la estimacion de Matlab se hace para una senal muestreada a Fs cte,
% generamos una version que guarde el valor anterior mas cercano para cada
% valor del vector de tiempos constante.
Fs=2e3;
Ts=1/Fs;
tiempoSeg=tiempo*1e-6;
t=min(tiempoSeg):1/Fs:max(tiempoSeg);
for k=1:length(t)
    [~,indice]=min(abs(tiempoSeg-t(k)));%Busco la muestra mas cercana al valor de tiempo en el que estoy mirando
    if tiempoSeg(indice)-t(k)>0 && indice>1
        indice=indice-1;%Me aseguro que siempre se quede con el valor mas cercano pero anterior, no futuro
    end
    w(k)=wA(indice);%Guardo el valor de wA anterior mas proximo al valor de tiempo que estoy considerando
    PWM(k)=PWMA(indice);
end
figure(1);plot(t,w,'r.',tiempoSeg,wA,'b.');
%figure(1);plot(t,PWM,'.',tiempoSeg,PWMA);
%% Estimacion Sist
% transformo la informacion al formato iidata:
% data = iddata(w',PWM',Ts);

data = iddata((w-w(1))',(PWM-PWM(1))',Ts);%Le corro el eje y para que arranque en 0.
%Nota: no funcion� :( . Para 2 polos 1 cero parece que da exactamente el
%mismo sistema que la versi�n no desplazada.

% data = iddata(y,u,Ts) creates an iddata object containing a time-domain
% output signal y and input signal u, respectively. Ts specifies the sample
% time of the experimental data.
% Obs: tienen que ser vectores columna!! Sino lo toma como 5millones de
% entradas/salidas.
np=1;%Le indico el nro de polos
nz=[];%Le indico el nro de ceros
iodelay=0;%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts);
%% Prueba del sist estimado
we=filter(sys.Numerator,sys.Denominator,PWM);
figure();plot(t,we,t,w,'.');
%% PID

% Hay varios metodos de ajuste; en este caso se va a implementar
% En este caso se aplica esta: http://csd.newcastle.edu.au/SpanishPages/clase_slides_download/C07.pdf
% Tambien se puede ver en el Ogata, pagina 569; Ver capitulo 8 (muy bueno,
% tiene codigos de matlab e ideas) pagina 567 :P
%calculo de la pendiente
try
close(1);close(2)
end
dt=diff(t); dw=diff(w); ddw=diff(dw);
N=6;
indices=zeros(1,N);
for i=1:N   
[val,indices(i)]=max(dw);
dw(indices(i))=0;
end
im=min(indices);imx=max(indices);
%[m, o]=polyfit(t(im:imx),w(im:imx),1);
[m, o]=polyfit(t(indices),w(indices),1);


%m=(max(w(indices))-min(w(indices)))/(max(t(indices))-min(t(indices)))

% Recta dada pendiente y punto: (y-y1)=m*(x-x1) -> y=m(x-x1) +y1
%Puntos_r=m(1)*(t(indices(1)-100:indices(1)+100)-t(indices(1)))+w(indices(1));
Puntos_r=m(1)*t+m(2); %Puntos_r=polyval(m,t,0);
%figure(1);plot(t,w,'b.',t(indices),w(indices),'ro');hold on ; plot(t((im-50):(imx+50)),Puntos_r((im-50):(imx+50)),'g');hold off

yinf=max(w);y0=min(w);
uinf=max(PWM);u0=min(PWM);
k0=(yinf-y0)/(uinf-u0);
[~,indice_t2]=min(abs(Puntos_r-yinf));t2=(t(indice_t2));
[~,indice_t1]=min(abs(Puntos_r-y0));t1=(t(indice_t1));
dPWM=diff(PWM);[~,indice_set]=max(dPWM);%plot(PWM,'.');hold on; plot(indice_set,PWM(indice_set),'or')
t0=t(indice_set);% plot(t,PWM);hold off
%figure(2);plot(t,w,'b.',t(indice_t2),Puntos_r(indice_t2),'ro',t(indice_t1),Puntos_r(indice_t1),'go',t(indice_set),w(indice_set),'go');
tao0=t1-t0;gama0=t2-t1;
k0 
tao0
gama0

%%
% Calculo de los coeficientes para los 3 casos:
Parametros=zeros(3,3); % Se considera el siguiente orden:
%   Kp, Tr, Td
% P 
% PI
% PID
% Segun la pagina en PDF
ind_Kp=1;ind_P=1;ind_PI=2;ind_PID=3;ind_Tr=2;ind_Td=3;
Parametros(ind_Kp,ind_P)=gama0/(k0*tao0);
Parametros(ind_Kp,ind_PI)=0.9*Parametros(ind_Kp,ind_P);Parametros(ind_Tr,ind_PI)=3*tao0;
Parametros(ind_Kp,ind_PID)=1.2*Parametros(ind_Kp,ind_P);Parametros(ind_Tr,ind_PID)=2*tao0;Parametros(ind_Td,ind_PID)=0.5*tao0;
Parametros'

% Segun OGATA

Parametros_Ogata=zeros(3,3);
Parametros_Ogata(ind_Kp,ind_P)=(t2-t1)/(t1-t0); % T/L
Parametros_Ogata(ind_Kp,ind_PI)=0.9*Parametros_Ogata(ind_Kp,ind_P);Parametros_Ogata(ind_Tr,ind_PI)=(t1-t0)/0.3;
Parametros_Ogata(ind_Kp,ind_PID)=1.2*Parametros_Ogata(ind_Kp,ind_P);Parametros_Ogata(ind_Tr,ind_PID)=2*(t1-t0);Parametros_Ogata(ind_Td,ind_PID)=0.5*(t1-t0);
Parametros_Ogata'
%
% Las formas del PID consideradas son:
% Kp(s)=Kp
%Kpi(s)=Kp(1+1/s*Tr)
%Kpd(s)=Kp(1+Td*s/(taod*s+1))
%Kpid(s)=Kp(1+1/(Tr*s)+Td*s/(taod*s+1))
% Para crear y probar el controlador con esos parametros hay que ejecutar la
% siguiente instruccion:
a=ind_PID;P=Parametros_Ogata;PIDF=1; % en 1 es si, en 0 es no
C_pdf=pid(P(1,a),P(2,a),P(3,a),PIDF,'Ts',data.Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
T_pi = feedback(C_pdf*sys, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys)
title ('Sin controlar')
%%
% Tunea automaticamente, segun las caracteristicas programadas
%C = pidtune(sys,'pid')
PIDF=1; % en 1 es si, en 0 es no
C0 = pid(1,1,1,PIDF,'Ts',data.Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
%C = pidtune(sys,C0);
opt = pidtuneOptions('DesignFocus','reference-tracking','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys,C0,opt);
%[C,info] = pidtune(sys,C0);
T_pi = feedback(C*sys, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys)
title ('Sin controlar')
%%
% 
% ########## PID estandar; lo paso a forma de cociente de polinomios


% ### Lo paso a la forma apropiada para programarlo
mm=C;
Primero=tf(mm.Kp,1);
Segundo=tf([mm.Ts*mm.Ki 0],[1 -1],mm.Ts);
if (PIDF==1)
Tercero=tf(mm.kd*[1 -1],mm.Tf*[1 -1]+mm.Ts*[1 0],mm.Ts);
else
Tercero=tf(mm.kd*[1 -1],mm.Ts*[1 0],mm.Ts); 
end
Pid=Primero+Segundo+Tercero;pipi=Pid; % Supongo que esta en la forma PID paralelo
num=pipi.Numerator{1};
den=pipi.Denominator{1};
num=num/den(1);den=den/den(1); % hay que dividir por den(1) porque es el coeficiente de u(k).
disp('A B C D E')
CUCU=[num -den(2:size(den,2))];
sprintf('%f, %f, %f, %f, %f',CUCU)
%tfsys=tf(sys.Numerator,sys.Denominator,sys.Ts)
% Transformo el sistema discreto a continuo


