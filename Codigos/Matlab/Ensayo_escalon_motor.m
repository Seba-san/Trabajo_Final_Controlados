%% Inicializacion de serial
clear all;clc;close all;
cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
InfoHard=instrhwinfo('serial');%Busco cu�l puerto tengo conectado con esta instrucci�n
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
s=InicializacionSerial(InfoHard.SerialPorts{1},115200);%Velocidad: 115200 baudios
%%
cd /media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba
%cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Codigos\Matlab')%compu Tania

addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab');
s=InicializacionSerial('/dev/ttyUSB1',1000000);%Velocidad: 115200 baudios
%% Cerrar puerto
fclose(instrfindall);       %cierra todos los puertos activos y ocultos
disp('puerto cerrado')
%% Ensayo al escalon

N=300;%Cantidad de muestras a tomar
wA=[];
tiempo=[];
%Env_instruccion(s,'online');%Le indico al nano que se ponga a escupir datos
PWMA=[];  
Comunic_test(s)
PWM=10;
%Env_instruccion(s,'PWM',[PWM PWM]);%Que arranque en 10% el PWM para que no tire error por no entrar a la interrupci�n po flanco
Env_instruccion(s,'Ucontrol',PWM);
pause(3);
Comunic_test(s)
Env_instruccion(s,'online');
flushinput(s);  %Vacia el buffer de entrada
for k=1:N %Ojo con el tope que pone el nano
    if k==10
        PWM=40;
        Env_instruccion(s,'Ucontrol',PWM);
    end
    dato1=str2double(fscanf(s));%Actualmente el nano est� enviando la vel en rpm y el tiempo en us.
    dato2=str2double(fscanf(s));
       %flushinput(s);
    wA=[wA dato1];
    tiempo=[tiempo dato2];
    PWMA=[PWMA PWM];
end
Env_instruccion(s,'Ucontrol',0);
Env_instruccion(s,'stop');%Le indico al nano que deje de transmitir datos
name=datestr(now,'yymmddhhMMss');
name=strcat('respuesta_escalon_motorA_',name,'.mat');
%save(name,'tiempo','PWMA','wA') 
figure(1);plot((tiempo-tiempo(1))*1e-6,PWMA,(tiempo-tiempo(1))*1e-6,wA,'.');%ylim([0 1500])
%legend('Se�al de PWM','Se�al de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo (s)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
figure(2);plot(diff(tiempo-tiempo(1))*1e-6,'.');ylabel('tiempo entre muestras en segundos')%ylim([0 10e-3])
%% Gr�fico de la Respuesta del motor B
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')

load('../../Mediciones/180423182356_resp_escalon.mat')
figure(1);
plot(tiempo,PWMA,tiempo,wA,'.');
%legend('Se�al de PWM','Se�al de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo (us)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $

%% Acomodando Ts para estimacion del sistema con INTERPOLACION LINEAL
% Aparentemente hay un problema con los datos, en lo que sigue voy a
% utilizar una interpolacion LINEAL entre los puntos.
entrada=PWMA;%PWMA;
salida=wA;%wA
tiempo_=tiempo;
Fs=2e3;
Ts=1/Fs;
tiempoSeg=tiempo_*1e-6;
t=min(tiempoSeg):1/Fs:max(tiempoSeg);

w_interp = interp1(tiempoSeg,salida,t); % Interpola lo datos de salida
% Interpola los datos de entrada, para evitar que los haga con una
% pendiente, los fabrico a mano.
inicia=find(entrada==max(entrada),1); 
% Busca cuando empezo el pulso
inicia_seg=find(abs(tiempo_(inicia)*1e-6 -t)==min(abs(tiempo_(inicia)*1e-6 -t)));

pwm_interp=zeros(1,length(t));
pwm_interp(inicia_seg:length(t))=max(entrada);pwm_interp(1:(inicia_seg-1))=min(entrada);
%pwm_interp=interp1(tiempoSeg,entrada,t);
figure
plot(tiempoSeg,salida,'o',t,w_interp,':.');
title('Interpolacion Lineal de w_salida');
figure
plot(tiempoSeg,entrada,'o',t,pwm_interp,':.');
title('Interpolacion Lineal de pwm_entrada');

%% Realizo el calculo de las constantes del sistema y acomodo los tiempos

entrada=pwm_interp;%PWMA;
salida=w_interp;%wA;
tiempo_=t;
% Revistar esto, porque la interpolacion genera error en el calculo de t0.
%dPWMA=diff(entrada);[~,indice_set]=max(dPWMA);t0=tiempo_(indice_set+1); % Obtengo el tiempo en el que inicia el pulso
t0=tiempo_(inicia_seg);
time=(tiempo_-t0);%*1e-6; % Acondiciono el tiempo para en ensayo escalon.
indices_=find(time>=0);time2=time(indices_);
wm=salida-min(salida);wm2=wm(indices_);
S = stepinfo(wm2,time2) %Extraigo la informacion de la respuesta escalon
PWMA2=entrada(indices_);
figure(1)
plot(time2,wm2,'b.',time2,PWMA2,'r.')
title('Respuesta escalon recortada al inicio del escalon ')

%%
%Acomodo los tiempos
index=find(time<2*S.SettlingTime & time>-S.SettlingTime);
w_data=wm(index);
pwm_data=entrada(index);pwm_data=pwm_data-min(pwm_data);
t_data=time(index);
plot(t_data,w_data,'b.',t_data,pwm_data)
disp('Respuesta escalon recortada desde 1 settlingtime antes del escalon y 2 Settlingtime despues ')

%% Acomodando Ts para estimacion del sistema con ZOH
% Como la estimacion de Matlab se hace para una senal muestreada a Fs cte,
% generamos una version que guarde el valor anterior mas cercano para cada
% valor del vector de tiempos constante.
entrada=PWMA;%PWMA;
salida=wA;%wA
Fs=2e3;
Ts=1/Fs;
tiempoSeg=tiempo*1e-6;
t_zoh=min(tiempoSeg):1/Fs:max(tiempoSeg);
for k=1:length(t_zoh)
    [~,indice]=min(abs(tiempoSeg-t_zoh(k)));%Busco la muestra mas cercana al valor de tiempo en el que estoy mirando
    if tiempoSeg(indice)-t_zoh(k)>0 && indice>1
        indice=indice-1;%Me aseguro que siempre se quede con el valor mas cercano pero anterior, no futuro
    end
    w(k)=salida(indice);%Guardo el valor de wA anterior mas proximo al valor de tiempo que estoy considerando
    PWM(k)=entrada(indice);
end
figure(1);plot(t_zoh,w,'r.',tiempoSeg,wA,'b.');
%figure(1);plot(t,PWM,'.',tiempoSeg,PWMA);

%% Estimacion Sist
% transformo la informacion al formato iidata:
% data = iddata(w',PWM',Ts);

%data = iddata((w-w(1))',(PWM-PWM(1))',Ts);%Le corro el eje y para que arranque en 0.
data = iddata(w_interp',pwm_interp',Ts);
%Nota: no funcion� :( . Para 2 polos 1 cero parece que da exactamente el
%mismo sistema que la versi�n no desplazada.

% data = iddata(y,u,Ts) creates an iddata object containing a time-domain
% output signal y and input signal u, respectively. Ts specifies the sample
% time of the experimental data.
% Obs: tienen que ser vectores columna!! Sino lo toma como 5millones de
% entradas/salidas.
np=3;%Le indico el nro de polos
nz=[2];%Le indico el nro de ceros
iodelay=[];%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts)
%%
% version continua
data = iddata(w_data',pwm_data');
sys = tfest(data,np,nz)
step(sys);hold on; plot(t_data,w_data,'.');hold off
sysd=c2d(sys,Ts);
%% Prueba del sist estimado
we=filter(sys.Numerator,sys.Denominator,pwm_interp);
%we=filter(sysd.Numerator,sysd.Denominator,pwm_data);
figure(1);plot(t,we,t,w_interp,'.');
%figure(1);plot(time,we,t_data,w_data,'.');
%figure(1);plot(t_data,we,'.');
%% PID

% Hay varios metodos de ajuste; en este caso se va a implementar
% En este caso se aplica esta: http://csd.newcastle.edu.au/SpanishPages/clase_slides_download/C07.pdf
% Tambien se puede ver en el Ogata, pagina 569; Ver capitulo 8 (muy bueno,
% tiene codigos de matlab e ideas) pagina 567 :P
%calculo de la pendiente
% entrada=pwm_data;
% salida=w_data;
% tiempo=t_data;
entrada=pwm_interp;%pwm_interp_2;
salida=w_interp;%w_interp_2;
tiempo=time;
try
close(1);close(2)
end
dt=diff(tiempo); dw=diff(salida); ddw=diff(dw);
N=6;
indices=zeros(1,N); % Se fija los N puntos que tienen un diferencial mas grande y esos los usa para estimar la recta
for i=1:N   
[val,indices(i)]=max(dw);
dw(indices(i))=0;
end
im=min(indices);imx=max(indices);
% Busco el cambio de concabidad:
[mx_ddw, ind_mx]=max(ddw);[min_ddw,ind_min]=min(ddw);
indice_set=round((ind_mx+ind_min)/2);indice_set=[indice_set indice_set-1];

%[m, o]=polyfit(t(im:imx),w(im:imx),1);
%[m, o]=polyfit(tiempo(indices),salida(indices),1); % Obtengo la ecuacion de la recta
[m, o]=polyfit(tiempo(indice_set),salida(indice_set),1); % Obtengo la ecuacion de la recta

%m=(max(w(indices))-min(w(indices)))/(max(t(indices))-min(t(indices)))

% Recta dada pendiente y punto: (y-y1)=m*(x-x1) -> y=m(x-x1) +y1
%Puntos_r=m(1)*(t(indices(1)-100:indices(1)+100)-t(indices(1)))+w(indices(1));
Puntos_r=m(1)*tiempo+m(2); %Puntos_r=polyval(m,t,0);
figure(1);plot(tiempo,salida,'b.',tiempo(indices),salida(indices),'ro');hold on ; plot(tiempo((im-50):(imx+50)),Puntos_r((im-50):(imx+50)),'g');ylim([0 900]);hold off

yinf=max(salida);y0=min(salida);
uinf=max(entrada);u0=min(entrada);
k0=(yinf-y0)/(uinf-u0);
[~,indice_t2]=min(abs(Puntos_r-yinf));t2=(tiempo(indice_t2));
[~,indice_t1]=min(abs(Puntos_r-y0));t1=(tiempo(indice_t1));
t0=0; % Si los datos estan acomodados, t0 tiene que ser 0.
figure(2);plot(tiempo,salida,'b.',tiempo(indice_t2),Puntos_r(indice_t2),'ro',tiempo(indice_t1),Puntos_r(indice_t1),'go',tiempo(indice_set(1)),salida(indice_set(1)),'go');
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
% Las formas del PID consideradas son:
% Kp(s)=Kp
%Kpi(s)=Kp(1+1/s*Tr)
%Kpd(s)=Kp(1+Td*s/(taod*s+1))
%Kpid(s)=Kp(1+1/(Tr*s)+Td*s/(taod*s+1))

% Segun OGATA

Parametros_Ogata=zeros(3,3);
Parametros_Ogata(ind_Kp,ind_P)=(t2-t1)/(t1-t0); % T/L
Parametros_Ogata(ind_Kp,ind_PI)=0.9*Parametros_Ogata(ind_Kp,ind_P);Parametros_Ogata(ind_Tr,ind_PI)=(t1-t0)/0.3;
Parametros_Ogata(ind_Kp,ind_PID)=1.2*Parametros_Ogata(ind_Kp,ind_P);Parametros_Ogata(ind_Tr,ind_PID)=2*(t1-t0);Parametros_Ogata(ind_Td,ind_PID)=0.5*(t1-t0);
Parametros_Ogata'
%
%Z-N considera:
% Gc(s)=Kp(1+1/(Ti*S)+Td*S)
% Para crear y probar el controlador con esos parametros hay que ejecutar la
% siguiente instruccion:

a=ind_P;P=Parametros_Ogata;PIDF=0; Ts=0.005;%0.015;%data.Ts; % en 1 es si, en 0 es no
C=pid(P(1,a),P(2,a),P(3,a),PIDF,'Ts',Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds')
%C=C/k0;
% C=pid(P(1,a),P(2,a),P(3,a));
%No tiene sentido hacer lo que sigue, porque el systema no queda bien
%estimado
% T_pi = feedback(C*sys_2, 1);
% figure (2);
% step(T_pi)
% title ('Controlado')
% figure (3);
% step(sys_2)
% title ('Sin controlar')
%%
% Tunea automaticamente, segun las caracteristicas programadas
%sys2=d2d(sys,0.015)
%C = pidtune(sys,'pid')
PIDF=0; % en 1 es si, en 0 es no
sistema=sys;%sys_2;
C0 = pid(1,1,1,PIDF,'Ts',sistema.Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
%C = pidtune(sys,C0);
opt = pidtuneOptions('DesignFocus','reference-tracking','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sistema,C0,opt);
%[C,info] = pidtune(sys,C0);
T_pi = feedback(C*sistema, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sistema)
title ('Sin controlar')
%%
% 
% ########## PID estandar; lo paso a forma de cociente de polinomios


% ### Lo paso a la forma apropiada para programarlo
mm=C;
% Primero, Segundo y Tercero son los distintos terminos del PID. Este
% algoritmo lo que hace es pasarlo de la forma "bibliografica" a la forma
% de cociente de polinomios.
% la forma final propuesta resulta:
% Y(k)=e(k)*A+e(k-1)*B+e(k-2)*C+y(k-1)*D+y(k-2)*E
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
CUCU=[num -den(2:size(den,2))]; % Cual es el orden de los parametros?
sprintf('%f, %f, %f, %f, %f',CUCU)
%tfsys=tf(sys.Numerator,sys.Denominator,sys.Ts)
% Transformo el sistema discreto a continuo



%%



