% Gr�fico de la Respuesta del motor B
% se cargan los valores del ensayo

load('../../Mediciones/180423182356_resp_escalon.mat')
imagenes=1;
if imagenes
figure(1);
plot(tiempo,PWMA,tiempo,wA,'.');
%legend('Se�al de PWM','Se�al de vel ang');
title('Respuesta del Motor B');
xlabel('tiempo (us)');ylabel('Vel Ang (rpm) / PWM') %Revisar la unidad!! $
end
%[pwm_interp,w_interp,t,S]=interpol(PWMA,wA,tiempo,imagenes);
entrada=PWMA;salida=wA;tiempo_=tiempo;


% Acomodando Ts para estimacion del sistema con INTERPOLACION LINEAL
% Aparentemente hay un problema con los datos, en lo que sigue voy a
% utilizar una interpolacion LINEAL entre los puntos.
 %[pwm_interp,w_interp,time,S]= interpol(entrada,salida,tiempo_,imagenes)
%entrada=PWMA;%PWMA;
%salida=wA;%wA
%tiempo_=tiempo;

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

t0=t(inicia_seg);
time=(t-t0);%*1e-6; % Acondiciono el tiempo para en ensayo escalon.
%indices_=find(time>=0);time2=time(indices_);
w_interp_2=w_interp-min(w_interp);%wm2=wm(indices_);
S = stepinfo(w_interp_2,time) %Extraigo la informacion de la respuesta escalon
%PWMA2=entrada(indices_);
%figure(1)
%plot(time2,wm2,'b.',time2,PWMA2,'r.')
%title('Respuesta escalon recortada al inicio del escalon ')
pwm_interp_2=pwm_interp-min(pwm_interp);
if imagenes
    figure(1)
    plot(tiempoSeg,salida,'o',t,w_interp,':.');
    title('Interpolacion Lineal de w_salida');
    figure(2)
    plot(tiempoSeg,entrada,'o',t,pwm_interp,':.');
    title('Interpolacion Lineal de pwm_entrada');
end

data = iddata(w_interp',pwm_interp',Ts);
np=3;%Le indico el nro de polos
nz=[2];%Le indico el nro de ceros
iodelay=[];%No se que ponerle
sys = tfest(data,np,nz,iodelay,'Ts',data.Ts)
sys_2=d2d(sys,0.015);


