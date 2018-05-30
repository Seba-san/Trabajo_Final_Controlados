function [Kp,Ki,Kd]=ControlZN(control,metodo,entrada,salida,tiempo,tangente,figuras)
% Esta funci�n calcula el controlador indicado en control (P, PI o PID)
% utilizando el m�todo de Ziegler Nichols (metodo='ZN') o el de Cohen Coon
% (metodo='CC'). Para ello se usan los datos de ensayo indicados por
% entrada, salida y tiempo. El m�todo de ajuste con Ziegler Nichols sigue
% lo indicado en el Ogata (p�g 569). Tambi�n aport� algo de claridad
% https://sites.google.com/site/picuino/ziegler-nichols. Para el m�todos
% de Cohen Coon falta encontrar mejor bibiliograf�a; por ahora est�n las
% constantes sacadas de una p�gina web random
% (http://blog.opticontrols.com/archives/383). 

% La variable figuras indica si se desean ver los gr�ficos internos o no.
% La variable tangente me indica c�mo estimar la recta tangente para sacar
% los par�metros que requiere Z-N; si tangente=1, busco la recta tangente
% en el punto de inflecci�n de la curva, sino uso la recta que pasa por los
% puntos de 10 y 90% del aumento de la se�al.

% Observaci�n: como la salida oscila, para el valor en estado estacionario
% tomo un promedio del �ltimo 10% de las muestras.

if nargin<5
    figuras=0;
end
if tangente==1
    [K,L,T]=ParametrosDelEnsayo(entrada,salida,tiempo,figuras);    
elseif tangente==0
    [K,L,T,tao]=ParametrosDelEnsayo2(entrada,salida,tiempo,figuras);
elseif tangente==2
     [K,L,T]=ParametrosDelEnsayo3(entrada,salida,tiempo,figuras);   
end
% Muestro el ajuste del sistema
    if figuras
        figure
        estimadoZN=tf(K,[T 1],'InputDelay',L);
        lsim(estimadoZN,entrada,tiempo)
        hold on 
        plot(tiempo,entrada,tiempo,salida,'.');
        hold off
    end


%Calculo los par�metros seg�n el m�todo elegido:
switch metodo
    case 'ZN'
    Parametros=ConstantesZN(K,L,T);
    case 'CC2'
     Parametros=ConstantesCC2(K,L,T);
     case 'CC'
     Parametros=ConstantesCC(K,L,tao);
end
% if metodo=='ZN'
%     Parametros=ConstantesZN(K,L,T);
% elseif metodo=='CC'
%     Parametros=ConstantesCC(K,L,tao);
% elseif metodo=='CC2'    
%     Parametros=ConstantesCC2(K,L,T);
% else  
%     disp('Error')
% end

switch control
    case 'P'
        indice=1;
    case 'PI'
        indice=2;
    case 'PID'
        indice=3;
end
Kp=Parametros.Kp(indice);
Ki=Parametros.Ki(indice);
Kd=Parametros.Kd(indice);
end

function [w_interp,pwm_interp,t]=interpLineal(entrada,salida,tiempo,figuras)
% Aparentemente hay un problema con los datos, en lo que sigue voy a
% utilizar una interpolacion LINEAL entre los puntos.
Fs=2e3;
Ts=1/Fs;
t=min(tiempo):1/Fs:max(tiempo);

w_interp = interp1(tiempo,salida,t); % Interpola lo datos de salida
% Interpola los datos de entrada, para evitar que los haga con una
% pendiente, los fabrico a mano.
inicia=find(entrada==max(entrada),1); 
% Busca cuando empezo el pulso
inicia_seg=find(abs(tiempo(inicia)-t)==min(abs(tiempo(inicia)-t)));

pwm_interp=zeros(1,length(t));
pwm_interp(inicia_seg:length(t))=max(entrada);pwm_interp(1:(inicia_seg-1))=min(entrada);
if figuras
figure
plot(tiempo,salida,'o',t,w_interp,':.');
title('Interpolacion Lineal de w salida');
figure
plot(tiempo,entrada,'o',t,pwm_interp,':.');
title('Interpolacion Lineal de pwm entrada');
end
end

function [K,L,T]=ParametrosDelEnsayo(entrada,salida,tiempo,figuras)
% Esta funci�n calcula los par�metros de la respuesta al escal�n necesarios
% para utilizar el m�todo de Zieger-Nichols.

%Uso interpolaci�n lineal para que me tire m�s puntos:
[w_interp,pwm_interp,t]=interpLineal(entrada,salida,tiempo,figuras);

dt=diff(t); dw=diff(w_interp); ddw=diff(dw);
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

[m, o]=polyfit(t(indice_set),w_interp(indice_set),1); % Obtengo la ecuacion de la recta

Puntos_r=m(1)*t+m(2);

n=length(salida);
yinf=mean(salida(round(n*0.9):n));%Como la salida oscila, estimo el valor en estado 
%estacionario como el promedio del �ltimo 10% de las muestras.
y0=min(salida);
uinf=max(entrada);u0=min(entrada);
if u0==uinf %Le agrego esto para que tome que el escal�n arranc� en 0 si la
    u0=0;   %se�al de entrada no tiene muestras antes de pegar el salto.
end
K=(yinf-y0)/(uinf-u0);
%t2 es el tiempo en el cual la recta alcanza el valor yinf y t1 el tiempo
%en el que alcanza el valor y0. Despejando de la ecuaci�n de la recta:
t2=(yinf-m(2))/m(1);
t1=(y0-m(2))/m(1);
%t0 es el valor en el que empieza el escal�n:
[~,indice_t0]=max(entrada);%Busco cu�ndo la entrada pega el salto
t0=tiempo(indice_t0);
if figuras==1
    figure();plot(t,w_interp,'b.',tiempo,salida,'m.',t,Puntos_r,'g');
    hold on ;plot(t(indices),w_interp(indices),'ro');ylim([y0-yinf*0.1,yinf*1.1])
    plot(t2,m(1)*t2+m(2),'rx',t1,m(1)*t1+m(2),'rx');%,tiempo(indice_set),salida(indice_set),'kx');
    plot(tiempo,yinf*ones(1,length(tiempo)),'y--');%Esto lo pongo como referencia del valor m�ximo.
    plot(tiempo,y0*ones(1,length(tiempo)),'y--');%Esto lo pongo como referencia del valor m�nimo.
    title('Ensayo al Escal�n con Par�metros Marcados')
    legend('Respuesta al escal�n interpolada','Respuesta al escal�n real',...
        'Recta Tangente','Puntos para el c�lculo de la recta tangente',...
        'Intersecci�n entre la recta y el valor m�ximo','Intersecci�n entre la recta y el valor minimo');
end
L=t1-t0;T=t2-t1;
end

function [K,L,T,tao]=ParametrosDelEnsayo2(entrada,salida,tiempo,figuras)
% Esta funci�n calcula los par�metros de la respuesta al escal�n necesarios
% para utilizar el m�todo de Zieger-Nichols. Para ello estima la recta
% tangente como la recta que une los puntos de 10% y de 90% de se�al.
% Adem�s, calcula el valor de la constante de tiempo para poder usar el
% m�todo de Cohen-Coon.

n=length(salida);
yinf=mean(salida(round(n*0.7):n));%Como la salida oscila, estimo el valor en estado 
%estacionario como el promedio del �ltimo 30% de las muestras.
y0=min(salida);
uinf=max(entrada);u0=min(entrada);
if u0==uinf %Le agrego esto para que tome que el escal�n arranc� en 0 si la
    u0=0;   %se�al de entrada no tiene muestras antes de pegar el salto.
end
K=(yinf-y0)/(uinf-u0);

%Uso interpolaci�n lineal para que me tire m�s puntos:
[w_interp,pwm_interp,t]=interpLineal(entrada,salida,tiempo,figuras);

%Calculo los valores que representan un 10% y un 90% del incremento total:
diez=y0+(yinf-y0)*0.1;
noventa=y0+(yinf-y0)*0.9;
%Busco los valores interpolados m�s cercano a estos n�meros:
[~,indices(1)]=min(abs(w_interp-diez));
[~,indices(2)]=min(abs(w_interp-noventa));

[m, o]=polyfit(t(indices),w_interp(indices),1); % Obtengo la ecuacion de la recta

Puntos_r=m(1)*t+m(2);


%t2 es el tiempo en el cual la recta alcanza el valor yinf y t1 el tiempo
%en el que alcanza el valor y0. Despejando de la ecuaci�n de la recta:
t2=(yinf-m(2))/m(1);
t1=(y0-m(2))/m(1);
%t0 es el valor en el que empieza el escal�n:
[~,indice_t0]=max(entrada);%Busco cu�ndo la entrada pega el salto
t0=tiempo(indice_t0);

L=t1-t0;T=t2-t1;

%Calculo la constante de tiempo:
y_en_tao=y0+(yinf-y0)*(1-exp(-1));%Valor de la salida en t0+tao
[~,ind]=min(abs(w_interp-y_en_tao));%Busco el valor m�s pr�ximo
tao=t(ind)-t0-L;%tomo tao como el valor de tiempo al que se daba el valor de 
                %salida m�s pr�ximo al deseado menos t0+L (por el delay de
                %la planta).
           
if figuras==1
    figure();plot(t,w_interp,'b.',tiempo,salida,'m.',t,Puntos_r,'g');
    hold on ;plot(t(indices),w_interp(indices),'ro');ylim([y0-yinf*0.1,yinf*1.1])
    plot(t2,m(1)*t2+m(2),'rx',t1,m(1)*t1+m(2),'rx');%,tiempo(indice_set),salida(indice_set),'kx');
    plot(tiempo,yinf*ones(1,length(tiempo)),'y--');%Esto lo pongo como referencia del valor m�ximo.
    plot(tiempo,y0*ones(1,length(tiempo)),'y--');%Esto lo pongo como referencia del valor m�nimo.
    title('Ensayo al Escal�n con Par�metros Marcados')
    legend('Respuesta al escal�n interpolada','Respuesta al escal�n real',...
        'Recta Tangente','Puntos para el c�lculo de la recta tangente',...
        'Intersecci�n entre la recta y el valor m�ximo','Intersecci�n entre la recta y el valor minimo');
end
end

function Parametros=ConstantesZN(K,L,T)
%Calculo la tabla de coeficientes para cada controlador seg�n
%https://sites.google.com/site/picuino/ziegler-nichols (es casi igual al
%Ogata, cambia en que el Ogata pone 0.5 donde la p�gina pone 0.6).
%Kp=(1/K)*T/L*[1;0.9;1.2];
%Ki=(1/K)*T/L^2*[0;0.27;0.6];
%Kd=(1/K)*T*[0;0;0.6];
% Propongo un cambio de notacion (seba): Para revisar corregir problemas
Kp=(1/K)*T/L*[1;0.9;1.2];
Ki=Kp.*[0;(L/0.3)^-1;(2*L)^-1];
Kd=Kp.*[0;0;0.5*L];
Parametros=table(Kp,Ki,Kd,'RowName',{'P';'PI';'PID'});
end

function Parametros=ConstantesCC(gp,td,tao)
%Calculo la tabla de coeficientes para cada controlador seg�n
%http://blog.opticontrols.com/archives/383
Kc=(1/gp)*[1.03;0.9;1.35].*(tao/td+[.34;.092;.185]);
Kp=Kc;
Ki=Kc.*[0;(.3/td)*(tao+2.22*td)/(tao+.092*td);(.4/td)*(tao+0.611*td)/(tao+0.185*td)];
Kd=Kc.*[0;0;.37*td*tao/(tao+0.185*td)];
Parametros=table(Kp,Ki,Kd,'RowName',{'P';'PI';'PID'});

%Obs: Da pseudo similar a lo que daba antes, pero cuando simulo con el
%sistema estimado me da inestable con el PID y con el P :/
end


function Parametros=ConstantesCC2(K,L,T)
%Calculo la tabla de coeficientes para cada controlador seg�n el dpf que
%nos dio daniel
%Teniendo en cuenta que Gc=kp(1+1/(Ti*S)+Td*s)
% Relaciones Segun ogata kp=kp ki=kp/Ti kd=kp*Td ; tambien aparecen en la
% formula 3.20 del pdf de Daniel
Kp=T/(K*L)*[1+3*L/T;0.9+3*L/(12*T);1.35+L/(4*T)];
Ki=Kp.*[0;(((30+3*(L/T))/(9+20*(L/T)))*L)^-1;(((32+6*(L/T))/(13+8*(L/T)))*L)^-1];
Kd=Kp.*[0;0;4*L/(11+2*(L/T))];
Parametros=table(Kp,Ki,Kd,'RowName',{'P';'PI';'PID'});
end
function [K,L,T]=ParametrosDelEnsayo3(entrada,salida,tiempo,figuras)
% Señal en el tiempo a fitear
PmR='K*(1-exp(-(x-L)/T))';%P mas Retardo (no me acuerdo que es la P)
[~,indice]=max(diff(entrada));
tiempo2=tiempo-tiempo(indice);
PuntosIniciales=[max(entrada) 1/200 0.1];
f1 = fit(tiempo2(indice:end)',salida(indice:end)',PmR);%,'Start',PuntosIniciales);
K=f1.K/max(entrada);
L=f1.L;T=f1.T;
% if figuras
%    plot(f1,tiempo,salida)
%    title('Respuesta temporal curva fiteada')
% end


end