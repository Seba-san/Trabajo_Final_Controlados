function [Kp,Ki,Kd]=ControlZN(control,entrada,salida,tiempo,figuras)
% Esta función calcula el controlador indicado en control (P, PI o PID)
% utilizando el método de Ziegler Nichols. Para ello se usan los datos de
% ensayo indicados por entrada, salida y tiempo. El método de ajuste sigue
% lo indicado en el Ogata (pág 569). También aportó algo de claridad
% https://sites.google.com/site/picuino/ziegler-nichols-
% La variable figuras indica si se desean ver los gráficos internos o no.

if nargin<5
    figuras=0;
end
[K,L,T]=ParametrosDelEnsayo(entrada,salida,tiempo,figuras);
Parametros=Constantes(K,L,T);
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

function [K,L,T]=ParametrosDelEnsayo(entrada,salida,tiempo,figuras)
% Esta función calcula los parámetros de la respuesta al escalón necesarios
% para utilizar el método de Zieger-Nichols.
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

[m, o]=polyfit(tiempo(indice_set),salida(indice_set),1); % Obtengo la ecuacion de la recta

Puntos_r=m(1)*tiempo+m(2);

yinf=max(salida);y0=min(salida);
uinf=max(entrada);u0=min(entrada);
if u0==uinf %Le agrego esto para que tome que el escalón arrancó en 0 si la
    u0=0;   %señal de entrada no tiene muestras antes de pegar el salto.
end
K=(yinf-y0)/(uinf-u0);
%t2 es el tiempo en el cual la recta alcanza el valor yinf y t1 el tiempo
%en el que alcanza el valor y0. Despejando de la ecuación de la recta:
t2=(yinf-m(2))/m(1);
t1=(y0-m(2))/m(1);
%t0 es el valor en el que empieza el escalón:
[~,indice_t0]=max(entrada);%Busco cuándo la entrada pega el salto
t0=tiempo(indice_t0);
if figuras==1
    figure();plot(tiempo,salida,'b.',tiempo,Puntos_r,'g');
    hold on ;plot(tiempo(indices),salida(indices),'ro');ylim([y0-yinf*0.1,yinf*1.1])
    plot(t2,m(1)*t2+m(2),'rx',t1,m(1)*t1+m(2),'rx');%,tiempo(indice_set),salida(indice_set),'kx');
    title('Ensayo al Escalón con Parámetros Marcados')
    legend('Respuesta al escalón','Recta Tangente','Puntos para el cálculo de la recta tangente',...
        'Intersección entre la recta y el valor máximo','Intersección entre la recta y el valor minimo');
end
L=t1-t0;T=t2-t1;
end

function Parametros=Constantes(K,L,T)
%Calculo la tabla de coeficientes para cada controlador según el Ogata
Kp=(1/K)*T/L*[1;0.9;1.2];
Ki=(1/K)*T/L^2*[0;0.27;0.6];
Kd=(1/K)*T*[0;0;0.6];
Parametros=table(Kp,Ki,Kd,'RowName',{'P';'PI';'PID'});
end