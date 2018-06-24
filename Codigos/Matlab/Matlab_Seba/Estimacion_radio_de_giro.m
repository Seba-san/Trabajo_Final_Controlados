%%
% Considero una circunferencia centrada en el eje Y, tal que el eje X sea
% tangente. De aqui resulta que si: (x-a)^2+(y-b)^2=r^2, entonces a=0; lo
% cual elimina una incognita, permitiendo de esta manera que con 2 puntos
% se pueda estimar el radio de giro.
% .--- Obtengo los puntos de una circunferencia
r=5;
y=-4:0.1:20;
A=r;
x=(r^2-(y-A).^2).^(1/2);
plot(x,y,'.');
P1=[x(50) y(50)];P2=[x(55) y(55)];hold on;
plot(P1(1),P1(2),'bx');plot(P2(1),P2(2),'rx');hold off
%P1=[8.6603 4]; P2=[9.7980 5];
%%
% A partir de los puntos anteriores, hago el proceso inverso para obtener
% la circunferencia. Hay que considerar que la presicion del metodo depende
% de que tan bien se estimen los puntos.
T=-[P1(1)^2+P1(2)^2 P2(1)^2+P2(2)^2];

alfa=(T(1)-T(2))/(P1(2)-P2(2));
beta=T(1)-alfa*P1(2);
A=-alfa/2;
R=(-beta+(alfa^2)/4)^(1/2)

y=-4:0.1:20;
x=(R^2-(y-A).^(2));x=x.^(1/2);

plot(P1(1),P1(2),'bx',P2(1),P2(2),'rx',real(x),y,'k.' );
%xlim([-10 10]);ylim([-10 10])


%%
% En funcion a los datos simuladores puedo generar puntos de la trayectoria
% para estimar el radio de giro.
for k=50:80
for i=1:80
k
P1=[Pos(1,i) Pos(2,i)]; P2=[Pos(1,k) Pos(2,k)];


% Obtengo todos los radios de giro calculados

T=-[P1(1)^2+P1(2)^2 P2(1)^2+P2(2)^2];

alfa=(T(1)-T(2))/(P1(2)-P2(2));
beta=T(1)-alfa*P1(2);
A=-alfa/2;
R(i)=(-beta+(alfa^2)/4)^(1/2);

y=-4:0.1:20;
x=(R(i)^2-(y-A).^(2));x=x.^(1/2);

%plot(P1(1),P1(2),'bx',P2(1),P2(2),'rx',real(x),y,'k.' );
end

%
%Radio de giro dados los parametros
wref=(wA+wB)/2;
dW=wB-wA;
Rg=L*(wref(1:80)./dW(1:80)-1/2);
plot(tiempo(1:80),Rg(1:80),'b.',tiempo(1:80),R(1:80),'r.');ylim([0 5])
pause(1)
end
