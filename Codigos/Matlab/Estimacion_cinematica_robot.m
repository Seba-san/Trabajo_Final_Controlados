% Este script estima el sistema de la dinámica del robot a partir de
% ensayos.
%% Cargo los ensayos
clear all;close all;clc
%Cargo el primer ensayo y guardo los datos con otro nombre:
load('../../Mediciones/180603192250resp_escalon_sistema_total');
tiempo{1}=t;angulo{1}=beta;velA{1}=wA;velB{1}=wB;
%Cargo el segundo ensayo y guardo los datos con otro nombre:
load('../../Mediciones/180603192511resp_escalon_sistema_total');
tiempo{2}=t;angulo{2}=beta;velA{2}=wA;velB{2}=wB;
%% Gráfico de los Ensayos
figure(1);subplot(211);title('Ensayo 1')
yyaxis right;plot(tiempo{1},angulo{1},'.');
yyaxis left;plot(tiempo{1},velA{1},tiempo{1},velB{1});grid on
subplot(212);;title('Ensayo 2')
yyaxis right;plot(tiempo{2},angulo{2},'.');
yyaxis left;plot(tiempo{2},velA{2},tiempo{2},velB{2});grid on
%% Gráfico de los Ensayos 2
figure(1);subplot(211);title('Ensayo 1')
yyaxis right;plot(tiempo{1},angulo{1},'.');
yyaxis left;plot(tiempo{1},-velA{1}+velB{1});grid on
subplot(212);;title('Ensayo 2')
yyaxis right;plot(tiempo{2},angulo{2},'.');
yyaxis left;plot(tiempo{2},-velA{2}+velB{2});grid on
%% Quito los valores donde perdió la línea
n0=50;%Para que tome desde el escalón
for k=1:2
    indice=find(angulo{k}==3);%Cuando pierde la línea el sensor queda en 0
    if length(indice)>0
        K=indice(1)-1;%Elimino los valores donde perdió la línea
    end
    angulo{k}=angulo{k}(n0+1:K);
    velA{k}=velA{k}(n0+1:K);
    velB{k}=velB{k}(n0+1:K);
    tiempo{k}=tiempo{k}(n0+1:K);
    diferencia{k}=velB{k}-velA{k};
end
%% Integral de la diferencia de velocidades
for k=1:2
    integral{k}(1)=0;%En el instante inicial la integral arranca en 0
    for m=2:length(tiempo{k})
        integral{k}(m)=trapz(tiempo{k}(1:m),diferencia{k}(1:m));
    end
end
figure(3);subplot(211);title('Ensayo 1')
yyaxis left;plot(tiempo{1},angulo{1}-angulo{1}(1),'.');
yyaxis right;plot(tiempo{1},integral{1});grid on
legend('beta medido','integral de wB-wA');
subplot(212);;title('Ensayo 2')
yyaxis left;plot(tiempo{2},angulo{2}-angulo{2}(1),'.');
yyaxis right;plot(tiempo{2},integral{2});grid on
legend('beta medido','integral de wB-wA');
%% Estimación del parámetro
y=angulo{1}-angulo{1}(1);
y=[y angulo{2}-angulo{2}(1)];
x=[integral{1} integral{2}];
x=integral{2};y=angulo{2}-angulo{2}(1);
%figure();yyaxis left;plot(x);yyaxis right;plot(y);
f=fit(x.',y.','k*x')
figure();plot(tiempo{2},x*0.086);hold on;plot(tiempo{2},y);grid on