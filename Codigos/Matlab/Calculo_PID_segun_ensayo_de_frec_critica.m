%% Prueba cálculo de ctes de ZN con Frec Crítica
Fsnano=200;
% Ts2=0.015;
Ts2=1/Fsnano;

%De la simulación hecha por Seba para el motor A:
% Kcr=0.28;
Kcr=0.25;%Lo bajo para probar
Pcr=55e-3;
K=9.8;%BUSCAR, ESTO ES LO QUE SEBA SE ACUERDO

tipo={'P';'PI';'PID'};
clear A B C D E F;
for k=1:3
    [Kp,Ki,Kd,~]=ConstantesZNFrecCritica(Kcr,Pcr,K,tipo{k});
    Tf=0;%No sé qué poner en Tf porque ZN no me lo da.
    control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts2,'tustin');
    [A(k,1),B(k,1),C(k,1),D(k,1),E(k,1)]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),tipo{k});
end
ctes=table(A,B,C,D,E,'RowNames',tipo)

%Para pasarle la información a Seba:
ctesP=['{' num2str(ctes.A(1)) ',' num2str(ctes.B(1)) ',' num2str(ctes.C(1)) ',' num2str(ctes.D(1)) ',' num2str(ctes.E(1)) '}'];
ctesPI=['{' num2str(ctes.A(2)) ',' num2str(ctes.B(2)) ',' num2str(ctes.C(2)) ',' num2str(ctes.D(2)) ',' num2str(ctes.E(2)) '}'];
ctesPID=['{' num2str(ctes.A(3)) ',' num2str(ctes.B(3)) ',' num2str(ctes.C(3)) ',' num2str(ctes.D(3)) ',' num2str(ctes.E(3)) '}'];