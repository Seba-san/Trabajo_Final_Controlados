function [Kp,Ki,Kd,Parametros]=ConstantesZNFrecCritica(Kcr,Pcr,K,control)
%Esta función calcula los coeficientes de ZN según el método de frec
%crítica dado en el Ogata.
Kp=(1/K)*Kcr*[0.5;0.45;0.6];
Ki=(1/K)*[0;1.2/Pcr;0.5/Pcr];
Kd=(1/K)*[0;0;0.125*Pcr];
Parametros=table(Kp,Ki,Kd,'RowName',{'P';'PI';'PID'});

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
