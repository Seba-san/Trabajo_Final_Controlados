function [A,B,C,D,E]=tf2ctesNano(numPID,denPID,tipo)
% Esta función convierte la función de tranferencia de un controlador PID a
% las constantes necesarias para el nano. Para ello se le debe dar como
% entradas el numerador y denominador de la función de transferencia.
% En el nano se uso la siguiente ecuación:
% u(k)=Ae(k)+Be(k-1)+Ce(k-2)+Du(k-1)+Eu(k-2), por lo que la función de
% transferencia del controlador debe ser:
% H(z)=(A+Bz^-1+Cz^-2)/(1-Dz^-1-Ez^-2) (se sabe que Matlab tira por defecto
% un coeficiente principal de 1 para el denominador de la función de
% transferencia).
A=0;B=0;C=0;D=0;E=0;
switch tipo
    case 'P'
        A=numPID(1);
    case 'PI'
        A=numPID(1);B=numPID(2);D=-denPID(2);
    case 'PID'
        A=numPID(1);B=numPID(2);C=numPID(3);D=-denPID(2);E=-denPID(3);
end
end
        
