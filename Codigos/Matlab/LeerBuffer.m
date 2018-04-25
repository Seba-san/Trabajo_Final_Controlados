function [w,u]=LeerBuffer(s,CantdeBytes)
% Proyecto Final de Controlados 2018
% Medici�n de velocidad de un motor.
% Esta funci�n lee el buffer del objeto serial s y las acomoda en dos
% vectores de muestras, w y u, seg�n la forma de comunicaci�n elegida con
% el microcontrolador. La forma de transmisi�n es la siguiente: el micro
% toma las muestras de la velocidad del motor y de la velocidad de la se�al
% de control (en float), las convierte a ASCII y las transmite en ese
% orden, separadas por un espacio (32 en decimal), seguido de un CR (13 en
% decimal). La idea es detectar las tramas usando el CR como referencia.
% Se lee un m�ximo de CantdeBytes bytes del buffer. Para no limitar el
% par�metro se debe poner CantdeBytes igual al tama�o del buffer.

dato=fread(s,CantdeBytes,'uchar');
% Antes yo ten�a: flushinput(s), pero si hago eso me va a borrar una
% posible trama a medio recibir y despu�s voy a leer dicha trama cortada
% (porque podr�a recibir bien el espacio y el CR, pero haber cortado
% algunas cifras del primer n�mero transmitido.
fin=find(dato==13);%Busco todos los CR
espacio=find(dato==32);%Busco todos los espacios
indice=1;
%Deber�a darse que fin(k-1)<espacio(k)<fin(k), expecto en la primer trama
%donde no habr�a un fin(k-1).
if espacio(1)<fin(1) %Verifico que la trama es v�lida
    w(indice)=dato(1):
end
for k=1:length(fin)%Para todas las tramas enteras encontradas
    if fin(k-1)<espacio(k) && espacio(k)<fin(k) %Verifico que la trama es v�lida
        algo
    end
end
end