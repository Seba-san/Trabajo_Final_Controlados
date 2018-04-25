function [x]=ascii2entero(y)
%Esta funci�n convierte el n�mero entero almacenado en y e forma de
%caracteres a un entero en la variable x. El vector y tiene una cifra del
%n�mero en cada una de sus componentes.
N=length(y);
x=0;
for i=1:N
    x=x+(y(i)-48)*10^(N-i);
end

end