function []=EscribirSerial(s,dato)
%dato es un vector cuyas componentes van de 0 a 255 (bytes)
%     N=length(dato);
%     vector(1)=1;
%     vector(2)=N;
%     vector(3:2+N)=dato;
%     suma=sum(dato);
%     while(suma>255)
%         suma=suma-256;
%     end
%     vector(N+3)=suma;
%     fwrite(s,vector,'uchar');
flushinput(s); % COmo voy a pedir datos, no me interesa lo anterior.
    fwrite(s,dato,'uchar');
end