function [dato]=LeerSerial(s,CantdeBytes)

    %CantdeBytes=5; %Este par�metro se fijo en 5, podr�a modificarse
    while(s.BytesAvailable<CantdeBytes)end   %Espera a que llegue algun byte
    dato=fread(s,CantdeBytes,'uchar');      %Lee el serial y lo pone en pwm (si es mas de uno se pone en formato vector)
%     dato(length(dato))=[];
    flushinput(s);  %Vacia el buffer de entrada
    flushoutput(s); %Vacia el buffer de salida
end