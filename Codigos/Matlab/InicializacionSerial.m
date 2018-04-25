function [s]=InicializacionSerial(PORT,baud)

% PORT='COM8'; %Determinaci�n del puerto de conexi�n
% baud=500000; %Velocidad de transmisi�n
try         %Intenta ejecutar y, si hay error, sigue con el resto del codigo
    fclose(instrfindall);       %cierra todos los puertos activos y ocultos
    delete(instrfindall);       %elimina todo los puertos activos y ocultos
    clear s
    disp('Puerto Cerrado')
end

s=serial(PORT,'BaudRate',baud,'Terminator','CR','FlowControl','none','InputBuffer',4096);  %HAY QUE PONER EL CR EN EL MICRO!
%"serial" inicializa la comunicaci�n (crea un objeto serial) a
%la velocidad indicada (baud), con un fin de comunicaci�n
%marcado por 'CR', sin control de flujo.
%--------------------------------------------------------------------------
% Esta parte es para tener interrupciones cada vez que se detecta el
% caracter de finalizaci�n (terminator):
%s.BytesAvailableFcnMode = 'terminator';
%s.BytesAvailableFcn = @TramaDisponible; % Esto genera un error en la ejecucion de EscribirSerial.
s.Terminator='CR';%Configuro que el caracter de finalizaci�n sea el CR, valor 13 en decimal.
s.Timeout=1; % 1 segundo
%--------------------------------------------------------------------------
%
fopen(s);
disp('Puerto Abierto')
flushinput(s);
flushoutput(s);
%Env_instruccion(s,'inicio');
%[Dato]=DatoRx_online(s)
bandera=1;
cuenta=0;
while (bandera)
    Env_instruccion(s,'test');
    salida=str2num(fscanf(s));
    if (salida==170)
        disp ('Se inicio correctamente')
        bandera=0;
    else
        cuenta=cuenta+1;
        flushinput(s);
flushoutput(s);
    end
    if cuenta > 10
        cuenta=0;
        try         %Intenta ejecutar y, si hay error, sigue con el resto del codigo
            fclose(instrfindall);       %cierra todos los puertos activos y ocultos
            delete(instrfindall);       %elimina todo los puertos activos y ocultos
            clear s
            disp('No hay conexion, se reinicia la conexion')
        end
        s=serial(PORT,'BaudRate',baud,'Terminator','CR','FlowControl','none','InputBuffer',4096);  %HAY QUE PONER EL CR EN EL MICRO!
        s.Terminator='CR';%Configuro que el caracter de finalizaci�n sea el CR, valor 13 en decimal.
        s.Timeout=1; % 1 segundo
        fopen(s);
        disp('Puerto Abierto')
        flushinput(s);
        flushoutput(s);
    end
end
end