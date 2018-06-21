function [s]=InicializacionSerial_rf(PORT,baud)

bandera=1;
cuenta=0;
while (bandera)
try         %Intenta ejecutar y, si hay error, sigue con el resto del codigo
    fclose(instrfindall);       %cierra todos los puertos activos y ocultos
    %delete(instrfindall);       %elimina todo los puertos activos y ocultos
    %clear s
    disp('Puerto Cerrado')
end
        s=serial(PORT,'BaudRate',baud,'Terminator','','FlowControl','none','InputBuffer',4096);  %HAY QUE PONER EL CR EN EL MICRO!
        s.Terminator='';%Configuro que el caracter de finalizaci�n sea el CR, valor 13 en decimal.
        s.Timeout=1; % 1 segundo
        fopen(s);
        disp('Puerto Abierto')
        flushinput(s);
        flushoutput(s);
bandera2=1;
pause(3); % Lei por ahi que tarda un toque en abrirlo
while (bandera2)
    flushinput(s);
    Env_instruccion(s,'test');
    while (cuenta <7)
    salida=LeerSerial(s);
    if (salida==170)
        disp ('Se inicio correctamente')
        bandera=0;
         bandera2=0;
         cuenta=1000;
    else
        cuenta=cuenta+1;
       % flushinput(s);
       % flushoutput(s);
    end
    end
    if cuenta > 2
        cuenta=0;
        bandera2=0;
    end
end

end