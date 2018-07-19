function [dato]=LeerSerial(s)

tic;a=1;ii=0;
while(s.BytesAvailable<2)
    t=toc;
    if t>a
        tic;
        disp ('Esperando a que llegue algo')
        ii=ii+1;
        if ii>3
            disp ('No llego nada, se deja de esperar.')
            break; 
        else
            dato=0;
        end
    end
end
if ii<3
    dato=fread(s,1,'uint8');      %Lee el serial y lo interpreta como un entero sin signo de 8 bits; Ver: https://la.mathworks.com/help/matlab/ref/serial.fread.html#f102-512501 
end
end