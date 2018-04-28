function Comunic_test(s)
    bandera=1;
    while (bandera)
    Env_instruccion(s,'test');
    salida=str2double(fscanf(s));
    if (salida==170)
        disp ('Se logro comuncar')
        bandera=0;   
    end
    end

end