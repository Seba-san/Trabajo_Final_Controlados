function Comunic_test_rf(s)
    bandera=1;
    while (bandera)
    Env_instruccion(s,'test');
    salida=LeerSerial(s);
    if (salida==170)
        disp ('Se logro comuncar')
        bandera=0;   
    end
    end

end