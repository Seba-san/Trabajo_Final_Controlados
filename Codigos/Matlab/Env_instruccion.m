function Env_instruccion(s,info,datos)

switch info    
    case 'PWM'
        instruccion=250;
    case 'trama'
        instruccion=253;
    case 'online'
        instruccion=252;
    case 'test'
        instruccion=249;
    case 'stop'
        instruccion=251;
    case 'setpoint'
        instruccion=248;
        
end
EscribirSerial(s,instruccion);
if (nargin==3) 
    for i=1:length(datos)
    EscribirSerial(s,datos(i));
    end
end


%Dato=DatoRx_online(s)

end