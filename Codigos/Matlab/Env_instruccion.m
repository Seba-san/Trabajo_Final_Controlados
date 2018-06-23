function Env_instruccion(s,info,datos)
parametro=1;
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
        parametro=10;
    case 'parametros'
        instruccion=247;
    case 'Ucontrol'
        instruccion=246;
    case 'devolver ensayo'
        instruccion=245;
    case 'control_off'
        instruccion=244;
    case 'control_on'
        instruccion=243;
       
end
EscribirSerial(s,instruccion);
%  if instruccion==247
%             datos=datos*10000; % Es para que se puedan enviar numeros con coma
%  end
if (nargin==3) 
    for i=1:length(datos)
       
    EscribirSerial(s,fix(datos(i)/parametro));
    end
end


%Dato=DatoRx_online(s)

end