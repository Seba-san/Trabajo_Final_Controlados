function beta=ConversionSensor(byteSensor)
% Esta función replica la conversión que hace el nano del byte del sensor a
% un valor de ángulo con la línea.
    switch byteSensor
        case 1 %Línea debajo del LED 1
          beta=-0.6981317;
        case 3 %Línea debajo de los LEDs 1 y 2
          beta=-0.53232542;
        case 7 %Línea debajo del LED 2
          beta=-0.32288591;
        case 2 %Línea debajo del LED 2
          beta=-0.32288591;
        case 6 %Línea debajo de los LEDs 2 y 3
          beta=-0.20071286;
        case 4 %Línea debajo del LED 3
          beta=-0.16580628;
        case 14 %Línea debajo del LED 3 (2-3-4) %%%AGREGAR AL NANO!!!
          beta=-0.16580628;
        case 12 %Línea debajo de los LEDs 3 y 4
          beta=-0.06981317;
        case 8 %Línea debajo del LED 4
          beta=0.052359878;
        case 28 %Línea debajo del LED 4 (3-4-5) %%%AGREGAR AL NANO!!!
          beta=0.052359878;
        case 24 %Línea debajo de los LEDs 4 y 5
          beta=0.161442956;
        case 16 %Línea debajo del LED 5
          beta=0.253072742;
        case 56 %Línea debajo del LED 5 (4-5-6) %%%AGREGAR AL NANO!!!
          beta=0.253072742;
        case 48 %Línea debajo de los LEDs 5 y 6
          beta=0.327249235;
        case 120 %Línea debajo de los LEDs 5 y 6 (4-5-6-7 prendidos) %%%AGREGAR AL NANO!!!
          beta=0.327249235;
        case 32 %Línea debajo del LED 6
          beta=0.327249235;
        case 80 %Línea debajo del LED 6 (el 6 anda más o menos, entonces se pueden prender sólo el 5 y el 7)
          beta=0.327249235;
        case 112 %Línea debajo del LED 6 (LEDs 5-6-7 prendidos)
          beta=0.327249235;
        case 240 %Línea debajo del LED 6 (LEDs 5-6-7-8 prendidos)
          beta=0.327249235;
        case 96 %Línea debajo de los LEDs 6 y 7 (no se dió nunca...le doy el valor de los casos de arriba)
          beta=0.327249235;
        case 64 %Línea debajo del LED 7
          beta=0.410152374;
        case 224 %Línea debajo del LED 7 (6-7-8) %%%AGREGAR AL NANO!!!
          beta=0.410152374;
        case 192 %Línea debajo de los LEDs 7 y 8
          beta=0.567232007;
        case 128 %Línea debajo del LED 8
          beta=0.772308194;
        case 0 %Se perdió la línea
          beta=3;%Le doy un valor absurdo y afuera de la rutina si detecto este valor lo doy por no valido
        otherwise %La lectura del sensor es errónea
          beta=4;%Le doy un valor absurdo y afuera de la rutina si detecto este valor lo doy por no valido
    end
end