function beta=ConversionSensor(byteSensor)
% Esta funci�n replica la conversi�n que hace el nano del byte del sensor a
% un valor de �ngulo con la l�nea.
    switch byteSensor
        case 1 %L�nea debajo del LED 1
          beta=-0.6981317;
        case 3 %L�nea debajo de los LEDs 1 y 2
          beta=-0.53232542;
        case 7 %L�nea debajo del LED 2
          beta=-0.32288591;
        case 2 %L�nea debajo del LED 2
          beta=-0.32288591;
        case 6 %L�nea debajo de los LEDs 2 y 3
          beta=-0.20071286;
        case 4 %L�nea debajo del LED 3
          beta=-0.16580628;
        case 14 %L�nea debajo del LED 3 (2-3-4) %%%AGREGAR AL NANO!!!
          beta=-0.16580628;
        case 12 %L�nea debajo de los LEDs 3 y 4
          beta=-0.06981317;
        case 8 %L�nea debajo del LED 4
          beta=0.052359878;
        case 28 %L�nea debajo del LED 4 (3-4-5) %%%AGREGAR AL NANO!!!
          beta=0.052359878;
        case 24 %L�nea debajo de los LEDs 4 y 5
          beta=0.161442956;
        case 16 %L�nea debajo del LED 5
          beta=0.253072742;
        case 56 %L�nea debajo del LED 5 (4-5-6) %%%AGREGAR AL NANO!!!
          beta=0.253072742;
        case 48 %L�nea debajo de los LEDs 5 y 6
          beta=0.327249235;
        case 120 %L�nea debajo de los LEDs 5 y 6 (4-5-6-7 prendidos) %%%AGREGAR AL NANO!!!
          beta=0.327249235;
        case 32 %L�nea debajo del LED 6
          beta=0.327249235;
        case 80 %L�nea debajo del LED 6 (el 6 anda m�s o menos, entonces se pueden prender s�lo el 5 y el 7)
          beta=0.327249235;
        case 112 %L�nea debajo del LED 6 (LEDs 5-6-7 prendidos)
          beta=0.327249235;
        case 240 %L�nea debajo del LED 6 (LEDs 5-6-7-8 prendidos)
          beta=0.327249235;
        case 96 %L�nea debajo de los LEDs 6 y 7 (no se di� nunca...le doy el valor de los casos de arriba)
          beta=0.327249235;
        case 64 %L�nea debajo del LED 7
          beta=0.410152374;
        case 224 %L�nea debajo del LED 7 (6-7-8) %%%AGREGAR AL NANO!!!
          beta=0.410152374;
        case 192 %L�nea debajo de los LEDs 7 y 8
          beta=0.567232007;
        case 128 %L�nea debajo del LED 8
          beta=0.772308194;
        case 0 %Se perdi� la l�nea
          beta=3;%Le doy un valor absurdo y afuera de la rutina si detecto este valor lo doy por no valido
        otherwise %La lectura del sensor es err�nea
          beta=4;%Le doy un valor absurdo y afuera de la rutina si detecto este valor lo doy por no valido
    end
end