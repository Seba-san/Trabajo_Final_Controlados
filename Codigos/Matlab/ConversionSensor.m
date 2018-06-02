function beta=ConversionSensor(byteSensor,cantBytesCorrer)
% Esta función replica la conversión que hace el nano del byte del sensor a
% un valor de ángulo con la línea.
% Con cantBytesCorrer desplazo los LEDs para compenzar que no está centrada
% la línea.
betaEnsayo=[-0.6981317,-0.53232542,-0.32288591,-0.20071286,-0.16580628,...
    -0.06981317,0.052359878,0.161442956,0.253072742,0.327249235,0.327249235...
    ,0.327249235,0.410152374,0.567232007,0.772308194];
byte2=dec2bin(byteSensor*2^cantBytesCorrer);%Me devuelve un char
N=length(byte2);
if N>8 %aparecen LEDs ficticios
    byte=byte2(N-8+1:N);
    N=length(byte);
else
    byte=byte2;
end
LED=zeros(1,8);
for k=0:N-1
    LED(k+1)=str2num(byte(N-k));
end
suma=sum(LED);
sumaPonderada=2*sum(LED.*(1:8));
if(suma==0)
    beta=3;%Si suma=0 es poque no detectó la línea, así que le doy el valor de error
else
aux=round(sumaPonderada/suma)%Potencial fuente de error: la división de enteros. $.$ Ver si es problema
%Le pongo el round para que me redondee los casos erroneos
beta=betaEnsayo(aux-1);
end
end