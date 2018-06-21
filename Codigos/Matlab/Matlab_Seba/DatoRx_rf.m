function dato=DatoRx_rf(s)

% Supongo [inicio][angulo][RPMA][RPMB]
% suponer inicio=0xFF
% hay comprecion de datos: RPMA=dato_rx*1000/255 ; Se puede suponer otra,
% por ejemplo: RPMA=dato_rx*delta/255 + x0; En el transmisor tendria que
% ser asi: dato_tx=freqA*255/1000; y con la otra comprecion es:
% dato_tx=(freqA -x0)*255/delta; donde delta=valor_maximo - valor_minimo;
CantdeBytes=3; 
inicio=255;
a=0;
while a~=inicio
a=LeerSerial(s);
end
dato=zeros(1,CantdeBytes);
for i=1:CantdeBytes
dato(i)=LeerSerial(s);
end
% Descomprecion
dato(2)=dato(2)*1300.0/255.0;
dato(3)=dato(3)*1300.0/255.0;

end