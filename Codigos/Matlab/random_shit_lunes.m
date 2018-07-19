%%
Kp=c.Kp;Ki=c.Ki;Kd=c.Kd;
% disp([Kp,Ki,Kd]);
Ts=1/200;
Tf=0;
control=c2d(tf(pid(Kp,Ki,Kd,Tf)),Ts,'tustin');
[A,B,C,D,E]=tf2ctesNano(cell2mat(control.num),cell2mat(control.den),'PID');
disp([A,B,C,D,E]);
%%
sysd=c2d(idpoly(P1DI),Ts,'tustin');
%% Armando los sistemas
%Sistema de los motores controlador:
MCA=feedback(series(Ca,sysA),1);
MCB=feedback(series(Cb,sysB),1);
%%
Ts=1/200;
sysd=c2d(idpoly(P1DI),Ts,'tustin');
systotdisc=series(sysd,MCA);
systotcont=d2c(systot);