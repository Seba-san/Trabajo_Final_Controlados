X=wA(80:end-1)-mean(wA(80:end-1));

Fs = 200;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(X);             % Length of signal
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

%%
% Fiteo de curva
%Polo mas retardo
load('../../Mediciones/respuesta_escalon_motorB_10_40_180529151917.mat')
PmR='K*(1-exp(-(x-L)/T))';
figure(1)
f1 = fit(tiempo(20:end)',wA(20:end)'-100,PmR)
plot(f1,tiempo(20:end)',wA(20:end)'-100)
title('respuesta temporal')

% Armo la funcion de transferencia
figure(2)
        estimadoZN=tf(f1.K/PWMA(end),[f1.T 1],'InputDelay',f1.L);
       % plot(tiempo(20:end),wA(20:end)-100,'.');
        plot(tiempo,wA-100,'.');
        hold on 
       % lsim(estimadoZN,PWMA(20:end),tiempo(20:end))
       PWMA1=[PWMA(1:20) PWMA(1:20) PWMA];
       tiempo1=[]
        lsim(estimadoZN,PWMA,tiempo)
        hold off
        title('respuesta considerando la tf')