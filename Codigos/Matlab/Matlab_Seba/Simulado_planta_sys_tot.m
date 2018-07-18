%% Antes de empezar poner esto
addpath('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab/Matlab_Seba')
cd('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Codigos/Matlab')
%%
% Cargo las mediciones 
% load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180622220141respuesta_escalon_systot_scontrolador_.mat')
%load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624183411respuesta_escalon_systot_scontrolador_.mat')
 %load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624184018respuesta_escalon_systot_scontrolador_.mat')
%load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624184237respuesta_escalon_systot_scontrolador_.mat') 
% load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624183234respuesta_escalon_systot_scontrolador_.mat')
% load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624183137respuesta_escalon_systot_scontrolador_.mat')
 load('/media/seba/Datos/Facultad_bk/Controlados/Trabajo_Final/Trabajo_Final_Controlados_git/Mediciones/180624180816respuesta_escalon_systot_scontrolador_.mat')

%% Grafico de los Ensayos
figure(1);subplot(211); title('Ensayo')
plot(tiempo,betas,'b.',tiempo,control,'r.');legend('angulo','control');grid on;xlabel('tiempo (s)');ylabel('angulo (rad)')
subplot(212);
plot(tiempo,wA,'b',tiempo,wB,'r');legend('motor A','motor B');xlabel('tiempo (s)');ylabel('F_{ang} (RPM)')
grid on
%% 
% Acomodo el angulo para sacar los valores fuera de rango e identificar
% cuando empieza el ensayo
indice=find(control==0);betas2=betas(indice);wA2=wA(indice);wB2=wB(indice);tiempo2=tiempo(indice);
indice=find(betas2<3);betas2=betas2(indice);wA2=wA2(indice);wB2=wB2(indice);tiempo2=tiempo2(indice);
dW=wB2-wA2;
% Muestro los datos filtrados
figure(1);subplot(211); title('Ensayo')
plot(tiempo2,betas2,'b.');legend('angulo');grid on;xlabel('tiempo (s)');ylabel('angulo (rad)')
subplot(212);
plot(tiempo2,wA2,'b',tiempo2,wB2,'r');legend('motor A','motor B');xlabel('tiempo (s)');ylabel('F_{ang} (RPM)')
grid on


