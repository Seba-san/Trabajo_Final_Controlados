%% 
% Se van a realizar algunas pruebas en simulacion y luego con nuestros
% algoritmos.
num=[100];
%den=[1 -3 1];
den=poly([-1 -5]); % Raices 
sys_ideal=tf(num, den);
step(sys_ideal)


