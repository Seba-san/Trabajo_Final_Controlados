function [beta_est]=est_beta(t,dW,beta0,parametros,forma)
if forma=='discreto'
    beta=parametros.beta;
    delay=parametros.delay;
Ts=parametros.Ts;
k=parametros.k;
N=length(t);
sys=parametros.sys;
sys_d=c2d(sys,Ts,'zoh');%'tustin');
a=cell2mat(sys_d.den);
b=cell2mat(sys_d.num);
    beta_est=zeros(1,N);
    beta_est(1)=beta0;
    for i=2:N
        try
        beta_est(i)=b(2)*dW(i+delay)+b(1)*dW(i-1+delay)+-a(2)*beta(i-1);
        end
    end
    
end

if forma=='continuo'
    sys=parametros.sys;
    beta_est=lsim(sys,dW,t);
    
end
end