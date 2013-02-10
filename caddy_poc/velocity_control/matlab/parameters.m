alpha = 40;
beta = 20;

w = 3;

Kp = 2*w*alpha - beta;
Ki = alpha*w*w;
Kt = 5*Ki;
Ts = 0.1;

%Kt = w;
%Ki = alpha*w*w - Kt*beta;
%Kp = 2*alpha*w - beta - Kt*alpha;

a = [1/(Ts^2)/(w-1/Ts)^2 2/(w-1/Ts)/Ts 1];
n=a/sum(a);

alpha_d = alpha/Ts;
beta_d = beta - alpha/Ts;

dw = w*Ts-1;
wd = dw;
Kpd =2*dw*alpha_d - beta_d + alpha_d;
Kid =(alpha_d*(Ts*w)^2+beta_d+ Kpd)/Ts;

