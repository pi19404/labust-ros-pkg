close all;
Ts=0.01;
t=[0:0.1:10];
u=[zeros(1,10),1:0.5:10,10*ones(1,31),10:-0.25:0];
ekf = ekf_init([2 0]',[100,0;0,100],Ts,@pv_model_f,@pv_model_h);

x =zeros(2,101);
xhat = x;
xn = x;
xbias = x;
xk_1 = [0 0]';
for i=1:101
    %Propagate model
    x(:,i) = pv_model_f(xk_1,Ts,0);
    xn(:,i) = x(:,i) + (0.1 - 0.2*rand(1));
    yk = pv_model_h(xn(:,i),Ts,0);
    xk_1 = x(:,i);
    %Do KF
    ekf = ekf_predict(ekf,0);
    ekf = ekf_correct(ekf,yk);
    xhat(:,i)=ekf.xk;
end

stairs(x(1,:),'b'); hold on;
stairs(xn(1,:),'r');
stairs(xhat(1,:),'g');
stairs(xhat(2,:),'k');
plot([0,101],[mean(xn(1,:)) mean(xn(1,:))],'c');
