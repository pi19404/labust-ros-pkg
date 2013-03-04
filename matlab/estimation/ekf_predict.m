function ekf = ekf_predict(ekf,input)
    %Run the prediction on X and calculate the derivatives
    [ekf.xk,Ak,Wk,Qk_1] = ekf.f(ekf.xk_1,ekf.Ts,input);
    %Update the covariance
    ekf.Pk = Ak*ekf.Pk_1*Ak' + Wk*Qk_1*Wk';
    
    ekf.Pk_1 = ekf.Pk;
    ekf.xk_1 = ekf.xk;
end