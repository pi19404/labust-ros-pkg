function ekf = ekf_correct(ekf, yk_meas)
    [ekf.yk,Hk,Vk,Rk]=ekf.h(ekf.xk, ekf.Ts, yk_meas);
    ekf.Sk = Hk*ekf.Pk*Hk' + Vk*Rk*Vk';
    Kk = ekf.Pk*Hk'*inv(ekf.Sk);
    ekf.vk = yk_meas - ekf.yk;
    ekf.xk = ekf.xk + Kk*ekf.vk;
    Kh = Kk*Hk;
    ekf.Pk = (eye(size(Kh)) - Kh)*ekf.Pk;
    
    ekf.Pk_1 = ekf.Pk;
    ekf.xk_1 = ekf.xk;
end