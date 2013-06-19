function ekf = ekf_init(x0, P0, Ts, ekf_f,ekf_h)
    ekf.xk = x0;
    ekf.xk_1 = x0;
    ekf.P = P0;
    ekf.Pk_1 = P0;
    ekf.Ts = Ts;
    ekf.f = ekf_f;
    ekf.h = ekf_h;
end