function draw_xy(meas, stateHat, testd, imu, n0, n1)
figure(1);
plot(meas.position.east(n0:n1), meas.position.north(n0:n1),'r');
hold on;
%plot(stateHat.position.east(n0:n1), stateHat.position.north(n0:n1),'k');
plot(stateHat.position.east(n0:n1), stateHat.position.north(n0:n1),'g');
plot(testd.position.east(n0:n1), testd.position.north(n0:n1),'b');

figure(2);
plot(meas.orientation.yaw(n0:n1),'r');
hold on;
plot(stateHat.orientation.yaw(n0:n1),'g');
plot(testd.orientation.yaw(n0:n1),'b');

figure(3);
plot(imu.gyro_z(n0:n1),'r');
hold on;
plot(stateHat.orientation_rate.yaw(n0:n1),'g');
plot(testd.orientation_rate.yaw(n0:n1),'b');

figure(4);
plot(stateHat.body_velocity.x(n0:n1),'g');
hold on;
plot(testd.body_velocity.x(n0:n1),'b');

end