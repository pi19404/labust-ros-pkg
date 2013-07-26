function dpmode=dp_plot(data)
dpmode = find(data.hl_diagnostics.mode == 3);

yaw = data.stateHat.orientation.yaw(dpmode);
xb = data.nuRef.twist.linear.x(dpmode);
yb = data.nuRef.twist.linear.y(dpmode);

ned = zeros(2,length(dpmode));
for i=1:length(yaw)
    R = [cos(yaw(i)), -sin(yaw(i)); sin(yaw(i)), cos(yaw(i))];
    nu = [xb(i); yb(i)];
    ned(:,i) = R*nu;
end

figure;
plot(data.stateHat.position.east(dpmode),'g');
hold on;
plot(data.meas.position.east(dpmode),'r');   
plot(ned(2,:),'b--');

legend('Estimate', 'Measurement', 'Desired east speed');

figure;
plot(data.stateHat.position.north(dpmode),'g');
hold on;
plot(data.meas.position.north(dpmode),'r');   
plot(ned(1,:),'b--');

legend('Estimate', 'Measurement', 'Desired north speed');

end