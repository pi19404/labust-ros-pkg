function circle_plot(data)
figure;

circmode = find(data.hl_diagnostics.mode == 4);

%data.stamp.t(circmode(end)) - data.stamp.t(circmode(1))


hold on;
plot(data.sf_diagnostics.position.east(circmode), data.sf_diagnostics.position.north(circmode),'b');
plot(data.stateHat.position.east(circmode), data.stateHat.position.north(circmode),'g');
plot(data.meas.position.east(circmode), data.meas.position.north(circmode),'r')

figure(2);
plot(data.meas.position.east(circmode), data.meas.position.north(circmode),'r')


cnc = [circmode(1):100:circmode(end)];
for i=1:length(cnc)
    plot([data.sf_diagnostics.position.east(cnc(i)), data.stateHat.position.east(cnc(i))],... 
    [data.sf_diagnostics.position.north(cnc(i)), data.stateHat.position.north(cnc(i))],'k');

daspect([1 1 1]);
end