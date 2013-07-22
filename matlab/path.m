% Prvi dio je za ART/BART ili generalno za kruznicu plavo i centar fiksni
% ili ne crveno


n1=2; % od
n=length(data.stateHat.position.east); % do
plot(data.stateHat.position.east(n1:n),data.stateHat.position.north(n1:n),'b');

for n2=1:n
    if (data.hl_diagnostics.ref_point.point.y(n2)==0)
        data.hl_diagnostics.ref_point.point.y(n2)=NaN;
    end
    if (data.hl_diagnostics.ref_point.point.x(n2)==0)
        data.hl_diagnostics.ref_point.point.x(n2)=NaN;
    end
end
hold on;         
plot(data.hl_diagnostics.ref_point.point.y(n1:n),data.hl_diagnostics.ref_point.point.x(n1:n),'r');
daspect([1 1 1]);

% Ovo je izvucena najprostija brzina iz GPSa i onda mao smooth-ana

for i=2:n
    vel(i)=10*sqrt((data.stateHat.position.north(i)-data.stateHat.position.north(i-1))*(data.stateHat.position.north(i)-data.stateHat.position.north(i-1))+(data.stateHat.position.east(i)-data.stateHat.position.east(i-1))*(data.stateHat.position.east(i)-data.stateHat.position.east(i-1)));
    if ((abs(vel(i)-vel(i-1))>0.1) && (i>2))
        vel(i)=vel(i-1);
    end
end
vel(1)=0;
vel2=smooth(vel,10);
figure;
plot(vel2);