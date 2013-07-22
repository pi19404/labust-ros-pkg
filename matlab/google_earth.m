
% kreira KML iz global_position

n1=1;
n=length(data.stateHat.global_position.latitude);
fid = fopen('google_earth.kml','w+'); % daj mu ime
%fid = fopen(FigNameKML,'w+');
fprintf(fid,'%s\r\n','<?xml version="1.0" encoding="UTF-8"?>');
fprintf(fid,'%s\r\n','<kml xmlns="http://www.opengis.net/kml/2.2">');
fprintf(fid,'%s\r\n','<Folder>');
fprintf(fid,'%s\r\n','<name>Paths</name>');
fprintf(fid,'%s\r\n','<description>AUV Mission Path</description>');
fprintf(fid,'%s\r\n','<Placemark>');
fprintf(fid,'%s\r\n','<name>Absolute</name>');
fprintf(fid,'%s\r\n','<description>Mission Trajectory</description>');
fprintf(fid,'%s\r\n','<Style id="RedLine">');
fprintf(fid,'%s\r\n','<LineStyle>');
fprintf(fid,'%s\r\n','<color>ff0000ff</color>');
fprintf(fid,'%s\r\n','</LineStyle>');
fprintf(fid,'%s\r\n','</Style>');
fprintf(fid,'%s\r\n','<styleUrl>#RedLine</styleUrl>');
fprintf(fid,'%s\r\n','<LineString>');
fprintf(fid,'%s\r\n','<altitudeMode>relative</altitudeMode>');
fprintf(fid,'%s\r\n','<coordinates>');
for kk=1:n-n1 % da ne bi kml bio preveliki nekad je bolje uzeti ne svaku vec svaku 10 ili 20 to?ku
     fprintf(fid,'%s\r\n',strcat(num2str(data.stateHat.global_position.longitude(kk),'%14.10f'),',',num2str(data.stateHat.global_position.latitude(kk),'%14.10f')));%,num2str(DFS(kk))));
end
fprintf(fid,'%s\r\n','</coordinates>');
fprintf(fid,'%s\r\n','</LineString>');
fprintf(fid,'%s\r\n','</Placemark>');
fprintf(fid,'%s\r\n','</Folder>');
fprintf(fid,'%s\r\n','</kml>');

fclose(fid);