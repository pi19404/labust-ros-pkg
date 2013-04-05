function Structure_PDAF_Show(TrackList,DataList,Par)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Structure_PDAF_Show - draws the data and tracks for each time interval
% Input:
%   TrackList    - kalman structure list and more
%   DataList     - 2 x DataPointsNum contains the relevant data for time t
%   Par          - parameters
% Output:
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%persistent ShowFigNum;
ShowFigNum  = 15;            % show figure number
AxisSc      = [Par.Y1Bounds Par.Y1Bounds]*1.1+[-.1 0 -.1 0];
SmallShift  = 5e-3;     % used for text display
NumSigma    = sqrt(Par.GateLevel); % number of sigmas


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show points
DLen = size(DataList,2);
figure(ShowFigNum), hold on;


% first time call for this function
persistent DataPlotHand;

if isempty(DataPlotHand),
    
    DataPlotHand = plot(DataList(1,:), DataList(2,:), 'b.'); 
    axis(AxisSc);
    title('Data Points & Tracks')
    xlabel('X1'),ylabel('X2')
end;

delete(DataPlotHand);
DataPlotHand = plot(DataList(1,:), DataList(2,:), 'b.'); 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% show tracking objects
figure(ShowFigNum), hold on;
    
% loop throught all tracks
TrackNum = length(TrackList);
ValidStatesForShow = [Par.State_Track Par.State_FirstCoast:Par.State_LastCoast];

    
% first time call for this function
persistent TrackPlotHand;
persistent TrackNumHand;
 
t = 0:.1:2*pi+.1;
circle = [cos(t);sin(t)]*NumSigma; %

% print state development
TrackNumStr = [];
TrackStateStr = [];

if isempty(TrackPlotHand),
    
     for i=1:TrackNum,
         y = TrackList{i}.TrackObj.H * TrackList{i}.TrackObj.x;
         H = TrackList{i}.TrackObj.H;
         P = TrackList{i}.TrackObj.P;
         R = TrackList{i}.TrackObj.R;
         
         S = H*P*H' + R; 
         [u,sing,v] = svd(S);
         %elipse = u*sing*circle;
         elipse = u*sqrt(sing)*circle;
         TrackPlotHand(i) = plot(elipse(1,:)+y(1), elipse(2,:)+y(2),'r'); 
         TrackNumHand(i) = text(y(1)+SmallShift,y(2),num2str(i),'FontSize',8);
         
         TrackNumStr = [TrackNumStr sprintf(' %2.0d',i)];
         
     end;

 % one time print
 if ~Par.ShowOn, fprintf('%s\n',TrackNumStr); end;
   
 end;
 
 
 delete(TrackPlotHand);
 delete(TrackNumHand);
  
 for i=1:TrackNum,
     
     
     y = TrackList{i}.TrackObj.H * TrackList{i}.TrackObj.x;
     H = TrackList{i}.TrackObj.H;
     P = TrackList{i}.TrackObj.P;
     R = TrackList{i}.TrackObj.R;
     
     S = H*P*H' + R;          
     %S

     [u,sing,v] = svd(S);
     elipse = u*sqrt(sing)*circle;
      %elipse = u*sing*circle;
     
     if ~any(TrackList{i}.TrackObj.State == ValidStatesForShow),
         y = [NaN;NaN]; % to prevent show
     end;

     
     TrackPlotHand(i) = plot(elipse(1,:)+y(1), elipse(2,:)+y(2),'r'); 
     TrackNumHand(i) = text(y(1)+SmallShift,y(2),num2str(i),'FontSize',8);
     
     
     TrackStateStr = [TrackStateStr sprintf(' %2.0d',TrackList{i}.TrackObj.State)];
     %TrackStateStr = [TrackStateStr sprintf(' %2.0i',round(TrackList{i}.TrackObj.LogLike*100))];
     
 end;

 if ~Par.ShowOn, fprintf('%s\n',TrackStateStr); end;
       
drawnow; 
hold off;
