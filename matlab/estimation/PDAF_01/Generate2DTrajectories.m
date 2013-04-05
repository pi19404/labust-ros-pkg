function [y,t,dT] = Generate2DTrajectories(TrajType,dT,Time)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate2DTrajectories - generates trajectories
% in 2D for tracking. Trajectory is defined by TrajType.
% y is 2D 2xT data confined to [0 1] square
% Input:
%   TrajType    - trajectory type
%   dT          - delta time between samples (in seconds)
%   Time        - final simulation time in seconds
% Output:
%   y           - 2xT vector of measurements
%   t           - 1xT time for each measurement
%   dT          - input dT if not specified
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parameters 
% 
if nargin < 3, Time = 3; end;
if nargin < 2, dT = 1/30;end; % dT between samples (video frame rate)
if nargin < 1, TrajType = 1; end;

t       = 0:dT:Time;
N       = length(t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% trajectory generation
switch TrajType,
case 1,             % circle
    fx  = .2;
    y   = [cos(2*pi*fx*t); sin(2*pi*fx*t)]*.5+.5;
    
case 2,             % jump in x
    SlopeV = 5;
    tmp = logsig(SlopeV*(t-Time/2));
    y   = [tmp;t./Time];

case 3,             % spiral
    R   = 0.5 + sin(2*pi/Time*t)*.5;
    Ang = 2*pi*1*t;
    y   = [R.*cos(Ang); R.*sin(Ang)]*.5+.5;
    
case 4,             % 8
    fx  = .5;
    fy  = 1;
    y   = [cos(2*pi*fx*t); sin(2*pi*fy*t)]*.5+.5;
    
case 5,             % exp
    SlopeV = 5;
    y   = [1-exp(-t./Time*SlopeV); t./Time];   
    
case 6,             % triang
    y   = [t./Time; triang(N)'];    
    
case 7,             % rising
    SlopeV = 1; N3 = round(N/3);
    y   = [t./Time; [zeros(1,N3) linspace(0,SlopeV,N3) ones(1,N-2*N3)*SlopeV]];    
    
case 8,             % non-moving random point
    tmp = rand(2,1);
    y   = tmp(:,ones(1,N));    
   
case 9,             % straight line from left upper to right lower
    LeftH = .95; RightH = 0.05;
    y   = [t./Time; linspace(LeftH,RightH,N)];    

case 10,             % straight line from left lower to right high corners
    LeftH = .05; RightH = 0.95;
    y   = [t./Time; linspace(LeftH,RightH,N)];    
    
case 11,             % straight line from left upper to right lower
    LeftH = .75; RightH = 0.25;
    y   = [t./Time; linspace(LeftH,RightH,N)];    

case 12,             % straight line from left lower to right high corners
    LeftH = .25; RightH = 0.75;
    y   = [t./Time; linspace(LeftH,RightH,N)];    
    
otherwise
    error('Unknown method.')    
end;