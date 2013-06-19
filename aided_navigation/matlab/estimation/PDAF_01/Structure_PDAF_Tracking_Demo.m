%function Structure_PDAF_Tracking_Demo
% Tracking a moving point in 2D plane
% State = (x xdot y  ydot). We only observe (x y).

close all,clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init parameters 
Par             = Structure_PDAF_Init_Parameters;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init data 
AllTheData      = Structure_PDAF_Init_Data(Par);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init Kalman Filter structures
TrackList       = Structure_PDAF_Init_Track(Par);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PDAF filtering
%FrameNum    = ;
%if Par.RecordOn, M = moviein(FrameNum); end;
for k = 1:size(AllTheData,3),
    
    % show
    if Par.ShowOn, fprintf('%3.0d : -----------------\n',k); end;
    
    % get the data for time k
    DataList    = AllTheData(:,:,k); % get the data at time 1
    
    % show
    Structure_PDAF_Show(TrackList,DataList,Par);
    if Par.RecordOn, M(:,k) = getframe; end;


    % data-track association
    TrackList = Structure_PDAF_Association(TrackList,DataList,Par);
        
    % track update
    TrackList = Structure_PDAF_Track_Update(TrackList,DataList,Par);
    
    % track separation
    TrackList = Structure_PDAF_Track_Separation(TrackList,DataList,Par);    
    
    % start new tracks
    TrackList = Structure_PDAF_Track_Start(TrackList,DataList,Par);
    
    % recording
    %Record = Structure_PDAF_Record(Record,TrackList,DataList);

end;
if Par.RecordOn,  figure, movie(M); end;
%movie2avi(M,'Tracks6Clutter12.avi','fps',5,'quality',95)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% show results
%Structure_PDAF_Show(DataList,TrackList);


