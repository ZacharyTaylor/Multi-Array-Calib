function [ camData ] = CamInfo( bag, topic, calib )
%FORDCAMINFO Sets the layout, masks and intrinsics of the camera
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing image data
%   topic- topic containing images
%   calib- struct containing camera calibration
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   camData- struct holding camera information
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

camData = struct;

%check inputs
validateattributes(topic,{'char'},{'vector'});

%get first camera image message
calibBag = select(bag,'Topic',topic);
calibBag = select(bag,'Time',[calibBag.MessageList.Time(1),calibBag.MessageList.Time(1)]);
imageMsg = readMessages(calibBag);

%get intrinsics
camData.DistModel = calib.DistModel;
camData.D = calib.D;
camData.K = calib.K;

%mask
camData.mask = true(imageMsg{1}.Height,imageMsg{1}.Width);

%timestamps
timingBag = select(bag,'Topic',topic);
camData.times = timingBag.MessageList.Time;

%topics
camData.topic = topic;
   
end
        
        
        
        
        