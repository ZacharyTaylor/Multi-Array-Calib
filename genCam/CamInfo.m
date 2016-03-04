function [ camData ] = CamInfo( bag, topic, calibTopic )
%FORDCAMINFO Sets the layout, masks and intrinsics of the camera
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing image data
%   topic- topic containing images
%   calibTopic- topic containing camera calibration
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
validateattributes(calibTopic,{'char'},{'vector'});

%get first calibration message
calibBag = select(bag,'Topic',calibTopic);
calibBag = select(bag,'Time',[calibBag.MessageList.Time(1),calibBag.MessageList.Time(1)]);
calibMsg = readMessages(calibBag);

%get intrinsics
camData.DistModel = calibMsg{1}.DistModel;
camData.D = calibMsg{1}.DistCoeff;
camData.K = [calibMsg{1}.FocalLength(1),0,calibMsg{1}.PrincipalPoint(1);0,calibMsg{1}.FocalLength(2),calibMsg{1}.PrincipalPoint(2);0,0,1];

%mask
camData.mask = true(calibMsg{1}.ImageHeight,calibMsg{1}.ImageWidth);

%timestamps
timingBag = select(bag,'Topic',topic);
camData.times = timingBag.MessageList.Time;

%topics
camData.topic = topic;
   
end
        
        
        
        
        