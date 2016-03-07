function [ viData ] = VIOInfo( bag, topic )
%KITTINAVINFO Gets information regarding visual-inertial data
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing image data
%   topic- topic containing VI data
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   viconData- struct holding VI information
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

%check inputs
validateattributes(topic,{'char'},{'vector'});

%topics
viData.topic = topic;

%filter bag
bag = select(bag,'Topic',topic);

%timestamaps
viData.times = bag.MessageList.Time;

%preallocate memory
viData.T_S1_Sk = zeros(length(viData.times),6);
viData.T_Var_S1_Sk = zeros(length(viData.times),6);
viData.T_Var_S1_Sk(1,:) = 1000*ones(1,6);

for i = 1:length(viData.times)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for VI transform %i of %i', i, length(viData.times));
    end
    
    in = readMessages(bag, i);
    
    tformMat = eye(4);
    or = in{1}.Pose.Pose.Orientation;
    pos = in{1}.Pose.Pose.Position;
    tformMat(1:3,1:3)  = quat2rot([or.W, or.X, or.Y, or.Z]');
    tformMat(1:3,4) = [pos.X, pos.Y, pos.Z];

    %get variance
    tformVar = diag(reshape(in{1}.Pose.Covariance,6,6));

    %write to navData
    viData.T_S1_Sk(i,:) = T2V(tformMat);
    viData.T_Var_S1_Sk(i,:) = tformVar;
end