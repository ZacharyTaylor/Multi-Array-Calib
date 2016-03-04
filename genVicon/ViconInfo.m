function [ viconData ] = ViconInfo( bag, topic )
%KITTINAVINFO Gets information regarding vicon data
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing image data
%   topic- topic containing vicon data
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   viconData- struct holding vicon information
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
viconData.topic = topic;

%filter bag
bag = select(bag,'Topic',topic);

%timestamaps
viconData.times = bag.MessageList.Time;

%preallocate memory
viconData.T_S1_Sk = zeros(length(viconData.times),6);
viconData.T_Var_Skm1_Sk = zeros(length(viconData.times),6);
viconData.T_Var_Skm1_Sk(1,:) = 1000*ones(1,6);

for i = 1:length(viconData.times)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for vicon transform %i of %i', i, length(viconData.times));
    end
    
    in = readMessages(bag, i);
    
    tformMat = eye(4);
    tformMat(1:3,1:3)  = quat2rot([in{1}.Transform.Rotation.W, in{1}.Transform.Rotation.X, in{1}.Transform.Rotation.Y, in{1}.Transform.Rotation.Z]');
    tformMat(1:3,4) = [in{1}.Transform.Translation.X, in{1}.Transform.Translation.Y, in{1}.Transform.Translation.Z];

    %get variance
    tformVar = [0.001,0.001,0.001,0.001*pi/180,0.001*pi/180,0.001*pi/180];
    tformVar = tformVar.^2;

    %write to navData
    viconData.T_S1_Sk(i,:) = T2V(tformMat);
    viconData.T_Var_Skm1_Sk(i,:) = tformVar;
end