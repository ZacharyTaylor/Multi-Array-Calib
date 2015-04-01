function [ navData ] = FordNavInfo( path )
%FORDNAVINFO Grabs the raw data from ford nav sensor
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to where the ford dataset is stored
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   navData- struct holding navigation information
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was modified from the load_pose_applanix.m function that came
%   with the processing code for the ford dataset 
%   http://robots.engin.umich.edu/SoftwareData/Ford

%check inputs
validateattributes(path,{'char'},{'vector'});
if(~exist(path,'dir'))
    error('%s is not a valid directory');
end

navData.folder = [path '/Processed/'];
navData.files = dir([navData.folder,'*.log']);

Pose.utime = uint64(0);
Pose.pos = double(zeros(3,1));
Pose.vel = double(zeros(3,1));
Pose.orientation = double(zeros(4,1)); % quaternions
Pose.rotation_rate = double(zeros(3,1));
Pose.accel = double(zeros(3,1));

%read the file
Pose = freadstruct([navData.folder navData.files(1).name],Pose);

navData.time = Pose.utime;

%sort times into chronological order
[~,idx] = sort(navData.time,'ascend');
%remove double ups
idx = idx([diff(navData.time(idx));1] > 0);

%sort results
Pose.orientation = Pose.orientation(idx,:);
Pose.pos = Pose.pos(idx,:);
Pose.utime = Pose.utime(idx,:);
navData.time = navData.time(idx,:);

navData.T_S1_Sk = zeros(length(navData.time),6);
for i = 1:length(navData.time)
    temp = eye(4);
    temp(1:3,1:3) = quat2rot(Pose.orientation(i,:));
    temp(1:3,4) = Pose.pos(i,:);
    navData.T_S1_Sk(i,:) = T2V(temp);
end

navData.files = repmat(navData.files,size(navData.T_S1_Sk,1),1);

navData.T_Var_Skm1_Sk = repmat([0.01,0.01,0.01,0.01*pi/180,0.01*pi/180,0.01*pi/180],size(navData.T_S1_Sk,1),1);

end
