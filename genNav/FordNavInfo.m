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

navData.T_S1_Sk = zeros(length(navData.time),6);
for i = 1:length(navData.time)
    navData.T_S1_Sk(i,:) = [R2V([quat2dcm(Pose.orientation(i,:)'));Pose.pos(i,:)'];
end

navData.T_Var_Skm1_Sk = err;
end
