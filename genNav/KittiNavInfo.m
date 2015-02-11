function [ navData ] = KittiNavInfo( path )
%KITTINAVINFO Grabs the raw data from kittis nav sensor
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to where the kitti dataset is stored
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
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%check inputs
validateattributes(path,{'char'},{'vector'});
if(~exist(path,'dir'))
    error('%s is not a valid directory');
end

navData.folder = [path '/oxts/data/'];
navData.files = dir([navData.folder,'*.txt']);
navData.time = ReadKittiTimestamps([navData.folder '../']);

navData.T_S1_Sk = zeros(length(navData.files),6);
navData.T_Var_Skm1_Sk = zeros(length(navData.files),6);

for i = 1:length(navData.files)
    
    UpdateMessage('Reading Transform for nav point %i of %i', i, length(navData.files));

    fid = fopen([navData.folder navData.files(i).name], 'r');

    in = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','CollectOutput', 1);
    in = in{1};
    fclose(fid);

    % compute scale from first lat value
    [x,y,~] = deg2utm(in(1),in(2));
    t = [x,y,in(3)];

    rx = in(4); % roll
    ry = in(5); % pitch
    rz = in(6); % heading 
    Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
    Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
    Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)

    tformMat = eye(4);
    tformMat(1:3,1:3)  = Rz*Ry*Rx;
    tformMat(1:3,4) = t;

    %get variance
    tformVar = [in(24),in(24),in(24),0.01*pi/180,0.01*pi/180,0.01*pi/180];
    tformVar = tformVar.^2;

    %write to navData
    navData.T_S1_Sk(i,:) = T2V(tformMat);
    navData.T_Var_Skm1_Sk(i,:) = tformVar;
end

navData.T_Var_Skm1_Sk(2:end,4:6) = navData.T_Var_Skm1_Sk(2:end,4:6).*repmat(diff(double(navData.time)/1000000),1,3);