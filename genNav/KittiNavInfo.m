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

%sort times into chronological order
[~,idx] = sort(navData.time,'ascend');
%remove double ups
idx = idx([diff(navData.time(idx));1] > 0);

%sort results
navData.files = navData.files(idx,:);
navData.time = navData.time(idx,:);

navData.T_S1_Sk = zeros(length(navData.files),6);
navData.T_Var_Skm1_Sk = zeros(length(navData.files),6);
navData.T_Var_Skm1_Sk(1,:) = 1000*ones(1,6);

for i = 1:length(navData.files)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for nav point %i of %i', i, length(navData.files));
    end
        
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
    tformVar = [in(24),in(24),in(24),0.03*pi/180,0.03*pi/180,0.03*pi/180];
    tformVar = tformVar.^2;

%     tdiff = double(navData.time(i)-navData.time(i-1))/1000000;
%     vx = in(12)*tdiff;
%     vy = in(13)*tdiff;
%     vz = in(14)*tdiff;
%     wx = in(18)*tdiff;
%     wy = in(19)*tdiff;
%     wz = in(20)*tdiff;
%     
%     tempMat = eye(4); 
%     Rx = [1 0 0; 0 cos(wx) -sin(wx); 0 sin(wx) cos(wx)]; % base => nav  (level oxts => rotated oxts)
%     Ry = [cos(wy) 0 sin(wy); 0 1 0; -sin(wy) 0 cos(wy)]; % base => nav  (level oxts => rotated oxts)
%     Rz = [cos(wz) -sin(wz) 0; sin(wz) cos(wz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)
%     tempMat(1:3,1:3) = Rz*Ry*Rx;
%     tempMat(1:3,4) = tempMat(1:3,1:3)*[vx,vy,vz]';
% 
%     tformMat = tformMat*tempMat;
%     
%     %get variance
%     tformVar = [in(25),in(25),in(25),0.01*pi/180,0.01*pi/180,0.01*pi/180];
%     tformVar = tformVar*tdiff;
%     tformVar = tformVar.^2;

    %write to navData
    navData.T_S1_Sk(i,:) = T2V(tformMat);
    navData.T_Var_Skm1_Sk(i,:) = tformVar;
end