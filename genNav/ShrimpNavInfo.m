function [ navData ] = ShrimpNavInfo( path )
%KITTINAVINFO Grabs the raw data from shrimps nav sensor
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

navData.folder = [path '/Processed/novatel/'];
navData.files = dir([navData.folder,'*.bin']);

blockSize = 100000;

fid = fopen([navData.folder navData.files(1).name],'r');

time = []; pos = []; speed = []; err = [];

run = true;

%read a block of data
idx = 0;
while run
    fseek(fid,116*blockSize*idx,'bof');
    t = fread(fid, blockSize, '*uint64', 104);
    fseek(fid,8+116*blockSize*idx,'bof');
    
    x = fread(fid, blockSize, '*double', 104);
    fseek(fid,16+116*blockSize*idx,'bof');
    y = fread(fid, blockSize, '*double', 104);
    fseek(fid,24+116*blockSize*idx,'bof');
    z = fread(fid, blockSize, '*double', 104);
    fseek(fid,32+116*blockSize*idx,'bof');
    
    rx = fread(fid, blockSize, '*double', 104);
    fseek(fid,40+116*blockSize*idx,'bof');
    ry = fread(fid, blockSize, '*double', 104);
    fseek(fid,48+116*blockSize*idx,'bof');
    rz = fread(fid, blockSize, '*double', 104);
    fseek(fid,56+116*blockSize*idx,'bof');
    
    vx = fread(fid, blockSize, '*double', 104);
    fseek(fid,64+116*blockSize*idx,'bof');
    vy = fread(fid, blockSize, '*double', 104);
    fseek(fid,72+116*blockSize*idx,'bof');
    vz = fread(fid, blockSize, '*double', 104);
    fseek(fid,80+116*blockSize*idx,'bof');
    
    wx = fread(fid, blockSize, '*double', 104);
    fseek(fid,88+116*blockSize*idx,'bof');
    wy = fread(fid, blockSize, '*double', 104);
    fseek(fid,96+116*blockSize*idx,'bof');
    wz = fread(fid, blockSize, '*double', 104);
    
    fseek(fid,104+116*blockSize*idx,'bof');
    e = fread(fid, blockSize, '*double', 104);
    
    if(feof(fid))
        run = false;
    end
    
    idx = idx +1;
    
    l = min([length(x),length(y),length(z),length(rx),length(ry),length(rz), length(vx), length(vy), length(vz), length(wx), length(wy), length(wz), length(t), length(e)]);
    
    t = t(1:l);
    
    x = x(1:l);
    y = y(1:l);
    z = z(1:l);
    
    rx = rx(1:l);
    ry = ry(1:l);
    rz = rz(1:l);
    
    vx = vx(1:l);
    vy = vy(1:l);
    vz = vz(1:l);
    
    wx = wx(1:l);
    wy = wy(1:l);
    wz = wz(1:l);
    
    e = e(1:l);
    
    time = [time; t];
    l = min([length(x),length(y),length(z),length(rx),length(ry),length(rz), length(vx), length(vy), length(vz), length(wx), length(wy), length(wz), length(t)]);
    
    tempPos = [x y z rx ry rz];

    pos = [pos;tempPos];
    
    e = repmat(e,1,6);
    e(:,1:3) = e(:,1:3).^2;
    e(:,4:6) = (0.03*pi/180).^2;
    
    speed = [speed ;vx vy vz wx wy wz];
    err = [err; e];
end

fclose(fid);

%remove doubleups in position
idx = all([diff(pos);[1,1,1,1,1,1]] ~= 0,2);
time = time(idx,:);
pos = pos(idx,:);
speed = speed(idx,:);
err = err(idx,:);

%sort times into chronological order
[~,idx] = sort(time,'ascend');
%remove double ups
idx = idx([diff(time(idx));1] > 0);

%sort results
time = time(idx,:);
pos = pos(idx,:);
speed = speed(idx,:);
err = err(idx,:);

navData.time = time;
navData.T_Var_Skm1_Sk = err;

navData.T_S1_Sk = zeros(size(pos));

%convert from shrimp coordinate system to matlabs
for j = 1:size(time,1)
    tempMat = eye(4); 
    tempMat(1:3,1:3) = inv(angle2dcm(pos(j,6),pos(j,5),pos(j,4),'ZYX'));
    tempMat(1:3,4) = pos(j,1:3)';
    navData.T_S1_Sk(j,:) = T2V(tempMat)';
end

navData.files = repmat(navData.files,size(navData.T_S1_Sk,1),1);

end

