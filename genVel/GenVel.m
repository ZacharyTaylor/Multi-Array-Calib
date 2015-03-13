function [ velData ] = GenVel( path, plotVel, range, dataset )
%GENCAM Generates velodyne transformations
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to the dataset to use
%   plotVel- bool, true for displaying a plot of trajectory while running
%       (note slows things down in a big way for large datasets)
%   range- 1xm vector giving the index of the images to use, leave empty []
%       for all scans
%   dataset- string specifing the dataset type (shrimp, ford or kitti)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   velData- struct holding velodyne information
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
validateattributes(plotVel,{'logical'},{'scalar'});
validateattributes(dataset,{'char'},{'vector'});

if(~exist(path,'dir'))
    error('%s is not a valid directory');
end

%Get the correct velodyne information
switch(dataset)
    case('Kitti')
        velData = KittiVelInfo(path);
    case('Ford')
        velData = FordVelInfo(path);
    case('Shrimp')
        velData = ShrimpVelInfo(path);
    otherwise
        error('%s is not a valid dataset',dataset);
end

%setup help info
velData.help = ['camData stores the following information:\n'...
'help- this information...'...
'folder- the folder containing the scans used\n'...
'files- the name of the scan files used to find the transforms\n'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'T_Var_Skm1_Sk- the variance in the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_Var_S1_Sk- the variance in the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'time- the time at which half of the scan had been recorded, epoch in microseconds\n'...
'type- sensor type (velodyne)'];

%set sensor type
velData.type = 'velodyne';

%fill range if empty
if(isempty(range))
   range = 1:length(velData.files);
end

validateattributes(range,{'numeric'},{'vector','positive','nonzero','integer'});

%get range of data
velData.files = velData.files(range);
velData.time = velData.time(range);

%preallocate memory
velData.T_Skm1_Sk = zeros(size(velData.files(:),1),7);
velData.T_S1_Sk = zeros(size(velData.files(:),1),7);

velData.T_Var_Skm1_Sk = zeros(size(velData.files(:),1),7);
velData.T_Var_S1_Sk = zeros(size(velData.files(:),1),7);
velData.T_Var_Skm1_Sk(1,:) = 1000*ones(1,7);

%setup for plotting
if(plotVel)
    figure;
    axis equal;
    hold on;
end

%load the first scan
[velCurr, ~, tCurr] = ReadVelData([velData.folder velData.files(1).name]);

%find the transforms for each scan
for frame = 2:size(velData.files,1)

    UpdateMessage('Finding Transform for scan %i of %i', frame,size(velData.files,1));

    velPrev = velCurr;
    tPrev = tCurr;
    
    %read new data
    [velCurr, ~, tCurr] = ReadVelData([velData.folder velData.files(frame).name]);

    %find transformation
    try
        [velData.T_Skm1_Sk(frame,:), velData.T_Var_Skm1_Sk(frame,:)] = GetVelTform(velCurr, velPrev, tCurr, tPrev, velData.T_Skm1_Sk(frame-1,:),40000,10);
    catch
      velData.T_Skm1_Sk(frame,:) = [0,0,0,0,0,0,0];
      velData.T_Var_Skm1_Sk(frame,:) = 1000*ones(1,7);
    end
       
    %generate absolute transformations
    velData.T_S1_Sk(frame,:) = T2V(V2T(velData.T_S1_Sk(frame-1,:))*V2T(velData.T_Skm1_Sk(frame,:)));
    velData.T_Var_S1_Sk(frame,:) = velData.T_Var_S1_Sk(frame-1,:) + velData.T_Var_Skm1_Sk(frame,:);
    
    %plot
    if(plotVel)
        plot3(velData.T_S1_Sk(frame-1:frame,1),velData.T_S1_Sk(frame-1:frame,2),velData.T_S1_Sk(frame-1:frame,3));
        drawnow;
    end
end

fprintf('\n');

end