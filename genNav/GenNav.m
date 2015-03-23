function [ navData ] = GenNav( path, plotNav, range, dataset )
%GENNAV Generates navigation transformations
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to the dataset to use
%   plotNav- bool, true for displaying a plot of trajectory while running
%       (note slows things down in a big way for large datasets)
%   range- 1xm vector giving the index of the tforms to use, leave empty []
%       for all points
%   dataset- string specifing the dataset type (shrimp, ford or kitti)
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
validateattributes(plotNav,{'logical'},{'scalar'});
validateattributes(dataset,{'char'},{'vector'});

if(~exist(path,'dir'))
    error('%s is not a valid directory');
end

%Get the correct raw nav information
switch(dataset)
    case('Kitti')
        navData = KittiNavInfo(path);
    case('Ford')
        navData = FordNavInfo(path);
    case('Shrimp')
        navData = ShrimpNavInfo(path);
    otherwise
        error('%s is not a valid dataset',dataset);
end

%setup help info
navData.help = ['navData stores the following information:\n'...
'folder- the folder containing the nav solution used\n'...
'files- the name of the nav files used to find the transforms\n'...
'help- this information...'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'T_Var_Skm1_Sk- the variance in the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_Var_S1_Sk- the variance in the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'time- the time at which the image was taken, epoch in microseconds\n'...
'type- sensor type (nav)'];

%fill range if empty
if(isempty(range))
    range = 1:size(navData.files(:));
end

validateattributes(range,{'numeric'},{'vector','positive','nonzero','integer'});

%get range of data
if(strcmpi(dataset,'Kitti'))
    navData.files = navData.files(range);
end
navData.time = navData.time(range);
navData.T_S1_Sk = navData.T_S1_Sk(range,:);
navData.T_Var_Skm1_Sk = navData.T_Var_Skm1_Sk;

%preallocate memory
navData.T_Skm1_Sk = zeros(size(navData.files(:),1),6);
navData.T_Skm1_Sk(1,:) = T2V(eye(4));
navData.T_Var_S1_Sk = zeros(size(navData.files(:),1),6);

%setup for plotting    
if(plotNav)
    figure;
    axis equal;
    hold on;
end

%set sensor type
navData.type = 'nav';

tempAbs = navData.T_S1_Sk;
navData.T_S1_Sk(:) = 0;

%find transform for each nav point
for frame = 2:size(navData.files,1)
    if(mod(frame,1000) == 0)
        UpdateMessage('Finding Transform for nav point %i of %i', frame-1, size(navData.files,1));
    end
    
    %find sensor transforms
    %navData.T_Skm1_Sk(frame,:) = T2V(inv(V2T(tempAbs(frame-1,:))\V2T(tempAbs(frame,:))));
    navData.T_Skm1_Sk(frame,:) = T2V(V2T(tempAbs(frame-1,:))\V2T(tempAbs(frame,:)));
    
    %generate absolute transformations
    navData.T_S1_Sk(frame,:) = T2V(V2T(navData.T_S1_Sk(frame-1,:))*V2T(navData.T_Skm1_Sk(frame,:)));
        
    if(plotNav)
        %plot points
        T = V2T(navData.T_S1_Sk(frame,:)');
        plot3(T(1,4),T(2,4),T(3,4));
        drawnow;
    end
        
    %generate absolute variance
    navData.T_Var_S1_Sk(frame,:) = navData.T_Var_S1_Sk(frame-1,:) + navData.T_Var_Skm1_Sk(frame,:);
end 

fprintf('\n');


