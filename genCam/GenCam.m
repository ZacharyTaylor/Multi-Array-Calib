function [ camData ] = GenCam( path, plotCam, range, dataset, idx )
%GENCAM Generates camera transformations
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to the dataset to use
%   plotCam- bool, true for displaying a plot of trajectory while running
%       (note slows things down in a big way for large datasets)
%   range- 1xm vector giving the index of the images to use, leave empty []
%       for all images
%   dataset- string specifing the dataset type (shrimp, ford or kitti)
%   idx- index of the camera to use (1-4 for kitti, 1-5 for ford, 1-6
%       for shrimp)
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

%check inputs
validateattributes(path,{'char'},{'vector'});
validateattributes(plotCam,{'logical'},{'scalar'});
validateattributes(dataset,{'char'},{'vector'});
validateattributes(idx,{'numeric'},{'scalar','nonzero','positive','integer'});

if(~exist(path,'dir'))
    error('%s is not a valid directory');
end


%Get the correct camera information
switch(dataset)
    case('Kitti')
        camData = KittiCamInfo(path, idx);
    case('Ford')
        camData = FordCamInfo(path, idx);
    case('Shrimp')
        camData = ShrimpCamInfo(path, idx);
    otherwise
        error('%s is not a valid dataset',dataset);
end

%setup help info
camData.help = ['camData stores the following information:\n'...
'help- this information...'...
'folder- the folder containing the images used\n'...
'K- the camera matrix\n'...,
'D- the distortion vector for the camera, matches opencv implementation'...
'files- the name of the image files used to find the transforms\n'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'T_Var_Skm1_Sk- the variance in the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_Var_S1_Sk- the variance in the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'time- the time at which the image was taken, epoch in microseconds\n'...
'type- sensor type (camera)'];

%set sensor type
camData.type = 'camera';

%find all camera images
camData.files = [dir([camData.folder,'*.jpg']);dir([camData.folder,'*.png'])];

%fill range if empty
if(isempty(range))
   range = 1:length(camData.files);
end

validateattributes(range,{'numeric'},{'vector','positive','nonzero','integer'});

%get range of data
camData.files = camData.files(range);
camData.time = camData.time(range);

%preallocate memory
camData.T_Skm1_Sk = zeros(size(camData.files(:),1),7);
camData.T_S1_Sk = zeros(size(camData.files(:),1),7);

camData.T_Var_Skm1_Sk = zeros(size(camData.files(:),1),7);
camData.T_Var_S1_Sk = zeros(size(camData.files(:),1),7);
camData.T_Var_Skm1_Sk(1:2,:) = 1000*ones(2,7);

%setup for plotting
if(plotCam)
    figure;
    axis equal;
    hold on;
end

%load the first image
image = imread([camData.folder camData.files(1).name]); 
if(size(image,3) == 3)
    image = rgb2gray(image);
end
imageOld = Undistort(image, camData.D, camData.K);

%load the second image
image = imread([camData.folder camData.files(2).name]); 
if(size(image,3) == 3)
    image = rgb2gray(image);
end
image = Undistort(image, camData.D, camData.K);

scale = 1;
scaleVar = 0;

%find the transforms for each image
for frame = 3:size(camData.files,1)

    UpdateMessage('Finding Transform for image %i of %i for camera %i', frame,size(camData.files,1),idx);

    imageOldest = imageOld;
    imageOld = image;
    
    %read new data
    image = imread([camData.folder camData.files(frame).name]); 
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    image = Undistort(image, camData.D, camData.K);

    %find transformation
    try
        [temp, tempVar] = GetCamTform3(imageOldest,imageOld,image, camData.mask, camData.K);
        temp(~isfinite(temp)) = 0;
        temp(4) = temp(4)*scale;
        if(temp(4) ~= 0)
            scale = abs(temp(4));
        end
        
        tempVar(~isfinite(temp)) = 1000;
        tempVar(4) = tempVar(4) * scale;
        %scaleVar = tempVar(4);
        
        camData.T_Skm1_Sk(frame,:) = temp;
        camData.T_Var_Skm1_Sk(frame,:) = tempVar;
    catch
        camData.T_Skm1_Sk(frame,:) = [0,0,0,0,0,0,0];
        camData.T_Var_Skm1_Sk(frame,:) = 1000*ones(1,7);
    end
       
    %generate absolute transformations
    camData.T_S1_Sk(frame,:) = T2V(V2T(camData.T_S1_Sk(frame-1,:))*V2T(camData.T_Skm1_Sk(frame,:)));
    camData.T_Var_S1_Sk(frame,:) = camData.T_Var_S1_Sk(frame-1,:) + camData.T_Var_Skm1_Sk(frame,:);
    
    %plot
    if(plotCam)
        plot3(camData.T_S1_Sk(frame-1:frame,1),camData.T_S1_Sk(frame-1:frame,2),camData.T_S1_Sk(frame-1:frame,3));
        drawnow;
    end
end

fprintf('\n');

end

