function [ camData ] = GenCam( bag, plotCam, topic, calib, range )
%GENCAM Generates camera transformations
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing image data
%   plotCam- bool, true for displaying a plot of trajectory while running
%       (note slows things down in a big way for large datasets)
%   topic- topic containing images
%   calib- struct containing calibration information
%   range- 1xm vector giving the index of the images to use, leave empty []
%       for all images
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
validateattributes(plotCam,{'logical'},{'scalar'});
validateattributes(topic,{'char'},{'vector'});

%Get the camera information
camData = CamInfo(bag, topic, calib);

%setup help info
camData.help = ['camData stores the following information:\n'...
'help- this information...'...
'K- the camera matrix\n'...,
'D- the distortion vector for the camera, matches opencv implementation'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'T_Var_Skm1_Sk- the variance in the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_Var_S1_Sk- the variance in the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'times- the time at which the image was taken, epoch in microseconds\n'...
'topic- the topic the images were published to\n'...
'type- sensor type (camera)'];

%set sensor type
camData.type = 'camera';

%fill range if empty
if(isempty(range))
   range = 1:length(camData.times);
end

validateattributes(range,{'numeric'},{'vector','positive','nonzero','integer'});

%only read relevent messages
bag = select(bag,'Topic',topic);

%get range of data
camData.times = camData.times(range);
bag = select(bag,'Time',[camData.times, camData.times]);

%preallocate memory
camData.T_Skm1_Sk = zeros(size(camData.times(:),1),6);
camData.T_S1_Sk = zeros(size(camData.times(:),1),6);

camData.T_Var_Skm1_Sk = zeros(size(camData.times(:),1),6);
camData.T_Var_S1_Sk = zeros(size(camData.times(:),1),6);
camData.T_Var_Skm1_Sk(1,:) = 1000*ones(1,6);

%setup for plotting
if(plotCam)
    figure;
    axis equal;
    hold on;
end

%load the first image
image = readMessages(bag, 1);
image = reshape(image{1}.Data,image{1}.Width,image{1}.Height)';

image = Undistort(image, camData.D, camData.K, camData.DistModel);

%find the transforms for each image
for frame = 2:size(camData.times,1)

    UpdateMessage('Finding Transform for image %i of %i for camera topic %s', frame,size(camData.times,1),topic);

    imageOld = image;
    
    %read new data
    image = readMessages(bag, frame);
    image = reshape(image{1}.Data,image{1}.Width,image{1}.Height)';
    image = Undistort(image, camData.D, camData.K, camData.DistModel);

    %find transformation
    try
        [temp, tempVar] = GetCamTform(imageOld,image, camData.mask, camData.K);
        temp(~isfinite(temp)) = 0;
        tempVar(~isfinite(temp)) = 1000;
        
        camData.T_Skm1_Sk(frame,:) = temp;
        camData.T_Var_Skm1_Sk(frame,:) = tempVar;
    catch
        camData.T_Skm1_Sk(frame,:) = [0,0,0,0,0,0];
        camData.T_Var_Skm1_Sk(frame,:) = 1000*ones(1,6);
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

