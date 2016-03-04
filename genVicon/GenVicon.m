function [ viconData ] = GenVicon( bag, plotVicon, topic, range )
%GENNAV Generates navigation transformations
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   bag- bag containing vicon data
%   plotVicon- bool, true for displaying a plot of trajectory while running
%       (note slows things down in a big way for large datasets)
%   topic- topic containing vicon
%   range- 1xm vector giving the index of the data to use, leave empty []
%       for all vicon poses
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
validateattributes(plotVicon,{'logical'},{'scalar'});
validateattributes(topic,{'char'},{'vector'});


%Get the raw vicon information
viconData = ViconInfo(bag, topic);

%setup help info
viconData.help = ['viconData stores the following information:\n'...
'help- this information...'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'T_Var_Skm1_Sk- the variance in the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_Var_S1_Sk- the variance in the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'...
'times- the time at which the data was received, epoch in microseconds\n'...
'topic- the topic the poses were published to\n'...
'type- sensor type (vicon)'];

%fill range if empty
if(isempty(range))
    range = 1:size(viconData.times(:));
end

validateattributes(range,{'numeric'},{'vector','positive','nonzero','integer'});

%get range of data
viconData.times = viconData.times(range);
viconData.T_S1_Sk = viconData.T_S1_Sk(range,:);
viconData.T_Var_Skm1_Sk = viconData.T_Var_Skm1_Sk;

%preallocate memory
viconData.T_Skm1_Sk = zeros(size(viconData.times(:),1),6);
viconData.T_Skm1_Sk(1,:) = T2V(eye(4));
viconData.T_Var_S1_Sk = zeros(size(viconData.times(:),1),6);

%setup for plotting    
if(plotVicon)
    figure;
    axis equal;
    hold on;
end

%set sensor type
viconData.type = 'vicon';

tempAbs = viconData.T_S1_Sk;
viconData.T_S1_Sk(:) = 0;

%find transform for each nav point
for frame = 2:size(viconData.times,1)
    if(mod(frame,1000) == 0)
        UpdateMessage('Finding Transform for Vicon point %i of %i', frame-1, size(viconData.times,1));
    end
    
    %find sensor transforms
    viconData.T_Skm1_Sk(frame,:) = T2V(inv(V2T(tempAbs(frame-1,:))\V2T(tempAbs(frame,:))));
    
    %generate absolute transformations
    viconData.T_S1_Sk(frame,:) = T2V(V2T(viconData.T_S1_Sk(frame-1,:))*V2T(viconData.T_Skm1_Sk(frame,:)));
        
    if(plotVicon)
        %plot points
        T = V2T(viconData.T_S1_Sk(frame,:));
        plot3(T(1,4),T(2,4),T(3,4));
        drawnow;
    end
        
    %generate absolute variance
    viconData.T_Var_S1_Sk(frame,:) = viconData.T_Var_Skm1_Sk(frame,:);
end 

fprintf('\n');


