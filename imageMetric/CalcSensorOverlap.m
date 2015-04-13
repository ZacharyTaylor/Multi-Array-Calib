function [ overlap ] = CalcSensorOverlap( tform, sensorA, sensorB )
%CALCSENSOROVERLAP calculates overlap in two sensors field of view assuming
%   there is minimal translation between them
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   tform- 1x6 transformation vector between the two sensors
%   sensorA- sensorData struct for sensor A
%   sensorB- sensorData sturct for sensor B
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   overlap- overlap in FOV (0 to 1)
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
validateattributes(tform,{'numeric'},{'vector','numel',6});
validateattributes(sensorA,{'struct'},{});
validateattributes(sensorB,{'struct'},{});

%get rotation
rotMat = V2R(tform(4:6));

%resolution of generated FOV rectangles
res = 0.01;

%get first sensor FOV
if(strcmpi(sensorA.type,'Velodyne'))
    xa = [-pi; pi; pi; -pi];
    ya = [0.035; 0.035; -0.433; -0.433];
    ya = ya + pi/2;
elseif(strcmpi(sensorA.type,'Nav'))
    overlap = 0;
    return;
else
    image = imread([sensorA.folder sensorA.files(1).name]);
    imsize = size(image(:,:,1));
    fovA = atan(imsize(:)./[sensorA.K(1,1); sensorA.K(2,2)]); 

    xa = [-fovA(1)/2; fovA(1)/2; fovA(1)/2; -fovA(1)/2];
    ya = [fovA(2)/2; fovA(2)/2; -fovA(2)/2; -fovA(2)/2];
    
end
[xa,ya] = interpm(xa,ya,res);

%get FOV of 2nd sensor
if(strcmpi(sensorB.type,'Velodyne'))
    xb = [-pi; pi; pi; -pi];
    yb = [0.035; 0.035; -0.433; -0.433];
    yb = yb + pi/2;
elseif(strcmpi(sensorB.type,'Nav'))
    overlap = 0;
    return;
else
    image = imread([sensorB.folder sensorB.files(1).name]);
    imsize = size(image(:,:,1));
    fovB = atan(imsize(:)./[sensorB.K(1,1); sensorB.K(2,2)]); 

    xb = [-fovB(1)/2; fovB(1)/2; fovB(1)/2; -fovB(1)/2];
    yb = [fovB(2)/2; fovB(2)/2; -fovB(2)/2; -fovB(2)/2];
    
end
[xb,yb] = interpm(xb,yb,res);

%calculate overlap
[x, y] = polybool('intersection', xa, ya, xb, yb);
if(strcmpi(sensorA.type,'Velodyne'))
	overlap = polyarea(x,y)/(fovB(1)*fovB(2));
else
    overlap = polyarea(x,y)/min(fovA(1)*fovA(2),fovB(1)*fovB(2));
end

end

