% Basic template for tests

%% user set variables

camVel = [0.268335180219155,-0.00560424184750061,-0.0715373816948283,-1.22104273580791,1.21095786046147,-1.20168273820590];

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData('Kitti','Vel','Cam3');

camInfo = sensorData{2};
lidarInfo = sensorData{1};

T = inv(V2T(camVel));
T = gpuArray(single(T));
K = gpuArray(single(camInfo.K));

tv = eye(4);

for i = 3:303
    %load image
    image = imread([camInfo.folder camInfo.files(i).name]);
    image = double(image)/255;
    
    %blur
    G = fspecial('gaussian',[100 100],1);
    for j = 1:size(image,3)
        image(:,:,j) = imfilter(image(:,:,j),G,'same');
    end
        
    %undistort image
    for j = 1:size(image,3)
        image(:,:,j) = Undistort(double(image(:,:,j)), camInfo.D, camInfo.K);
    end
        
    %move to gpu
    image = gpuArray(single(image));
    
    %match lidar with camera frame
    time = double(lidarInfo.time) - double(camInfo.time(i));
    [~,idx] = min(abs(time));
    t = double(lidarInfo.time(idx));
    tNext = double(lidarInfo.time(idx+1));

    %read lidar data
    [xyz, intensity, timeFrac] = ReadVelData([lidarInfo.folder lidarInfo.files(idx).name]);

    %correct for motion during scan
    xyz = VelCorrect(xyz, timeFrac, lidarInfo.T_Skm1_Sk(idx,:));

    %find transformation between scans
    tform = V2T(lidarInfo.T_Skm1_Sk(idx,:));

    %get difference in points
    dPoints = (tform*[xyz, ones(size(xyz,1),1)]')';
    dPoints = dPoints(:,1:3) - xyz;

    %interpolate point positions
    tStartFrac = (double(camInfo.time(i))-t)/(tNext - t);

    %combine
    xyz = xyz + tStartFrac*dPoints;
    
    %get distance to points
    dist = sqrt(sum(xyz.^2,2));

    %move everything to the gpu
    scan = gpuArray(single(xyz));
    
    %get point colours
    [pro, valid] = projectLidar(T, K, scan(:,1:3), [size(image,1),size(image,2)]);    
    c = interpolateImage(image, pro(valid,1:2));
    
    %eliminate hidden points
    loc = gather(pro(valid,:));
    dist = dist(valid);
    
    %find abs tform
    tv = tv*inv(V2T(lidarInfo.T_Skm1_Sk(idx,:)));

    %get difference in points
    scan = (tv*[scan(:,1:3), ones(size(scan,1),1)]')';
    
    %find closest points
    idx = knnsearch(loc,loc,'k',10);
    idx = idx(:,2:end);
    diff = reshape(dist(idx),size(idx)) - repmat(dist,1,size(idx,2));
    diff = min(diff,[],2)./dist;
    v = diff > -0.1;
    valid(valid) = v;
    
    s = gather(sum(valid));
    out = zeros(s,6);
    out(:,1:3) = gather(scan(valid,1:3));
    out(:,4:6) = gather(c(v,:));
    
    clear Data;
    Data.vertex.x = single(out(:,1));
    Data.vertex.y = single(out(:,2));
    Data.vertex.z = single(out(:,3));
    Data.vertex.R = single(out(:,4));
    Data.vertex.G = single(out(:,5));
    Data.vertex.B = single(out(:,6));
    
    ply_write(Data,['./gen/' num2str(i) '.ply']);
    i
    
end
