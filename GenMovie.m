% Basic template for tests

%% user set variables

%camVel = [0.268335180219155,-0.00560424184750061,-0.0715373816948283,-1.22104273580791,1.21095786046147,-1.20168273820590];

%after
camVel = [0.270482705016916,-0.0778653559733182,-0.0567838762407863,-1.21127380239188,1.20079749275924,-1.20056300528952];
%before
%camVel = [0.219474839661894,0.265581123684475,0.368883899050718,-1.21761070386342,1.20080099267351,-1.18933830996184];

navVel = [-0.835424904515285,0.133258726481109,-0.667135038366552,0.0155244008226300,0.000392089117612828,-0.00672304275823091];

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData('Kitti','Vel','Cam1','Nav');

camInfo = sensorData{2};
lidarInfo = sensorData{1};
navInfo = sensorData{3};

T = inv(V2T(camVel));
T = gpuArray(single(T));
K = gpuArray(single(camInfo.K));

navVel = V2T(navVel);

li = 1;

range = 3382;
out = cell(li,1);

for i = 2:size(navInfo.T_S1_Sk)
    navInfo.T_S1_Sk(i,:) = T2V(V2T(navInfo.T_S1_Sk(i-1,:))/V2T(navInfo.T_Skm1_Sk(i,:)));
end

for i = range
    %load image
    image = imread([camInfo.folder camInfo.files(i).name]);
    image = repmat(image,1,1,3);
    image = double(image)/255;
    
    %blur
    G = fspecial('gaussian',[10 10],1);
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
    
    %match nav with camera frame
    time = double(navInfo.time) - double(camInfo.time(i));
    [~,idxNav] = min(abs(time));

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
    
    valid(valid) = all(c > 3/255,2);
    
    %eliminate hidden points
    loc = gather(pro(valid,:));
    dist = dist(valid);
    
    %find abs tform
    tv = V2T(navInfo.T_S1_Sk(idxNav,:))*inv(navVel);

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
    
    j = mod(i,li)+1;
    out{j} = zeros(s,6);
    out{j}(:,1:3) = gather(scan(valid,1:3));
    out{j}(:,4:6) = gather(c(v,:));

    i
    
    if(j == li)
        out2 = cell2mat(out);

        clear Data;
        Data.vertex.x = single(out2(:,1));
        Data.vertex.y = single(out2(:,2));
        Data.vertex.z = single(out2(:,3));
        Data.vertex.R = single(out2(:,4));
        Data.vertex.G = single(out2(:,5));
        Data.vertex.B = single(out2(:,6));

        ply_write(Data,['./gen/' num2str(i) '.ply']);
    end
    
end
