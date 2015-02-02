%get test files
folder = '/home/z/Documents/Datasets/Shrimp/high-clutter-2/Processed/velodyne/';
files = dir([folder '*.ply']);

numScans = 100;
sphere = cell(numScans,1);
val = cell(numScans,1);
for i = 1:numScans
    idx = round(rand(1)*(size(files,1)-1))+1;

    %read in ply data
    data = ply_read([folder, files(idx).name]);
    xyz = [data.vertex.x, data.vertex.y, data.vertex.z];
    intensity = data.vertex.intensity;
    time = data.vertex.timeOffset;
    valid = data.vertex.valid > 0;

    %get valid sections
    xyz = xyz(valid,:);
    intensity = intensity(valid,:);
    time = time(valid);

    %convert from time to timeFrac
    timeFrac = time/(max(time) - min(time));
    
    val{i} = timeFrac;
    
    %project onto sphere
    sphere{i} = zeros(size(xyz,1),2);
    sphere{i}(:,1) = atan2(xyz(:,2), -xyz(:,1));
    sphere{i}(:,2) = atan(xyz(:,3)./ sqrt(xyz(:,2).^2 + xyz(:,1).^2));
    
    %discretize
    sphere{i} = 5000*sphere{i}/(2*pi);
    sphere{i}(:,1) = sphere{i}(:,1) + 2500;
    sphere{i}(:,2) = sphere{i}(:,2) + 50;
    sphere{i} = ceil(sphere{i});
    
    i
    
end

sphere = cell2mat(sphere);
val = cell2mat(val);

%interpolate
[xq,yq] = meshgrid(1:5000, 1:400);
base = griddata(sphere(:,1),sphere(:,2),ones(size(val)),xq,yq,'linear');
timeMatrixI = griddata(sphere(:,1),sphere(:,2),val,xq,yq);
timeMatrixE = griddata(sphere(:,1),sphere(:,2),val,xq,yq,'nearest');

timeMatrix = timeMatrixI;
timeMatrix(isnan(base)) = timeMatrixE(isnan(base));
timeMatrix(~isfinite(timeMatrix(:))) = 0;
    
save('timingMatrix.mat','timeMatrix');    