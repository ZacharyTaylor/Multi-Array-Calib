
R = cell(5,1);
R{1} = eye(3);
R{2} = V2R([0.00156862564717513,1.25898709931336,0.00709582651079661]);
R{3} = V2R([0.00288999277500487,2.51317663766461,0.00921341463144476]);
R{4} = V2R([0.00122534746685227,-2.51300315909813,-0.00352673202040254]);
R{5} = V2R([0.00266609363143122,-1.25597909006783,-0.00567080545234518]);

cam = cell(5,1);
cam{1} = [621,811.8100,403.431];
cam{2} = [623,810.0020,410.810];
cam{3} = [628,810.7710,412.206];
cam{4} = [609,816.0450,409.433];
cam{5} = [627,826.4070,404.472];

sensorData = LoadSensorData('Shrimp','Cam1','Cam2','Cam3','Cam4','Cam5');

data = load('./results/Test_8.2.mat');

out = cell(5,1);
imNum = 1620;

r = zeros(5,3);
for i = 1:length(data.results)
    r = r + data.results{i}.rot;
end
r = r/length(data.results);

for i = 1:5
    R{i} = V2R(r(i,:));
end

for i = 1:size(cam,1)
    %load image
    image = imread([sensorData{i}.folder, sensorData{i}.files(imNum).name]);
    sensorData{i}.mask = any(image ~= 0,3);
    se = strel('disk',11);        
    sensorData{i}.mask = imerode(sensorData{i}.mask,se);
    
    %get image coordinates
    [x,y] = meshgrid(0:(size(image,2)-1),0:(size(image,1)-1));
    x = x(sensorData{i}.mask);
    y = y(sensorData{i}.mask);
    
    c1 = image(:,:,1); c1 = c1(sensorData{i}.mask);
    c2 = image(:,:,2); c2 = c2(sensorData{i}.mask);
    c3 = image(:,:,3); c3 = c3(sensorData{i}.mask);
    
    x = x - cam{1}(1);
    y = y - cam{1}(2);
    
    d = sqrt(x.^2 + y.^2);
    
    v = [x,y,repmat(cam{i}(3),length(x),1)];
    v = v./repmat(sqrt(sum(v.^2,2)),1,3);
    
    %convert to global
    v = (R{i}*v')';
    
    %convert to angles
    a = asin(v(:,2));
    b = atan2(v(:,1),v(:,3));
    out{i} = [b,a,double([c1,c2,c3]),d];
end

out = cell2mat(out);

out(:,1) = out(:,1) - min(out(:,1));
out(:,2) = out(:,2) - min(out(:,2));
out(:,1:2) = out(:,1:2)*(2500/max(max(out(:,1:2))));
out(:,1:2) = round(out(:,1:2))+1;

[~,idx] = sort(out(:,6),'ascend');
out = out(idx,:);
[~,idx] = unique(out(:,1:2),'rows','first');
out = out(idx,:);

im = zeros([max(out(:,2)),max(out(:,1)),3]);

idx = sub2ind(size(im), out(:,2), out(:,1), ones(size(out,1),1));
im(idx) = out(:,3);
idx = sub2ind(size(im), out(:,2), out(:,1), 2*ones(size(out,1),1));
im(idx) = out(:,4);
idx = sub2ind(size(im), out(:,2), out(:,1), 3*ones(size(out,1),1));
im(idx) = out(:,5);
imsave(imshow(uint8(im)));

