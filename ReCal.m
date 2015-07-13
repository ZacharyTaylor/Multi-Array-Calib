% Basic template for tests
function [] = ReCal()

navCamGT = V2T([-0.815190930387079,0.249688618286015,-1.13444188556610,1.22361935881520,-1.21839638464709,1.19688941135166]);
K = [707.091200000000,0,601.887300000000;0,707.091200000000,183.110400000000;0,0,1];

    a = RunReCal(navCamGT, K, 100);
    
navCamGT = V2T([-0.815190930387079,0.249688618286015,-1.13444188556610,1.22361935881520,-1.31839638464709,1.19688941135166]);

    b = RunReCal(navCamGT, K, 100);

end

function [out] = RunReCal(navCamGT, K, num)

    path = 'C:\Users\Zachary\Documents\Datasets\Kitti\2011_09_30_drive_0028_sync';

    navData.folder = [path '/oxts/data/'];
    navData.files = dir([navData.folder,'*.txt']);

    camData.folder = [path '/image_00/data/'];
    camData.files = dir([camData.folder,'*.png']);

    %setup
    tform = eye(4);
    points = zeros(0,2);
    image = [];
    out = zeros(num,1);

    %start loop
    for i = 1:num%length(navData.files)       
        tPrev = tform;
        tform = ReadNavFile(navData,i);
        imPrev = image;
        image = imread([camData.folder, camData.files(i).name]);

        tic
        %find corners
        pPrev = points;
        points = detectFASTFeatures(image);
        %points = points.Location;
        [~,idx] = sort(points.Metric,'descend');
        points = points.Location(idx(1:min(length(idx),1000)),:);

        if(i == 1)
            continue;
        end

        %get nav movement
        tDiff = tPrev\tform;

        %transform to camera movement
        tDiff = navCamGT/tDiff/navCamGT;

        %normalize translation
        tDiff(1:3,4) = tDiff(1:3,4)./norm(tDiff(1:3,4));
        tDiff = inv(tDiff);

        %create essential matrix
        E = tDiff(1:3,1:3)*[0, -tDiff(3,4), tDiff(2,4); tDiff(3,4), 0, -tDiff(1,4); -tDiff(2,4), tDiff(1,4), 0];

        %create fundemental matrix
        F = K'\E/K;
        F = F / norm(F);

        %find error
        err = pErr(single(pPrev'),single(points'),single(F));
        
        %showMatchedFeatures(imPrev,image,pPrev(idx,:),points);
        err = sort(err);
        %err = mean(err(1:ceil(0.1*length(err))));
        out(i) = median(err);
        
        m = R2V(tDiff(1:3,1:3));
        out(i) = out(i)/norm(m);
        i
    end

end

function [tform, tVar] = ReadNavFile(navData,i)
    %read value
    fid = fopen([navData.folder navData.files(i).name], 'r');
    in = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','CollectOutput', 1);
    in = in{1};
    fclose(fid);

    % compute scale from first lat value
    [x,y,~] = deg2utm(in(1),in(2));
    t = [x,y,in(3)];

    rx = in(4); % roll
    ry = in(5); % pitch
    rz = in(6); % heading 
    Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
    Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
    Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)

    tform = eye(4);
    tform(1:3,1:3)  = Rz*Ry*Rx;
    tform(1:3,4) = t;
    
    %get variance
    tVar = [in(24),in(24),in(24),0.01*pi/180,0.01*pi/180,0.01*pi/180];
    tVar = tVar.^2;
end