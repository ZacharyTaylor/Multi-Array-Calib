% this script generates all the required transforms for the kitti and
% shrimp dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%path to data
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_10_03_drive_0027_extract/image_01/data/';

K = [990.3522,0,702,0;0,985.5674,260.7319,0;0,0,1,0];
D = [-3.712084e-01 1.978723e-01 -3.709831e-05 -3.440494e-04 -6.724045e-02];

%Sets if the sensor transforms will be plotted
plotTforms = false;

range = 49:3048;

%% setup folders

%velodyne processing
addpath('./genKittiVel/');
addpath('./genKittiVel/libicp/matlab/');

%nav processing
addpath('./genKittiNav');

%cam processing
addpath('./genKittiCam');

addpath('./handEye');

files = dir([kittiPath,'*.png']);
files = files(range);

image = imread([kittiPath files(1).name]); 
if(size(image,3) == 3)
    image = rgb2gray(image);
end
image = uint8(undistort(double(image), D, K(1:3,1:3)));

for frame = 2:size(files,1)

        %% get inital tform
        
        fprintf('Finding Transform for image %i of %i\n', frame,size(files,1));

        imageOld = image;
        %read new data
        image = imread([kittiPath files(frame).name]); 
        if(size(image,3) == 3)
            image = rgb2gray(image);
        end
        image = uint8(undistort(double(image), D, K(1:3,1:3)));

        %detect features in images
        pointsOld = detectMinEigenFeatures(imageOld);
        pointsOld = pointsOld.Location;

        pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
        initialize(pointTracker, pointsOld, imageOld);
        [pointsNew, matches] = step(pointTracker, image);
        release(pointTracker);

        pointsOld = pointsOld(matches,:);
        pointsNew = pointsNew(matches,:);
    
        [~, inliers] = estimateFundamentalMatrix(pointsOld,pointsNew,'NumTrials',500);
        pointsOld = pointsOld(inliers,:);
        pointsNew = pointsNew(inliers,:);
    
        out = ['/home/z/Documents/Houdini/ICRA2015/cameraMatches/', num2str(frame-1,'%04i'),'.png'];
        showMatchedFeatures(imageOld, image, pointsOld, pointsNew);
        drawnow;
        saveas(gcf, out);
        
end
