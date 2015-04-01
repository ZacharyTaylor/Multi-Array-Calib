%Builds the required mex files

cd('./tforms');
mex V2R.cpp
cd('../');

cd('./imageMetric');
mex colourImage.cpp
cd('../');

cd('./handEye');
mex logpdfT.cpp
mex logpdfR.cpp
cd('../');

fprintf('Basic functions have finished building\n');
fprintf('\nNote this script will not build projectLidarMex or interpolateImageUint8Mex\n');
fprintf('   these files make use of cuda and all the linking and such needed to build is kind of tricky\n');
fprintf('   either use the provided binaries or use the provided makefiles or vsprojects in mex source\n');
fprintf('This script also will not build icp mex due to its use of boost\n');
fprintf('   either use the provided binary or use the provided makefile in genVel/velicp\n');