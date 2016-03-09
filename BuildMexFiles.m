%Builds the required mex files
cd('./tforms');
mex V2R.cpp
cd('../');

cd('./handEye');
mex logpdfT.cpp
mex logpdfR.cpp
cd('../');