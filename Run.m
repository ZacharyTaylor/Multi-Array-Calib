% Basic template for tests

%% user set variables
%samples
timeSamples = 10000;

%if time offset should be performed
findTimeOffset = false;

%shows a plot of rotated data for sanity check
plotData = false;

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData('Vicon1','VI1');

%% fix timestamps
if(findTimeOffset)
    fprintf('Finding Timing Offsets\n');
else
    fprintf('Preprocessing Data\n');
end

[sensorData, offsets, varOffsets] = CorrectTimestamps(sensorData, timeSamples, findTimeOffset);

if(findTimeOffset)
    fprintf('Offsets:\n');
    disp(offsets);
    fprintf('Offset sd:\n');
    disp(sqrt(varOffsets));
end

%% run calibration
    
%evenly sample data
sData = SampleData(sensorData);

%remove uninformative data
sData = RejectPoints(sData, 10, 0.00001);
    
%find rotation
fprintf('Finding Rotation\n');
rotVec = RoughR(sData);
rotVec = OptR(sData, rotVec);
rotVarL = ErrorEstCR(sData, rotVec);
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));

fprintf('Rotation:\n');
disp(rotVec);
fprintf('Rotation sd:\n');
disp(sqrt(rotVarM));

%find camera transformation scale (only used for RoughT, OptT does its
%own smarter/better thing
fprintf('Finding Camera Scale\n');
sDataS = EasyScale(sData, rotVec, rotVarL,zeros(2,3),ones(2,3));

%show what we are dealing with
if(plotData)
    PlotData(sDataS,rotVec);
end

fprintf('Finding Translation\n');
tranVec = RoughT(sDataS, rotVec);
tranVec = OptT(sData, tranVec, rotVec, rotVarM);
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));

fprintf('Translation:\n');
disp(tranVec);
fprintf('Translation sd:\n');
disp(sqrt(tranVarM));

%get grid of transforms
fprintf('Generating transformation grid\n');
[TGrid, vTGrid] = GenTformGrid(tranVec, rotVec, tranVarM, rotVarM);

%convert transforms to tform matricies
fprintf('Generating final transformation matricies\n');
TMat = Grid2Mat(TGrid);