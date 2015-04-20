function [ calib ] = Test2VelCam( scansTimeRange, numScans, dataset )

%% user set variables

%samples
timeSamples = 10000;

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData(dataset,'Vel','Cam1');

%% fix timestamps
fprintf('Finding Timing Offsets\n');
[sensorData, offsets, varOffsets] = CorrectTimestamps(sensorData, timeSamples);
fprintf('Offsets:\n');
disp(offsets);
fprintf('Offset sd:\n');
disp(sqrt(varOffsets));

calib.timeOffset = offsets;
calib.timeVar = varOffsets;

%% run calibration
    
%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 100, 0.00001);

%find rotation
fprintf('Finding Rotation\n');
rotVec = RoughR(sData);
calib.rotR = rotVec;
rotVec = OptR(sData, rotVec);
calib.rot = rotVec;
rotVarL = ErrorEstCR(sData, rotVec);
calib.rotCR = rotVarL;
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
calib.rotVar = rotVarM;

fprintf('Rotation:\n');
disp(rotVec);
fprintf('Rotation lower sd:\n');
disp(sqrt(rotVarL));
fprintf('Rotation mid sd:\n');
disp(sqrt(rotVarM));

%find camera transformation scale (only used for RoughT, OptT does its
%own smarter/better thing
fprintf('Finding Camera Scale\n');
sDataS = EasyScale(sData, rotVec, rotVarL,zeros(2,3),ones(2,3));

fprintf('Finding Translation\n');
tranVec = RoughT(sDataS, rotVec);
calib.tranR = tranVec;
tranVec = OptT(sData, tranVec, rotVec, rotVarM);
calib.tran = tranVec;
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
calib.tranCR = tranVarL;
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));
calib.tranVar = tranVarM;

fprintf('Translation:\n');
disp(tranVec);
fprintf('Translation lower sd:\n');
disp(sqrt(tranVarL));
fprintf('Translation mid sd:\n');
disp(sqrt(tranVarM));

%get grid of transforms
fprintf('Generating transformation grid\n');
[TGrid, vTGrid] = GenTformGrid(tranVec, rotVec, tranVarM, rotVarM);
calib.TGrid = TGrid;
calib.vTGrid = vTGrid;

%refine transforms using metrics
fprintf('Refining transformations\n');
[TGridR, vTGridR] = MetricRefine(TGrid, vTGrid, sDataBase, numScans);
calib.TGridR = TGridR;
calib.vTGridR = vTGridR;

%correct for differences in grid
fprintf('Combining results\n');
[finalVec, finalVar] = OptGrid(TGridR, vTGridR);

calib.final = finalVec;
calib.finalVar = finalVar;

