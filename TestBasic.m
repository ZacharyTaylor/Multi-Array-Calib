% Basic template for tests

%% user set variables

%number of scans to use
scansTimeRange = 100;
%scansTimeRange = 5:5:100;

%number of scans to combine in metric refine step
numScans = 20;

%number of times to perform test
reps = 10;

%samples
timeSamples = 100000;

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData('Shrimp','Vel','Cam1');

%gives results in terms of positions rather then coordinate frames
%less usful more intuative
%sensorData = InvertSensorData(sensorData);

%% fix timestamps
%[sensorData, offsets] = CorrectTimestamps(sensorData, timeSamples);

%% run calibration
    
%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 5, 0.001);

%find rotation
fprintf('Finding Rotation\n');
rotVec = RoughR(sData);
rotVec = OptR(sData, rotVec);
rotVarL = ErrorEstCR(sData, rotVec);
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));

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

%show what we are dealing with
%PlotData(sDataS,rotVec);

fprintf('Finding Translation\n');
tranVec = RoughT(sDataS, rotVec);
tranVec = OptT(sData, tranVec, rotVec, rotVarL);
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));

fprintf('Translation:\n');
disp(tranVec);
fprintf('Translation lower sd:\n');
disp(sqrt(tranVarL));
fprintf('Translation mid sd:\n');
disp(sqrt(tranVarM));

%get grid of transforms
fprintf('Generating transformation grid\n');
[TGrid, vTGrid] = GenTformGrid(tranVec, rotVec, tranVarM, rotVarM);

%refine transforms using metrics
fprintf('Refining transformations\n');
[TGridR, vTGridR] = MetricRefine(TGrid, vTGrid, sDataBase, numScans);

%correct for differences in grid
fprintf('Combining results\n');
[finalVec, finalVar] = OptGrid(TGridR, vTGridR);
