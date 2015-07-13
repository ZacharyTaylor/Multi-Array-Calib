function [ calib ] = Test9Full( scansTimeRange, dataset )

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData(dataset,'Vel','Cam1','Cam2','Cam3','Cam4');

%% run calibration
    
%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

[sDataBase, ~, ~] = CorrectTimestamps(sensorData, 10000);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 10, 0.00001);

%find rotation
rotVec = RoughR(sData);
rotVec = OptR(sData, rotVec);
calib.rot = rotVec;
rotVarL = ErrorEstCR(sData, rotVec);
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
calib.rotVar = rotVarM;

tranVec = RoughT(sData, rotVec);
calib.tranR = tranVec;
tranVec = OptT(sData, tranVec, rotVec, rotVarM);
calib.tran = tranVec;
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));
calib.tranVar = tranVarM;

%get grid of transforms
fprintf('Generating transformation grid\n');
[TGrid, vTGrid] = GenTformGrid(tranVec, rotVec, tranVarM, rotVarM);

numScans = 25;

%refine transforms using metrics
[TGridR, vTGridR] = MetricRefine(TGrid, vTGrid, sDataBase, numScans, 'motion');
calib.ref = TGridR;
calib.vref = vTGridR;



