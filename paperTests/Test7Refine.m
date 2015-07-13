function [ calib ] = Test7Refine( scansTimeRange, dataset )

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData(dataset,'Vel','Cam1');

%% run calibration
    
%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 100, 0.00001);

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

noBase = TGrid;
noSol = vTGrid;
noSol{1,2}(:) = 10000;
numScans = 25;

%refine transforms using metrics
fprintf('Refining transformations\n');
[TGridR, ~] = MetricRefine(TGrid, vTGrid, sDataBase, numScans, 'nmi');
calib.NMIConRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(noBase, noSol, sDataBase, numScans, 'nmi');
calib.NMIRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(TGrid, vTGrid, sDataBase, numScans, 'lev');
calib.LevConRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(noBase, noSol, sDataBase, numScans, 'lev');
calib.LevRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(TGrid, vTGrid, sDataBase, numScans, 'gom');
calib.GOMconRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(noBase, noSol, sDataBase, numScans, 'gom');
calib.GOMRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(TGrid, vTGrid, sDataBase, numScans, 'motion');
calib.MotionConRef = S2V(TGridR{1,2});

[TGridR, ~] = MetricRefine(noBase, noSol, sDataBase, numScans, 'motion');
calib.MotionRef = S2V(TGridR{1,2});


