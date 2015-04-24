function [ calib ] = Test5Multi( scansTimeRange, dataset )

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData(dataset,'Vel','Nav','Cam1','Cam2');

%% run calibration
    
%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 100, 0.00001);

%find rotation
rotVec = RoughR(sData);
calib.rotR = rotVec;
for i = 2:4
    rotVec([1,i],:) = OptR(sData([1,i]), rotVec([1,i],:));
end
calib.rotI = rotVec;
rotVec = OptR(sData, rotVec);
calib.rot = rotVec;
rotVarL = ErrorEstCR(sData, rotVec);
calib.rotCR = rotVarL;
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
calib.rotVar = rotVarM;

tranVec = RoughT(sData, rotVec);
calib.tranR = tranVec;
for i = 2:4
    tranVec([1,i],:) = OptT(sData([1,i]), tranVec([1,i],:), rotVec([1,i],:), rotVarM([1,i],:));
end
calib.tranI = tranVec;
tranVec = OptT(sData, tranVec, rotVec, rotVarM);
calib.tran = tranVec;
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
calib.tranCR = tranVarL;
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));
calib.tranVar = tranVarM;

