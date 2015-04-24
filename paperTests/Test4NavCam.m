function [ calib ] = Test4NavCam( scansTimeRange, dataset )

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData(dataset,'Nav','Cam1');

%% run calibration
[sensorData, offsets, varOffsets] = CorrectTimestamps(sensorData, 10000);

%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 100, 0.00001);

sDataE = sData;
sDataE{1}.T_Var_Skm1_Sk(:) = 1;
sDataE{1}.T_Var_S1_Sk = cumsum(sDataE{1}.T_Var_Skm1_Sk);
sDataE{2}.T_Var_Skm1_Sk(:) = 1;
sDataE{2}.T_Var_S1_Sk = cumsum(sDataE{2}.T_Var_Skm1_Sk);

%find rotation
rotVec = RoughR(sDataE);
calib.rotR = rotVec;
rotVec = RoughR(sData);
rotVec = OptR(sData, rotVec);
calib.rot = rotVec;
rotVarL = ErrorEstCR(sData, rotVec);
calib.rotCR = rotVarL;
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
calib.rotVar = rotVarM;

estVecT = RoughTS(sData, rotVec);
calib.tranRT = estVecT;

tranVec = RoughT(sDataE, calib.rotR);
calib.tranR = tranVec;
tranVec = RoughT(sData, rotVec);
tranVec = OptT(sData, tranVec, rotVec, rotVarM);
calib.tran = tranVec;
tranVarL = ErrorEstCT(sData, tranVec, rotVec, rotVarL);
calib.tranCR = tranVarL;
tranVarM = max(tranVarL,ErrorEstCT2(sData, tranVec, rotVec, rotVarM));
calib.tranVar = tranVarM;

