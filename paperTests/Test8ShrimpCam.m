function [ calib ] = Test8ShrimpCam( scansTimeRange )

%% load sensor data
CalibPath(true);
%make sure to read in cameras last (due to issue with how I compensate for scale)
sensorData = LoadSensorData('Shrimp','Cam1','Cam2','Cam3','Cam4','Cam5');

%% run calibration

%get random contiguous scans to use
sDataBase = RandTformTimes(sensorData, scansTimeRange);

%evenly sample data
sData = SampleData2(sDataBase);

%remove uninformative data
sData = RejectPoints(sData, 10, 0.0000001);

%find rotation
rotVec = RoughR(sData);
calib.rotR = rotVec;
for i = 2:5
    rotVec([1,i],:) = OptR(sData([1,i]), rotVec([1,i],:));
end
calib.rotI = rotVec;
rotVec = OptR(sData, rotVec);
calib.rot = rotVec;
rotVarL = ErrorEstCR(sData, rotVec);
calib.rotCR = rotVarL;
rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
calib.rotVar = rotVarM;

end