function [ calib ] = Test6UpCam( scansTimeRange )

try
    timeSamples = 10000;

    %% load sensor data
    CalibPath(true);
    %make sure to read in cameras last (due to issue with how I compensate for scale)
    sensorData = LoadSensorData('Shrimp','Cam1','Cam6');

    %% run calibration

    %get random contiguous scans to use
    sDataBase = RandTformTimes(sensorData, scansTimeRange);

%     %find time offset
%     [~, offsets, varOffsets] = CorrectTimestamps(sDataBase, timeSamples);
%     calib.tOff = offsets;
%     calib.tV = varOffsets;

    %evenly sample data
    sData = SampleData2(sDataBase);

    %remove uninformative data
    sData = RejectPoints(sData, 1000, 0.0000001);
    calib.n = length(sData{1}.time);

    %find rotation
    rotVec = RoughR(sData);
    calib.rotR = rotVec;
    calib.rotI = rotVec;
    rotVec = OptR(sData, rotVec);
    calib.rot = rotVec;
    rotVarL = ErrorEstCR(sData, rotVec);
    calib.rotCR = rotVarL;
    rotVarM = max(rotVarL,ErrorEstCR2(sData, rotVec));
    calib.rotVar = rotVarM;
catch
    calib.err = 1;
end
end