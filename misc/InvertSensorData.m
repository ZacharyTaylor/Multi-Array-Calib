function [ sensorData ] = InvertSensorData( sensorData )

if(iscell(sensorData))
    for i=1:length(sensorData)
        sensorData{i} = InvData(sensorData{i});
    end
elseif(isstruct(sensorData))
    sensorData = InvData(sensorData);
else
    error('sensorData must be a struct of cell of structs');
end

end
    

function [ sData ] = InvData( sData )

for i = 2:size(sData.T_Skm1_Sk,1)
    sData.T_Skm1_Sk(i,:) = T2V(inv(V2T(sData.T_Skm1_Sk(i,:))));
    sData.T_S1_Sk(i,:) = T2V(V2T(sData.T_S1_Sk(i-1,:))*V2T(sData.T_Skm1_Sk(i,:)));
end

end