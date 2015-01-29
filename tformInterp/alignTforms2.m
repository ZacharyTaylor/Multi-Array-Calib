function [ out ] = alignTforms2( sensorData, samples )
%ALIGNTFORMS Summary of this function goes here
%   Detailed explanation goes here

T = cell(length(sensorData),1);
C = cell(length(sensorData),1);
t = cell(length(sensorData),1);
for i = 1:length(sensorData)

    [~,idx] = sort(sensorData{i}.time);
    valid = diff(double(sensorData{i}.time(idx))) > 0;
    idx = idx(valid);
    t{i} = double(sensorData{i}.time(idx));
    
    %get absolute angle magnitude
    T{i} = sqrt(sum(sensorData{i}.T_S1_Sk(idx,4:6).^2,2));
    C{i} = sum(sensorData{i}.T_Cov_Skm1_Sk(idx,4:6),2);
    
    %smooth away accumulated bias
    %T{i} = T{i} - smfilter(T{i},10);
end
offset = 10*ones(length(sensorData)-1,1)';

% options = psooptimset('PopulationSize', 100,...
%     'TolCon', 1e-1,...
%     'StallGenLimit', 50,...
% ...    'PlotFcns',{@AlignPlotSwarm},...
%     'Generations', 200);
% 
% lower = offset - 10000000;
% upper = offset + 10000000;

%out = simulannealbnd(@(offset) findErr(offset,T,C,t,samples),offset,lower,upper);
%out = pso(@(offset) findErr(offset,T,C,t,samples), length(offset),[],[],[],[],lower,upper,[],options);
out = fminsearch(@(offset) findErr(offset,T,C,t,samples),offset);


end

