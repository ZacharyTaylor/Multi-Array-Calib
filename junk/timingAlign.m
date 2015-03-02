load('kittiCam1Data.mat');
load('kittiVelData.mat');
load('kittiNavData.mat');

%range = 100:1000;

cM = cam1Data.T_S1_Sk(:,4:6);
vM = velData.T_S1_Sk(:,4:6);
nM = navData.T_S1_Sk(:,4:6);

lower = [0.8,-0.05];
upper = [1.2, 0.05];

options = psooptimset('PopulationSize', 500,...
    'TolCon', 1e-1,...
    'StallGenLimit', 50,...
    'Generations', 200);

opts.LBounds = lower'; opts.UBounds = upper'; 
opts.SaveVariables = 'off';

samples = 1000;

A = cM;
B = vM;

windSize = ceil(0.01*length(A));
A = A-filter(ones(windSize,1)/windSize, 1, A);
windSize = ceil(0.01*length(B));
B = B-filter(ones(windSize,1)/windSize, 1, B);

%valid = and(iA < 0.5,iB < 0.5);
%iA = iA(valid);
%iB = iB(valid);

% path = dtw4(iA',iB');
% 
% hold off;
% plot(iA,'b-');
% hold on;
% plot(iB(path),'r-');

solution =pso(@(vec) matchInt(A,B, vec, samples), 2,[],[],[],[],lower,upper,[],options);
