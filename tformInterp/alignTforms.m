function [ tformAOut, tformBOut ] = alignTforms( tformA, tformB, samples, trim )
%ALIGNTFORMS Summary of this function goes here
%   Detailed explanation goes here

%get absolute angle magnitude
A = sqrt(sum(tformA.T_S1_Sk(:,4:6).^2,2));
B = sqrt(sum(tformB.T_S1_Sk(:,4:6).^2,2));

%smooth away accumulated bias
iA = A - smfilter(A,1);
iB = B - smfilter(B,1);

%get sampling spacing
rA = size(A,1)*(0:1/(samples-1):1);
rB = size(B,1)*(0:1/(samples-1):1);

%interpolate at samples
iA = interp1(1:size(iA,1),iA,rA','pchip');
iB = interp1(1:size(iB,1),iB,rB','pchip');

%use dtw to find mapping between samples
path1 = dtw4(iB',iA');
path2 = dtw4(iA',iB');

%find correct mapping
if(mean((iB - iA(path1)).^2) < mean((iA - iB(path2)).^2))
    path = path1';
    iA = iA(path);
    
    valid = (path ~= 1);
    %path = smfilter(path,1);
    %valid = and(valid, [diff(path);1] > 0);
    
    time = [path,(1:size(path,1))'];
    time = time(valid,:);
    
else
    path = path2';
    iB = iB(path);
    
    valid = (path ~= 1);
    %path = smfilter(path,1);
    %valid = and(valid, [diff(path);1] > 0);
    
    time = [path,(1:size(path,1))'];
    time = time(valid,:);
end

%find time samples occured at
time = [time(:,1), time(:,2)]/samples;

tA = interp1((1:size(time,1))'/size(time,1),time(:,1),(1:size(A,1))/size(A,1)','pchip')';
tB = interp1((1:size(time,1))'/size(time,1),time(:,2),(1:size(B,1))/size(B,1)','pchip')';

tOut = (trim:(1-2*trim)/(samples):(1-trim))';

%interpolate the required info
tformAOut.T_S1_Sk = interp1(tA,tformA.T_S1_Sk,tOut,'pchip');
tformBOut.T_S1_Sk = interp1(tB,tformB.T_S1_Sk,tOut,'pchip');

for i = 2:size(tformAOut.T_S1_Sk,1)
    tformAOut.T_Skm1_Sk(i-1,:) = tran2vec(vec2tran(tformAOut.T_S1_Sk(i-1,:)')\vec2tran(tformAOut.T_S1_Sk(i,:)'));
end

for i = 2:size(tformBOut.T_S1_Sk,1)
    tformBOut.T_Skm1_Sk(i-1,:) = tran2vec(vec2tran(tformBOut.T_S1_Sk(i-1,:)')\vec2tran(tformBOut.T_S1_Sk(i,:)'));
end

% tformAOut.T_Skm1_Sk = diff(tformAOut.T_S1_Sk);
% tformBOut.T_Skm1_Sk = diff(tformBOut.T_S1_Sk);

tformAOut.T_S1_Sk = tformAOut.T_S1_Sk(1:end-1,:);
tformBOut.T_S1_Sk = tformBOut.T_S1_Sk(1:end-1,:);

tformA.T_Cov_Skm1_Sk(1,:) = 0;
tformB.T_Cov_Skm1_Sk(1,:) = 0;

tformAOut.T_Cov_Skm1_Sk = cumsum(tformA.T_Cov_Skm1_Sk);
tformBOut.T_Cov_Skm1_Sk = cumsum(tformB.T_Cov_Skm1_Sk);

tformAOut.T_Cov_Skm1_Sk = interp1(tA,tformAOut.T_Cov_Skm1_Sk,tOut,'pchip');
tformBOut.T_Cov_Skm1_Sk = interp1(tB,tformBOut.T_Cov_Skm1_Sk,tOut,'pchip');

tformAOut.T_Cov_Skm1_Sk = diff(tformAOut.T_Cov_Skm1_Sk);
tformBOut.T_Cov_Skm1_Sk = diff(tformBOut.T_Cov_Skm1_Sk);

tformAOut.type = tformA.type;
tformBOut.type = tformB.type;

%remove invalid points
A = sqrt(sum(tformAOut.T_Skm1_Sk(:,4:6).^2,2));
B = sqrt(sum(tformBOut.T_Skm1_Sk(:,4:6).^2,2));

sA = sqrt(sqrt(sum(tformAOut.T_Cov_Skm1_Sk(:,4:6).^2,2)));
sB = sqrt(sqrt(sum(tformBOut.T_Cov_Skm1_Sk(:,4:6).^2,2)));

valid = (sA + sB) > abs(A-B);

tformAOut.T_S1_Sk = tformAOut.T_S1_Sk(valid,:);
tformAOut.T_Skm1_Sk = tformAOut.T_Skm1_Sk(valid,:);
tformAOut.T_Cov_Skm1_Sk = tformAOut.T_Cov_Skm1_Sk(valid,:);

tformBOut.T_S1_Sk = tformBOut.T_S1_Sk(valid,:);
tformBOut.T_Skm1_Sk = tformBOut.T_Skm1_Sk(valid,:);
tformBOut.T_Cov_Skm1_Sk = tformBOut.T_Cov_Skm1_Sk(valid,:);

end

