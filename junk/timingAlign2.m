load('kittiCam1Data.mat');
load('kittiVelData.mat');
load('kittiNavData.mat');

%range = 100:1000;

cM = sqrt(sum(cam1Data.T_S1_Sk(:,4:6).^2,2));
vM = sqrt(sum(velData.T_S1_Sk(:,4:6).^2,2));
nM = sqrt(sum(navData.T_S1_Sk(:,4:6).^2,2));


samples = 1000;

A = vM(:);
B = cM(:);

rA = length(A)*(0:1/(samples-1):1);
rB = length(B)*(0:1/(samples-1):1);

iA = A - smfilter(A,1);
iB = B - smfilter(B,1);


iA = interp1(1:size(iA,1),iA,rA','pchip');
iB = interp1(1:size(iB,1),iB,rB','pchip');

%A = diff(A);
%B = diff(B);

iA = sqrt(sum(iA.^2,2));
iB = sqrt(sum(iB.^2,2));

path1 = dtw4(iB',iA');
path2 = dtw4(iA',iB');

if(mean((iB - iA(path1)).^2) < mean((iA - iB(path2)).^2))
    path = path1';
    iA = iA(path);
    
    valid = (path ~= 1);
    path = smfilter(path,1);
    
    time = [path,(1:size(path,1))'];
    time = time(valid,:);
    
else
    path = path2';
    iB = iB(path);
    
    valid = (path ~= 1);
    wind = gausswin(ceil(0.01*length(path)));
    wind = wind./sum(wind);
    %path = conv(path,wind,'valid');
    
    time = [(1:size(path,1))',path];
    time = time(valid,:);
end

time = [time(:,1)*length(A), time(:,2)*length(B)]/samples;

tA = interp1((1:size(time,1))'/size(time,1),time(:,1),(1:size(A,1))/size(A,1)','pchip')';
tB = interp1((1:size(time,1))'/size(time,1),time(:,2),(1:size(B,1))/size(B,1)','pchip')';

%windSize = ceil(0.01*length(path));
%path = filter(ones(windSize,1)/windSize, 1, path);

hold off;
plot(iA,'b-');
hold on;
plot(iB,'r-');


