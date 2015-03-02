function [] = plotCam6Shrimp()
%PLOTCAM6SHRIMP Summary of this function goes here
%   Detailed explanation goes here

load('shrimpNavData.mat');
load('shrimpCam6Data.mat');

Tc = [0.000796323127265841,0.999997687711085,-0.00199760906076025,0.00740000000000000;0.999995190893873,-0.000790330440132239,0.00299892762861593,0.000999835477429291;0.00299734192298062,-0.00199998756947394,-0.999993507974486,-1.31280000265337;0,0,0,1];
T = Tc;

for i = 2:size(cam6Data.T_S1_Sk,1)
    [~,j] = min(abs(double(cam6Data.time(i)) - double(navData.time)));
    [~,k] = min(abs(double(cam6Data.time(i-1)) - double(navData.time)));
    
    mag = V2T(navData.T_S1_Sk(j,:))\V2T(navData.T_S1_Sk(k,:));
    mag = norm(mag(1:3,4));
    
    cam6Data.T_Skm1_Sk(i,1:3) = mag.*cam6Data.T_Skm1_Sk(i,1:3); 
    cam6Data.T_S1_Sk(i,:) =T2V(V2T(cam6Data.T_S1_Sk(i-1,:))*V2T(cam6Data.T_Skm1_Sk(i,:)));
end

for i = 2:size(cam6Data.T_S1_Sk,1)
    cam6Data.T_S1_Sk(i,:) =T2V(T*V2T(cam6Data.T_S1_Sk(i,:)));
end

close all;
figure;
hold on;
plot3(cam6Data.T_S1_Sk(:,1),cam6Data.T_S1_Sk(:,2),cam6Data.T_S1_Sk(:,3),'b-');
plot3(navData.T_S1_Sk(:,1),navData.T_S1_Sk(:,2),navData.T_S1_Sk(:,3),'r-');
axis equal;

figure;
Mag = sqrt(sum(cam6Data.T_Skm1_Sk(:,4:6).^2,2));
Mag2 = sqrt(sum(navData.T_Skm1_Sk(:,4:6).^2,2));
Sig = sqrt(sum(cam6Data.T_Cov_Skm1_Sk(:,4:6),2));
Sig(~isfinite(Sig)) = 100;
boundedline(cam6Data.time,Mag(:,1),Sig(:,1),'b-');
hold on;
plot(navData.time,Mag2,'r-');


end

