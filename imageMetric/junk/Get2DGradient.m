function [ x, y ] = Get2DGradient( in, tform )
%GETGRADIENT Calculates an estimate of the gradients at each point

in = double(in);

out = in(:,1:4);
out(:,4) = 1;
out = (tform*out')'; 

vals = in(:,4);

%project points onto sphere
sphere = zeros(size(out,1),2);
sphere(:,1) = atan2(out(:,1), out(:,3));
sphere(:,2) = atan(out(:,2)./ sqrt(out(:,1).^2 + out(:,3).^2));

kdTree = KDTreeSearcher(sphere(:,1:2),'distance','euclidean');

%get nearest neighbours
idx = knnsearch(kdTree,kdTree.X,'k',9);

%remove self
idx = idx(:,2:end);

dVals = repmat(vals,1,8);
dVals(:) = dVals(:) - vals(idx(:));

xLocs = kdTree.X(:,1);
dxLocs = repmat(xLocs,1,8);
dxLocs(:) = dxLocs(:) - xLocs(idx(:));

yLocs = kdTree.X(:,2);
dyLocs = repmat(yLocs,1,8);
dyLocs(:) = dyLocs(:) - yLocs(idx(:));

dLocs = sqrt(dxLocs.^2 + dyLocs.^2);
phase = atan2(dyLocs,dxLocs);

dVals = dVals./dLocs;
dxLocs = sum(dVals.*cos(phase),2)/8;
dyLocs = sum(dVals.*sin(phase),2)/8;

mag = sqrt(dxLocs.^2 + dyLocs.^2);
phase = 180*atan2(dxLocs,dyLocs)/pi;

mag = MyHistEq(mag);

x = mag.*cos(phase);
y = mag.*sin(phase);

end

