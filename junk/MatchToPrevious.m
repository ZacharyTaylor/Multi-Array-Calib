function [ points, prevPoints, matches ] = MatchToPrevious( image, mask )
%MATCH_TO_PREVIOUS Uses SURF to match points in the input image to the
%previous one
%
%   Inputs:
%   Image - the image to match to the previous one
%
%   Outputs:
%   points - n by 2 array of corner points in current image [x, y]
%   prevPoints - n by 2 array of corner points in previous image [x, y]
%   matches- index of matching points

%setup persistant storage of features
persistent prevSurfPoints;
persistent prevSurfVar;
persistent prevSurfFeats;


%if empty arguments clear everything
if(nargin == 0)
    clear prevSurfPoints;
    clear prevSurfVar;
    clear prevSurfFeats;
    return;
end

validateattributes(image,{'numeric'},{'2d'});

%ensure image is of type double
image = double(image);

%ensure range is 0 to 1
image = image - min(image(:));
image = image ./ max(image(:));

%detect surf points in images
surfPoints = detectSURFFeatures(image);

%reject masked points
notMasked = mask(round(surfPoints.Location(:,2))+size(mask,1)*(round(surfPoints.Location(:,1))-1)) ~= 0;
surfPoints = surfPoints(notMasked);

%find variance of surf points
varPoints = EstPointVar(image,surfPoints.Location);

%reject infinite variances
valid = any(~isfinite(varPoints),2);
varPoints = varPoints(valid);
surfPoints = surfPoints(valid);

%find features for valid surf points
[surfFeats, surfPoints] = extractFeatures(image, surfPoints);

%extract corner points
points = [surfPoints.Location, varPoints];
    
%skip matching if no previous points exist
if(isempty(prevSurfPoints))
    prevPoints = zeros(0,6);
    matches = zeros(0,2);
else
    prevPoints = [prevSurfPoints.Location, prevSurfVar];
    %find matching features and get index
    matches = matchFeatures(prevSurfFeats, surfFeats, 'Prenormalized', true);
end

%store surf points and features
prevSurfPoints = surfPoints;
prevSurfVar = varPoints;
prevSurfFeats = surfFeats;

end