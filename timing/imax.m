function [z,I] = imax(x,varargin)
%IMAX    Interpolated largest elements in array.
%   Works as MAX except that interpolations are performed on three
%   consecutive points to provide a (likely) more realistic maximum.
%
%   For vectors, IMAX(X) is the largest interpolated element in X. For
%   matrices, IMAX(X) is a row vector containing the interpolated maximum
%   element from each column. For N-D arrays, IMAX(X) operates along the
%   first non-singleton dimension.
%
%   [Y,I] = IMAX(X) returns the locations of the interpolated maximum
%   values in a floating-point array I. For example, I = 1.5 means that the
%   interpolated maximum is located at the middle of the [1 2] segment. If
%   the values along the first non-singleton dimension contain more than
%   one interpolated maximal element, the index of the first one is
%   returned. 
%
%   [Y,I] = IMAX(X,[],DIM) operates along the dimension DIM. 
%
%   IMAX(X,Y) does not interpolate and makes the same as MAX(X,Y). It
%   returns an array the same size as X and Y with the largest elements
%   taken from X or Y. Either one can be a scalar.
%
%   When X is complex, IMAX does not interpolate and makes the same as MAX.
%   The maximum is computed using the magnitude MAX(ABS(X)). In the case of
%   equal magnitude elements, then the phase angle MAX(ANGLE(X)) is used.
%
%   NaN's are ignored when computing the interpolated maximum. When all
%   elements in X are NaN's, then the first one is returned as the
%   interpolated maximum.
%
%   Note:
%   ----
%   The interpolations are performed using parabolas on 3 consecutive
%   points.
%   
%   Example:
%   -------
%   x = [2 8 4;7 3 9];
%   imax(x,[],2)  % returns [8.05;9]
%
%   x = [0 64 96 96 64 0];
%   % compare MAX and IMAX
%   [z,I] = max(x)  % returns 96 and 3
%   [z,I] = imax(x) % returns 100 and 3.5
%
%   See also MAX, IMIN
%
%   -- Damien Garcia -- 2009/02

if nargin==2
    y = varargin{1};
elseif nargin==3;
    y = varargin{1};
    dim = varargin{2};
end

%% Use MAX if nargin=2 or if X is complex
if nargin==2 || ~isreal(x)
    try
        if nargout<2
            z = max(x,varargin{:});
        else
            [z,I] = max(x,varargin{:});
        end
        return
    catch
        rethrow(disp_err)
    end        
end

%% Shift dimension in order to work on rows
perm = []; nshifts = 0;
if nargin == 3 % imax(y,[],dim)
    if ~isempty(y)
        try
            % create an error...
            max(x,y,dim)
        catch
            % ...and display it
            rethrow(disp_err)
        end
    end
    y = x; clear x
    perm = [dim:max(ndims(y),dim) 1:dim-1];
    y = permute(y,perm);
else % imax(y)
    y = x; clear x
    [y,nshifts] = shiftdim(y);
end

%% Y must be a floating point array
if ~isfloat(y)
    error('Y must be a single or double array.')
end

%% Special case if Y is empty
if isempty(perm) && isequal(y,[])
  [z,I] = max(y);
  return;
end

%% Seek the extrema of the fitting parabolas
warn0 = warning('query','MATLAB:divideByZero');
warning('off','MATLAB:divideByZero')

y0 = y;
% Seek the extrema of the parabola P(x) defined by 3 consecutive points
% so that P([-1 0 1]) = [y1 y2 y3]
y1 = y(1:end-2,:);
y2 = y(2:end-1,:);
y3 = y(3:end,:);
y(2:end-1,:) = -(y3-y1).^2/8./(y1+y3-2*y2)+y2;

% ... and their corresponding normalized locations
I = zeros(size(y));
I(2:end-1,:) = (y1-y3)./(2*y1+2*y3-4*y2);

warning(warn0.state,'MATLAB:divideByZero')

%% Interpolated maxima...
% if locations are not within [-1,1], calculated extrema are ignored
test = I<=-1|I>=1|isnan(y);
y(test) = y0(test);
try
    [z,I0] = max(y);
catch
    rethrow(disp_err)
end

% ...and their corresponding location
siz = size(y);
I = I(sub2ind(siz,I0(:)',1:length(I0)))+I0(:)';

%% Resizing
siz(1) = 1;
z = reshape(z,[ones(1,nshifts),siz]);
if ~isempty(perm), z = ipermute(z,perm); end

I = reshape(I,size(z));

%% Replace the standard error messages from MAX
function err = disp_err
err = lasterror;
err.message = regexprep(err.message,'max','imax','preservecase');
