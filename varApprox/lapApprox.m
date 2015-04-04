function [ val, var ] = IndVar(offset, func, varargin )
%INDVAR Finds variance of a function given input variance and assuming the
%   inputs have independent effect and the output is gaussian
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   offset- offset to apply when testing variance (0.01 usually works well)
%   func- handle to function to find variance of, note all inputs to this
%       function must be of type double
%
%   Requires the inputs to function func + their variance
%   For example if func is called by func(A,B)
%   IndVar would be called as IndVar(offset,@func,A,varA,B,varB)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   val- output of func
%   var- variance of function output
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%check inputs
validateattributes(offset,{'double'},{'scalar'});
validateattributes(func,{'function_handle'},{});
for i = 1:ceil(length(varargin)/2)
    validateattributes(varargin{2*i-1},{'double'},{});
    validateattributes(varargin{2*i},{'double'},{'size',size(varargin{2*i})});
end

base = varargin(1:2:end);
baseV = varargin(2:2:end);
val = func(base{:});

var = zeros(size(val));
for i = 1:length(base)
    for j = 1:length(base{i})
        %sample at offset
        base{i}(j) = base{i}(j) + offset;
        temp = func(base{:});
        base{i}(j) = base{i}(j) - offset;
        
        %combine variance
        if(isfinite(baseV{i}(j)))
            temp = baseV{i}(j).*((temp-val)./offset).^2;
        else
            %handle inf variance
            temp = temp-val;
            temp(temp~=0) = inf;
        end
        var = var + temp;
    end
end

end

