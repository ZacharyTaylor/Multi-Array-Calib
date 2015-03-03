function [] = ParSave(fileName, data, variableName)
%PARSAVE Allows threads to save variables without waiting
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   fileName- file to save variable to
%   data- holds the data being saved
%   variableName- name the data will take when its loaded into matlab
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   I found this code somewhere on stackoverflow (I forget where)
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

validateattributes(fileName,{'char'},{'vector'});
validateattributes(variableName,{'char'},{'vector'});

eval(sprintf('%s = data;', variableName));
eval(sprintf('save(fileName, ''%s'');',variableName));
end

