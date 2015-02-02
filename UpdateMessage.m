function [] = UpdateMessage( message, varargin )
%UPDATEMESSAGE Removes the last message and prints a new one (functions the
%   same as printf when passing inputs
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   Message- the message to print (operates the same as fprintf)
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

persistent messLength
if(isempty(messLength))
    fprintf(' ');
    messLength = 0;
end

validateattributes(message,{'char'},{'vector'});

%remove last message
%fprintf(repmat('\b', 1, messLength));
message = [message '\n'];

%setup new message
toPrint = cell(length(varargin) + 2,1);
toPrint{1} = 'sprintf(message';
for i = 1:length(varargin)
    toPrint{i+1} = [', varargin{' num2str(i) '}'];
end
toPrint{end} = ');';

%print message
toPrint = eval(cell2mat(toPrint'));
fprintf(toPrint);

%record length
messLength = length(toPrint);

