function bytes = sizeof(arg)
%SIZEOF Return the size (in bytes) of a variable as stored on disk.
%   BYTES = SIZEOF(ARG) determines the size (in bytes) of a structure or other
%   (simple) variable.  It is designed to correspond to the C sizeof() operator.
%
%   Examples:
%     a = struct('a', int32(1), 'b', double(3));
%     bytes = sizeof(a); % 4 + 8 = 12 bytes
%
%   (c) 2007 Ryan M. Eustice
%            University of Michigan
%            eustice@umich.edu
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who          What
%    -----------     -------      -----------------------------
%    08-06-2007      RME          Created and written.

bytes = privateSizeOf(arg);

%===============================================================================
function bytes = privateSizeOf(arg)

switch (class(arg))
    case {'int8' 'uint8' 'char'}
        bytes = 1 * numel(arg);
    case {'int16' 'uint16'}
        bytes = 2 * numel(arg);
    case {'int32' 'uint32' 'single'}
        bytes = 4 * numel(arg);
    case {'int64' 'uint64' 'double'}
        bytes = 8 * numel(arg);
    case {'struct'}
        bytes = 0;
        fields = fieldnames(arg);
        for k=1:length(fields)
            bytes = bytes + privateSizeOf(arg.(fields{k}));
        end
    otherwise
        error('MATLAB:sizeof:badtype', 'Unknown type provided as an argument.');
end
