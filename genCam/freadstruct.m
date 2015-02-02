function S = freadstruct(filename,Proto,varargin)
%FREADSTRUCT  Read a structured binary data file.
%   S = FREADSTRUCT(FILENAME,PROTO) reads the binary file FILENAME and
%   parses it according to the prototype C-structure PROTO, returning the
%   data in the structure S.  Multiple data records are organized
%   along the fist dim of the arrays in S.
%
%   S = FREADSTRUCT(FILENAME,PROTO,'PARAM',VALUE,...) same as above, but
%   accepts optional PARAM/VALUE pairs:
%      'nRecords' % number of records to return, default [inf]
%      'offset'   % header offset in bytes, default [0]
%
%   Note: If any field of the PROTO is of greater dimension than a vector, then
%   the data for that field will be returned along the column dimension of the 
%   array.  If you want to preserve the original dimensionality, then it is
%   recommended that you use MEMMAPSTRUCT instead.
%
%   Example:
%   % Suppose that you had data organized in a C-stucture like such:
%   typedef struct {
%      int i;
%      int j;
%   } __attribute__((packed)) Foo;
%
%   typedef struct {
%      int i;
%      uint ui;
%      double d[3];
%      char c;
%      Foo F;
%   } __attribute__((packed)) myStruct;
%
%   % and that you had created 10 records of it and written it to disk
%   % as a binary file, for example, using fwrite().
%   myStruct S[10] = {0};
%   FILE* fid = fopen('test.dat','w');
%   fwrite(S,sizeof(myStruct),10,fid);
%   fclose(fid);
%
%   % To read the data records back into Matlab simply do the following:
%   % 1) Create a prototype of the C-structure in Matlab matching the C
%   %    data structure.
%   P.i      = int32(0);
%   P.ui     = uint32(0);
%   P.d      = double(zeros(3,1));
%   P.c      = char(0);
%   P.Foo.i  = int32(0);
%   P.Foo.ui = uint32(0);
%
%   % 2) Call freadstruct with the prototype structure
%   S = freadstruct('test.dat',P);
%
%   % S will be a Matlab struct with the records organized by rows, i.e.
%   S.i      [10 x 1] of int32
%   S.ui     [10 x 1] of uint32
%   S.d      [10 x 3] of double
%   S.c      [10 x 1] of char
%   S.Foo.i  [10 x 1] of int32
%   S.Foo.ui [10 x 1] of uint32
%
%   See also memmapstruct, sizeof.
%
%   (c) 2007 Ryan M. Eustice
%            University of Michigan
%            eustice@umich.edu
%
%--------------------------------------------------------------------------
%    History:
%    Date            Who          What
%    -----------     -------      -----------------------------------------
%    08-06-2007      RME          Created and written.

p = inputParser(); % create an instance of the inputParser class
% define agruments
p.addRequired('filename',@ischar);
p.addRequired('Proto',@isstruct);
p.addParamValue('offset',0,@isnumeric);
p.addParamValue('nRecords',inf,@isnumeric);
% parse args
p.parse(filename, Proto, varargin{:});
nRecords = p.Results.nRecords;
offset = p.Results.offset;

fid = fopen(filename,'r');
if fid < 0
    error('Invalid filename arg.');
end
fseek(fid,offset,'bof');
S = readRecords(fid,nRecords,Proto,sizeof(Proto));
fclose(fid);

%==========================================================================
function D = readRecords(fid,nRecords,S,s) % recursive fcn
fields = fieldnames(S);
for k=1:length(fields)
    field = fields{k};
    if isstruct(S.(field))
        D.(field) = readRecords(fid,nRecords,S.(field),s);
    else
        %A = FREAD(FID,SIZE,PRECISION,SKIP)
        siz = [numel(S.(field)) nRecords];
        precision = sprintf('%d*%s',numel(S.(field)),class(S.(field)));
        bytes = sizeof(S.(field));
        skip = s-bytes;
        
        offset = ftell(fid);
        D.(field) = fread(fid,siz,precision,skip)';% [nRecords x numel(S.(field))]
        fseek(fid,offset+bytes,'bof');% move offset past start of current data records
    end
end
