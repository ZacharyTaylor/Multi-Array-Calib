function cross_corr = xcorr2_fast(T,A,RetSizeOpt)
% function cross_corr = xcorr2_fast(T,A,RetSizeOpt)
% image correlation that switches between freq and spatial domains
% automatically for speed.
% T - Template or Image 
% A - Template or Image
% RetSizeOpt - 'valid', 'same','full' - see doc on conv2 for details
%   default is 'full' to be compatible with the original matlab xcorr_fast
%   that I found buried in another mathworks function somewhere.
% cross_corr - response image - size depends on RetSizeOpt
if(~exist('RetSizeOpt','var'))
    RetSizeOpt = 'full';
end
if(numel(T) > numel(A))
    Tmp = A;
    A=T;
    T=Tmp;
    Tmp=[];
end
T_size = size(T);
A_size = size(A);
outsize = A_size + T_size - 1;

% figure out when to use spatial domain vs. freq domain
conv_time = time_conv2(T_size,A_size); % 1 conv2
fft_time = 3*time_fft2(outsize); % 2 fft2 + 1 ifft2

if (conv_time < fft_time)
    cross_corr = conv2(A,rot90(T,2),RetSizeOpt);
    RetSizeOpt='';
else
    cross_corr = freqxcorr(T,A,outsize);
end
switch(lower(RetSizeOpt))
    case 'same'
        TmpltRadius=floor(T_size/2);
        cross_corr = cross_corr(TmpltRadius(1)+1:TmpltRadius(1)+A_size(1),TmpltRadius(2)+1:TmpltRadius(2)+A_size(2));
    case 'valid'
        OutImgSize = A_size - T_size + 1;
        cross_corr = cross_corr(T_size(1):T_size(1)+OutImgSize(1)-1,T_size(2):T_size(2)+OutImgSize(2)-1);
end
%-------------------------------
% Function  freqxcorr
%
function xcorr_ab = freqxcorr(a,b,outsize)
  
% calculate correlation in frequency domain
Fa = fft2(rot90(a,2),outsize(1),outsize(2));
Fb = fft2(b,outsize(1),outsize(2));
xcorr_ab = real(ifft2(Fa .* Fb));

function time = time_conv2(obssize,refsize)         
K = 2.7e-8; 
% convolution time = K*prod(obssize)*prod(refsize)
time =  K*prod(obssize)*prod(refsize);


%-------------------------------
% Function  time_fft2
%
function time = time_fft2(outsize)

% time a frequency domain convolution by timing two one-dimensional ffts

R = outsize(1);
S = outsize(2);

% Tr = time_fft(R);
% K_fft = Tr/(R*log(R)); 

% K_fft was empirically calculated by the 2 commented-out lines above.
K_fft = 3.3e-7; 
Tr = K_fft*R*log(R);

if S==R
    Ts = Tr;
else
%    Ts = time_fft(S);  % uncomment to estimate explicitly
   Ts = K_fft*S*log(S); 
end

time = S*Tr + R*Ts;

