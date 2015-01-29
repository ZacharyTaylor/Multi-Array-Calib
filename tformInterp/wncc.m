function TwnccI = wncc(I,T,W,RetSizeOptStr)
% function TwnccI = wncc(I,T,W,RetSizeOptStr)
% Weighted Normalized Cross Correlation (Image Template matching)
% INPUTS
%   I - Image - single or doubles - should really use doubles
%   T - Template - single or doubles - odd sized matrix (i.e. all(rem(size(T),2)==1))
%   W - optional - single or doubles.
%       Matrix of weights; one for each Template element - 
%       i.e. Wij is the weight on Tij (==>size(W)=size(T)) where Wij >= 0.
%       Not supplying W or giving it as [] is the same as giving W 
%       with all equal numbers (W= ones(size(T))/numel(T)) ==> wncc == ncc.
%       W does not have to be normalized (sums  to 1) as that's done
%       internally
%   RetSizeOptString - optional with default of 'same' - allowable values:
%       'same' - result image (TwnccI) is the same size as image I but the
%           edge of the result, to a radius of floor(size(T)/2), is not really
%           valid is the template in that area can't fit in the image
%       'valid' - The result with the above mentioned edge area removed.
%
% OUTPUTS
%   TwnccI - Result/Response Image - single or doubles depending on I,T,W
%       if RetSizeOptString == 'same' then TwnccI is the same size as I
%       if RetSizeOptString == 'valid' then size(TwnccI) =
%           size(I)-size(T)+1
% NOTES
%   The time dominant operation involved is 3 convolutions (really image
%   correlation).  wncc uses a form that automatically switches between
%   doing convolution in the frequency or spatial domains depending upon
%   its estimate of which is faster (upshot - this makes the user of large 
%   templates feasible).
%   
% Algorithm - given a vector x,y NCC can be expressed as:
%   NCC = cov(x,y)/sqrt(cov(x,x)*cov(y,y))
%   where cov(a,b) = mean((a-mean(a)) .* (b-mean(b)))
%   given a vector of weights, w, the weighted NCC is likewise defined as:
%   WNCC = wcov(x,y,w)/sqrt(wcov(x,x,w)*wcov(y,y,w))
%   where wcov(a,b,w) = mean(w .* (a-wmean(a,w)) .* (b-wmean(b,w)))
%   where wmean(v,w) is the weighted average of v. (note: if
%           sum(w)==1==>wmean(v,w)=sum(v.*w)
%       i.e. The contribution of each pair of elements, (a(i), b(i)), is
%       weighted by the weight wi and the (weighted) means are computed w/r to the
%       weights vs the standard mean.
%   For images, x corresponds to each template sized sliding
%       filter/window/patch of the image and y for the template.
%   Note that since mean(x) = sum(x)/length(x) that length(x) cancels out
%       in NCC or WNCC so cov(x,y) or wcovx(w,y,w) can use sum rather than
%       mean (as is done here)
%
%   Author: Andrew Diamond, adiamondcsi@gmail.com
 
%  Create weights if not supplied.  Normalize (i.e. sum to 1)
if(~exist('W','var') || isempty(W))
    Wn = ones(size(T)) / numel(T);
else
    Wn = W ./ sum(W(:)); 
end
if(~exist('RetSizeOptStr','var') || isempty(RetSizeOptStr))
    RetSizeOptStr = 'same';
end
% mIW is the wmean image (i.e. each of its pixel's is a wmean for the window 
% centered at that pixel).  
mIW=xcorr2_fast(I,Wn,'same'); % weighted means of all windows of I (i.e. wmean(x))
mTwn = sum(Wn(:) .* T(:)); % weighted mean of T
mTWn=T-mTwn; % T - its weighted mean
% covariance calculations:
% wCovxy- wcov(w,x,y) = mean(w .* (x-wmean(x)) .* (y-wmean(y)))
%   == mean((x-wmean(x)) .* (w .* (y-wmean(y))) (where here x represents a
%   single window of I in vector form and y,w the template and its weights
%   in vector form)
%  so (w .* (y-wmean(y))) is the weighted "normalized" template, Wn .* mTWn
%   , call it: WnmTWn
%  wcov(Wn,x,y) = mean((x-wmean(x)) * WnmTWn) = mean(x .* WnmTWn-wmean(x)*WnmTWn)
%   note that WnmTWn is static across all windows of the image (it's
%   the filter) 
WnmTWn=Wn .* mTWn;
wCovxy=xcorr2_fast(I,WnmTWn,'same') - mIW .* sum(WnmTWn(:));

% weighted version of the well known equation:
% variance(X) = E((X-E(X)).^2) = E(X .^2)-E(X).^2
wCovxx = xcorr2_fast(I .^ 2 , Wn ,'same') -  mIW .^ 2; 

% wCovyy = mean(w .* (y-wmean(y)) .^ 2) - this quantity corresponds the
%   filter/template and so is static and so it can be computed directly (i.e. no
%   convolution needed)
wCovyy = sum(mTWn(:) .^ 2 .* Wn(:));

Denom = sqrt(max(0,wCovxx .* wCovyy));

% The mathematical answer as this point is wCovxy/Denom but Denom could be
% really tiny due to round off errors and such and so the code below sets
% the response for elements with very small Denom values to 0;
TwnccI = zeros(size(Denom));
tol = 1000*eps( max(abs(Denom(:))) ); % should get a more formal tol
i_nonzero = find(Denom > tol);
TwnccI(i_nonzero) = wCovxy(i_nonzero) ./ Denom(i_nonzero);

% If the user selected a 'valid' sized output response image, chop
% off the naughty bits
switch(lower(RetSizeOptStr))
    case 'valid'
        T_size = size(T);
        A_size = size(I);
        OutImgSize = A_size - T_size + 1;
        TmpltRadius=floor(T_size/2);
        TwnccI = TwnccI(TmpltRadius(1)+1:TmpltRadius(1)+OutImgSize(1),TmpltRadius(2)+1:TmpltRadius(2)+OutImgSize(2));
end
    
        


