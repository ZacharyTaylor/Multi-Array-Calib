function path = dtw4(key_t,sig_t)
% path = dtw(key,input)
%
% dtw takes in a key signal, and an input signal and returns a path that
% maps the indices of the input to the indices of the key.  For example, if
% the key had 5 (1..N) elements and input had 6 (1..M), with 
% path = [1 2 [3,4] 5 6], the new input would have five elements with 
%
%     new_input(1) = input(1)
%     new_input(2) = input(2)
%     new_input(3) = some function of input(3) and input(4) - avg?
%     new_input(4) = input(5)
%     new_input(5) = input(6)
%
% The following restriction holds: w(1) = 1, w(N) = M
%
% ELEC 301 Project - "Mars Lander the Theologian", aka, "Speaker Verification"
% Sara MacAlpine, Aamir Virani, Nipul Bharani, JP Slavinsky
% WRC (OCEE) Class of 2001 - Fall '99

%global N M ldm key sig   % Declare as global so don't have to pass them around
key = key_t;
sig = sig_t;
N = length(key);
M = length(sig);

% Compute local distance matrix (ldm) - uses a Matlab trick
K = key(ones(size(sig)),:);
S = sig';
S = S(:,ones(size(key)));
ldm = abs(K-S).^2;

% Compute accumulative distance matrix (adm)
%global adm
adm = zeros(M,N);
adm(:,1) = ldm(:,1);
for n = 2:N,
  col = adm(:,n-1);
  most = max(col) + 1;
  col = [most; most; col];
  new = [col(1:M), col(2:M+1), col(3:M+2)];
  minimums = min(new,[],2);
  adm(:,n) = ldm(:,n) + minimums;
end

% Derive best path, w, starting at w(N) = M and working back to w(1) = 1
% w(n) will be the indices of sig to use to shrink/stretch it to key length.
%      new_sig(1:N) = sig(w(n));
% Now using a modified Itakura algorithm -- can't go flat for 2 consecutive
% times and can't use big jump 2 consecutive times

w(N) = M;
w(1) = 1;
r = M;            % r = row
flatflag = 0;
jumpflag = 0;
for c = N-1:-1:2, % c = column
  % develop choices based on rows above current posn
  if (r >= 3) & ~jumpflag
    choices = [adm(r,c) adm(r-1,c) adm(r-2,c)];
  elseif (r >= 3) & jumpflag
    choices = [adm(r,c) adm(r-1,c)];
    jumpflag = 0;
  elseif (r == 2)
    choices = [adm(r,c) adm(r-1,c)];
  elseif (r == 1)
    choices = [adm(r,c)];
  end
  % find posn up of smallest change remove "flat choice" if just did flat
  % unless we're on the last row.
  if flatflag & (r ~= 1)
    choices = choices(2:length(choices));
    posn = max(find(choices==min(choices)));
    if posn == length(choices)  % we're going to use jump
      jumpflag == 1;
    end
    flatflag = 0;
  else
    posn = max(find(choices==min(choices))) - 1;
    if posn == 2
      jumpflag = 1;
    end
    if posn == 0
      flatflag = 1;
    end
  end
  % save path
  r = r - posn;
  w(c) = r;
end

% return path
path = w;
