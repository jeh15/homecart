function PhDiff = phdiffmeasure(x, y)
% represent the signals as column-vectors
x = x(:);
y = y(:);
% remove the DC component of the signals
x = x - mean(x);
y = y - mean(y);
% signals length calculation
xlen = length(x);
ylen = length(y);
% windows generation
xwin = hanning(xlen, 'periodic');
ywin = hanning(ylen, 'periodic');
% perform fft on the signals
X = fft(x.*xwin); 
Y = fft(y.*ywin);
% fundamental frequency detection
[~, indx] = max(abs(X));
[~, indy] = max(abs(Y));
% phase difference estimation
PhDiff = angle(Y(indy)) - angle(X(indx));
end