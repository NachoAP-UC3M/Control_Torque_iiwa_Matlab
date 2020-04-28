function dataout = scaledata(datain,minval,maxval)
% Program to scale the values of a matrix from a user specified minimum to a user specified maximum
dataout = datain - min(datain(:));
dataout = (dataout/range(dataout(:)))*(maxval-minval);
dataout = dataout + minval;