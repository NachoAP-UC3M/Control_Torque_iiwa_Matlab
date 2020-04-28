function[Inputs,Targets,TsInputs,TsTargets]=divide_data(trainRatio,x,y)
% 
% train_ratio=(number training samples/ number of samples in dataset)
% inputs   : training inputs
% targets  : training targets
% TsInputs : testing inputs
% TsTargets: testing targets

%%% calculate testing ratio
testRatio=1-trainRatio;
%%% get random dividing indexes 
[trainInd,valInd,testInd]=divideint(size(x,1),trainRatio,0,testRatio);
%%% divide  data
Inputs(1:length(trainInd),:)=x(trainInd,:); 
Targets(1:length(trainInd),:)=y(trainInd,:); 
TsInputs(1:length(testInd),:)=x(testInd,:); 
TsTargets(1:length(testInd),:)=y(testInd,:); 
% BERGHOUT Tarek
end