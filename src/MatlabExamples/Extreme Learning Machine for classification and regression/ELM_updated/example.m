%%
%  Machine Learning vs ANN: https://www.quora.com/What-is-the-difference-between-machine-learning-and-neural-networks
%  ANN vs Deep Learning: https://bernardmarr.com/default.asp?contentID=1789
% 

%%
% 
% <<ELM_TB.png>>
% 
 
clear all;clc;
%addpath('codes','dataset');
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/dataset/');
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/codes/')
%% Load data
D=load('spambase.data');
A=D(:,1:57);             % Inputs
B=D(:,58);               % Targets
%% define Options
Opts.ELM_Type='Regrs';    % 'Class' for classification and 'Regrs' for regression
Opts.number_neurons=200;  % Maximam number of neurons 
Opts.Tr_ratio=0.70;       % training ratio
Opts.Bn=1;                % 1 to encode  lables into binary representations
                          % if it is necessary
%% Training
[net]= elm_LB(A,B,Opts);

%% prediction
[output]=elmPredict(net,A);

e = gsubtract(B,output);
%performance = perform(net,tTesting,output)

figure, ploterrhist(e)

% Video de dataset cifar-10: https://www.mathworks.com/videos/training-a-neural-network-from-scratch-with-matlab-1492008542195.html
% Tutorial Regresion - Clasification: 



