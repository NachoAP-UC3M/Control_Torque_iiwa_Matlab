%%
%  Machine Learning vs ANN: https://www.quora.com/What-is-the-difference-between-machine-learning-and-neural-networks
%  ANN vs Deep Learning: https://bernardmarr.com/default.asp?contentID=1789
% 

%%
% 
% <<ELM_TB.png>>
% 
 
clear all;clc;
% Load libraries
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/dataset/');
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/codes/')
%% Load data
joint = 1;
[x,t,xTesting,tTesting,tau_realTest,tau_IDTest,t_measured] = unificar_Dataset(joint);
x = x';
t = t';
xTesting = xTesting';
tTesting = tTesting';
t_measured = t_measured';
tau_realTest = tau_realTest';
tau_IDTest = tau_IDTest';

%% define Options
Opts.ELM_Type='Regrs';    % 'Class' for classification and 'Regrs' for regression
Opts.number_neurons=100;  % Maximam number of neurons 
Opts.Tr_ratio=0.90;       % training ratio
Opts.Bn=1;                % 1 to encode  lables into binary representations
                          % if it is necessary
%% Training
%[net]= elm_LB(A,B,Opts);
[net]= elm_LB(x,t,Opts);

%% prediction
%[output]=elmPredict(net,A);
%[output]=elmPredict(net,x);
[output]=elmPredict(net,xTesting);

e = gsubtract(tTesting,output);
etotal = sum((sqrt((e).^2)))
vector_mse = mse(tau_realTest-tau_IDTest)
vectorIJL_mse = mse(tau_realTest-tau_IDTest+output)

figure, ploterrhist(e)


% figure; hold on; plot(B);plot(output);plot(yB);
% legend('Salida original','output','Bayesian_regularization')
figure; hold on; plot(tTesting);plot(output);
legend('Salida original','RN-ELM')

figure; hold on; 
plot(tau_realTest);plot(tau_IDTest);plot(tau_IDTest+output);
legend('TauReal','TauID','Tau with ELM')

figure; hold on; 
plot(tau_realTest(700:4500));plot(tau_IDTest(700:4500));
legend('TauReal','TauID');

% figure;hold on;plot(t_measured,tau_realTest,'b');plot(t_measured,tau_IDTest,'r');plot(t_measured,tau_IDTest+output,'g');
% legend('tauRobotRealMeasured(Nm)','tauRobotID(Nm)','tauRobotID+IJL(Nm)')

% Video de dataset cifar-10: https://www.mathworks.com/videos/training-a-neural-network-from-scratch-with-matlab-1492008542195.html
% Tutorial Regresion - Clasification: 

