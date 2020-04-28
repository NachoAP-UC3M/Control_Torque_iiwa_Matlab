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
load('netBayesian.mat');
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/dataset/');
addpath('/home/nacho/MATLAB Add-Ons/Collections/Extreme Learning Machine for classification and regression/ELM_updated/codes/')
%% Load data
D=load('spambase.data');
A=D(:,1:57);             % Inputs
B=D(:,58);               % Targets

[X,T] = simplefit_dataset; 
A = X';
B = T';

clear vector_save;
min_v = 100;
for nacho1 = 1:700
%% define Options
Opts.ELM_Type='Regrs';    % 'Class' for classification and 'Regrs' for regression
% n_neu = scaledata([0 rand(1) 1],100,200);
% Opts.number_neurons= round(n_neu(2));  % Maximam number of neurons 
Opts.number_neurons= round(rand(1)*100);  % Maximam number of neurons
Opts.Tr_ratio=0.7;       % training ratio
while Opts.Tr_ratio == 0
    Opts.Tr_ratio=round(rand(1),1);
end
Opts.Bn=1;                % 1 to encode  lables into binary representations
                          % if it is necessary
%% Training
[net]= elm_LB(A,B,Opts);


%% prediction
[output1]=elmPredict(net,A);

e = gsubtract(B,output1);
%e = gsubtract(tTesting,output);
etotal1 = sum((sqrt((e).^2)));
%performance = perform(net,tTesting,output)

vector_save(nacho1,:) = [Opts.number_neurons etotal1 Opts.Tr_ratio];

if vector_save(nacho1,2) < min_v
    min_v = vector_save(nacho1,2);
    conf_vs = [net.Opts.number_neurons net.Opts.Bn net.min net.max]; %net.Opts.ELM_Type 
    pesos_vs = [net.IW net.OW]; 
end

end

[M,p] = min(vector_save(:,2))
vector_save(p,3)
vector_save(p,1)

net.Opts.number_neurons = conf_vs(1);
%net.Opts.ELM_Type = pesos_vs(2);
net.Opts.Bn = conf_vs(2);
net.min = conf_vs(3);
net.max = conf_vs(4);
net.IW = pesos_vs(:,1);
net.OW = pesos_vs(:,2);

[output]=elmPredict(net,A);

e = gsubtract(B,output);
%e = gsubtract(tTesting,output);
etotal = sum((sqrt((e).^2)))

figure, ploterrhist(e)

yB = net_Bayesian(A');

% figure; hold on; plot(B);plot(output);plot(yB);
% legend('Salida original','output','Bayesian_regularization')
figure; hold on; plot(B);plot(output);plot(output1);
legend('Salida original','output-Bueno','output-Malo')


% figure;hold on;plot(t_measured,tau_realTest,'b');plot(t_measured,tau_IDTest,'r');plot(t_measured,tau_IDTest+output,'g');
% legend('tauRobotRealMeasured(Nm)','tauRobotID(Nm)','tauRobotID+IJL(Nm)')



% Video de dataset cifar-10: https://www.mathworks.com/videos/training-a-neural-network-from-scratch-with-matlab-1492008542195.html
% Tutorial Regresion - Clasification: 



