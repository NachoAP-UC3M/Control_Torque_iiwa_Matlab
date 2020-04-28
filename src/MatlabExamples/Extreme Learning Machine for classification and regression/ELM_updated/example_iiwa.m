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

joint = 1;
[x,t,xTesting,tTesting,tau_realTest,tau_IDTest,t_measured] = unificar_Dataset(joint);
x = x';
t = t';
xTesting = xTesting';
tTesting = tTesting';
t_measured = t_measured';
tau_realTest = tau_realTest';
tau_IDTest = tau_IDTest';

clear vector_save;
min_v = 100000;
for nacho1 = 1:100
%% define Options
Opts.ELM_Type='Regrs';    % 'Class' for classification and 'Regrs' for regression
% n_neu = scaledata([0 rand(1) 1],0,110);
% Opts.number_neurons= round(n_neu(2));  % Maximam number of neurons
Opts.number_neurons= 100;  % Maximam number of neurons
Opts.Tr_ratio=0.90;       % training ratio
Opts.Bn=1;                % 1 to encode  lables into binary representations
                          % if it is necessary
%% Training
%[net]= elm_LB(A,B,Opts);
[net]= elm_LB(x,t,Opts);

%% prediction
%[output]=elmPredict(net,A);
%[output]=elmPredict(net,x);
[output1]=elmPredict(net,xTesting);
%y = net(xTesting);

%e = gsubtract(B,output);
e = gsubtract(tTesting,output1);
% etotal1 = sum((sqrt((e).^2)));
etotal1 = mse(tTesting-output1);
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
net.IW = pesos_vs(:,1:3);
net.OW = pesos_vs(:,2);

[output]=elmPredict(net,xTesting);

clear output;
e = gsubtract(tTesting,output);
etotal = sum((sqrt((e).^2)))

vector_mse = mse(tau_realTest-tau_IDTest)
vectorIJL_mse = mse(tau_realTest-tau_IDTest+output)

% save('ELM-J1.mat','xTesting','net','conf_vs','pesos_vs','t_measured','tau_realTest','tau_IDTest','output')
figure, ploterrhist(e)


% figure; hold on; plot(B);plot(output);plot(yB);
% legend('Salida original','output','Bayesian_regularization')
figure; hold on; plot(tTesting);plot(output);plot(output1);
legend('Salida original','output-Bueno','output-Malo')

figure; hold on; 
plot(t_measured,tau_realTest);plot(t_measured,tau_IDTest);plot(t_measured,tau_IDTest+output);
legend('TauReal(Nm)','TauID(Nm)','Tau with ELM(Nm)')

% figure;hold on;plot(t_measured,tau_realTest,'b');plot(t_measured,tau_IDTest,'r');plot(t_measured,tau_IDTest+output,'g');
% legend('tauRobotRealMeasured(Nm)','tauRobotID(Nm)','tauRobotID+IJL(Nm)')



% Video de dataset cifar-10: https://www.mathworks.com/videos/training-a-neural-network-from-scratch-with-matlab-1492008542195.html
% Tutorial Regresion - Clasification: 


%%

vn1 = [1 2 14 2 10 7]
vn2 = [3 -10 2 1 1 8]
resta = (vn1-vn2).^2
rn = mse(vn1-vn2)
(sum((vn1-vn2).^2))/size(vn1,2)

%mse(Y-Y_hat)

vrn = rn*ones(1,size(vn1,2))
figure; hold on; plot(resta,'o');plot(vrn);
