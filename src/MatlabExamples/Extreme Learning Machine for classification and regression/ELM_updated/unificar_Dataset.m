function [x,t,xTesting,tTesting,tau_realTest,tau_IDTest,t_measured1] = unificar_Dataset(joint)

% Bibligraphy: The information has been taken form paper "Improving the Inverse Dynamic Model of the KUKA LWR
% IV+ using Independent Joint Learning"
%
% Explanation: In this code it's load 9 trayectories of iiwa14 robot. It's
% choose since the 2 until 9 trayectory to train the ELM neural network. 
% The 1 trayectory use the trained ELM neural network to obtain the desired
% datas. I do it for my case. You can modify the code for your case.
%
% INPUT
%
% joint: Select the joint that want to save their datas
% 
% For example in the iiwa it has 7 joint. You can choose since 1 until 7.
% For the number 4 for example write in the command window:
%
% unificar_Dataset(4)
%
% OUTPUTs
% It is used the trayectory 1 to testing
% xTesting 
% tTesting
% It's used since trayectory 2 until 9 to training and validation 
% x
% t
% 
% tau_realTest: It's the Torque in Newton*meter in the joint choose 
% measured in the robot when the robot does the trayectory

% tau_IDTest: It's the Torque in Newton*meter calculate by the Dynamic inverse Matlab 
% function inverseDynamics. For more information write in the command
% Window:
%
% help inverseDynamics
%
% t_measured1: Time for each point measured in seconds


%joint = 1;
Q_ = [];
Qdot_ = [];
tau_ID_ = [];
error = [];

        dataset = strcat('Input-Dataset-Trayectory',int2str(1),'.mat');
        load(dataset);
        
        Q_=     [Q_;
                 Q(700:end-50,:)];
        Qdot_ = [Qdot_;
                 Qdot(700:end-50,:)];
        tau_ID_ = [tau_ID_;
                    tau_ID(700:end-50,:)];
                
        error = [error;
                 tau_real(700:end-50,:)-tau_ID(700:end-50,:)];
             
%         figure;
%         plot(tau_real(700:end-50,3))%567%585%585%585%585%585%585
%         figure;
%         plot(tau_real(100:end,3))
             
    Q = Q_';
    Qdot = Qdot_';
    tau_ID = tau_ID_';
    error = error';
    tau_real = tau_real(700:end-50,:)';
     
    
    xTesting = [Q(joint,:);
    Qdot(joint,:);
    tau_ID(joint,:)];

    tTesting = [error(joint,:)]; 
    
    tau_realTest = tau_real(joint,:);
    tau_IDTest = tau_ID(joint,:);
    t_measured1 = t_measured(700:end-50);
    
Q_ = [];
Qdot_ = [];
tau_ID_ = [];
error = [];

    for i=2:9
        
        dataset = strcat('Input-Dataset-Trayectory',int2str(i),'.mat');
        load(dataset);
        
%         figure;
%         plot(tau_real(620:end-50,1))%567%585%585%585%585%585%585
%         figure;
%         plot(tau_real(100:end,1))
        
        Q_=     [Q_;
                 Q(620:end-50,:)];
        Qdot_ = [Qdot_;
                 Qdot(620:end-50,:)];
        tau_ID_ = [tau_ID_;
                    tau_ID(620:end-50,:)];
                
        error = [error;
                 tau_real(620:end-50,:)-tau_ID(620:end-50,:)];
        
    end
    
    Q = Q_';
    Qdot = Qdot_';
    tau_ID = tau_ID_';
    error = error';
    
    x = [Q(joint,:);
    Qdot(joint,:);
    tau_ID(joint,:)];

    t = [error(joint,:)];
    
end