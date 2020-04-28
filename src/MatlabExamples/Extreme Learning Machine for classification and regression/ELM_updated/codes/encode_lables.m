function [BY,BYts,label]=encode_lables(Y,Yts)

% Note : instances must be a colomn vectors
% Y    : trainig Targets
% Yts  : testing Targets
% BY   : binary encoded version of Y
% BYts : binary encoded version of Yts
% Label: Lable of each new input

%%%
NumberofTrainingData=size(Y,2);
NumberofTestingData=size(Yts,2);
%%%%%%%%%%%% Preprocessing the data of classification
    sorted_target=sort(cat(2,Y,Yts),2);
    label=zeros(1,1);                               %   Find and save in 'label' class label from training and testing data sets
    label(1,1)=sorted_target(1,1);
    j=1;
    for i = 2:(NumberofTrainingData+NumberofTestingData)
        if sorted_target(1,i) ~= label(1,j)
            j=j+1;
            label(1,j) = sorted_target(1,i);
        end
    end
    number_class=j;
    NumberofOutputNeurons=number_class;
       
    %%%%%%%%%% Processing the targets of training
    temp_Y=zeros(NumberofOutputNeurons, NumberofTrainingData);
    for i = 1:NumberofTrainingData
        for j = 1:number_class
            if label(1,j) == Y(1,i)
                break; 
            end
        end
        temp_Y(j,i)=1;
    end
    BY=temp_Y;

    %%%%%%%%%% Processing the targets of testing
    temp_Yts=zeros(NumberofOutputNeurons, NumberofTestingData);
    for i = 1:NumberofTestingData
        for j = 1:number_class
            if label(1,j) == Yts(1,i)
                break; 
            end
        end
        temp_Yts(j,i)=1;
    end
 BYts=temp_Yts; 

end