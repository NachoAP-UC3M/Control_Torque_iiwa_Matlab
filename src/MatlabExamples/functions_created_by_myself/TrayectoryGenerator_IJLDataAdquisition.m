function [t,q]=TrayectoryGenerator_IJLDataAdquisition(numero)

% tWaypoints: Time vector
% qWaypoints: Trayectory vector
%
%
% References take from my own file: IJL_DataAdquisition
switch numero 
    case 1
        
    disp('Trayectoria 1');
A = pi/2;
T = 14;
n = 1;
t = 0:0.1:T;

% Creamos la trayectoria para 7 articulaciones

for i=1:7
   q(:,i) = A*cos((n*t)/T); 
end

figure('Name','Joint'); plot(t,q);

% Sampling rate of 100 Hz = 0.01 s using the Fast Research Interface (FRI)
%
% joints’ torques were filtered through a 4th order zero-
% phase digital Butterworth filter with a cuttoff frequency of
% 1 Hz

% Class Gazebo>SensorTorqueForce: http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ForceTorqueSensor.html
% sensor torque: https://answers.gazebosim.org//question/18715/does-anyone-have-a-working-example-or-a-tutorial-for-a-force_torque-plugin/
              
    case 2
        
        disp('Trayectoria 2');
        
        A = [1.4 1.7 1.6 1 1.2 1.4 1.6]
        T = 20;
        n = [3 1 2 1 1 1 2];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
        
    case 3
        
        disp('Trayectoria 3');
        
        A = [1.4 1.4 1.8 1.4 1.6 1.7 1.2]
        T = 20;
        n = [1 3 2 1 2 3 2];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end

    case 4
        
        disp('Trayectoria 4');
        
        A = [2 1.1 1.8 1.5 1.7 2 1.6]
        T = 60;
        n = [2 1 2 1 3 3 2];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
        
    case 5
        
        disp('Trayectoria 5');
        
        A = [1.5 1.7 1.4 1.1 1.9 1.2 1.6]
        T = 20;
        n = [2 1 2 2 2 1 3];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
     
     case 6
         
        disp('Trayectoria 6');
        
        A = [1.7 1.1 1.4 1.9 1.4 1.7 1.2]
        T = 20;
        n = [1 2 3 3 2 1 3];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
        
     case 7
         
        disp('Trayectoria 7');
        
        A = [1.1 1.4 1.6 1.2 0.9 1.9 1.3]
        T = 30;
        n = [1 2 2 1 3 3 3];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
        
     case 8
         
        disp('Trayectoria 8');
        
        A = [0.9 1.4 1.7 1.5 1.9 1.2 1.6]
        T = 30;
        n = [3 3 2 2 1 1 2];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end
        
     case 9
         
        disp('Trayectoria 9');
        
        A = [1.2 1.3 1.1 1.6 1.3 1.5 1.5]
        T = 20;
        n = [3 2 2 3 1 1 2];
        t = 0:0.1:T;

        % Creamos la trayectoria para 7 articulaciones

        for i=1:7
           q(:,i) = A(i)*cos((n(i)*t)/T);
           
           % Representamos las trayectorias
           JointName = strcat('Joint',int2str(i));
           figure('Name',JointName); plot(t,q(:,i));
        end        
        
     end

end