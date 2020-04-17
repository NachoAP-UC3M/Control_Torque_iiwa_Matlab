function init_sin_iiwa()
%INIT Summary of this function goes here
%   Detailed explanation goes here

%Cerramos el nodo global y el ROS master 
rosshutdown

%%%%% Los enlaces gracias a los cuales he conseguido guardar la dirección 
%%%%% IP en una variable son:
%%%%% https://ch.mathworks.com/matlabcentral/answers/428272-get-computer-ip-problem
%%%%% https://www.mundotelematico.com/linux-variable-de-entorno-con-direccion-ip-del-equipo/

%rosinit('http://160.69.69.100:11311','NodeName','Matlab');
c1 = 'http://';
[~, result] = system('(ifconfig |grep -A 1 "wlp2s0" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)');
c3 = ':11311';
direccionIP = strcat(c1,result,c3);

% Inicializa el nodo global y el ROS master
rosinit(direccionIP,'NodeName','Matlab');

global data_dir;
    data_dir='/home/nacho/catkin_ws/src/iiwa_stack/roboespas_iiwacontrol/data/';
end

