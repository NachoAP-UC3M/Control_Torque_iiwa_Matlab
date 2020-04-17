%redireccionador de trayectoria 
function [t_trayectoria,q_trayectoria] = redireccionarInicioTrayectoria(tWaypoints,qWaypoints)

    % homeRobotPosition = joint positions [0,0,0,0,0,0,0]
    %
    % Comprobation to know if the trayectory start in homeRobotPosition or
    % not
    
    pto_1 = qWaypoints(1,:);
    pto_2 = qWaypoints(end,:);

% Visualizar distancia entre punto inicial y punto final

IIWA = importrobot('iiwa14.urdf');
IIWA.DataFormat = 'row';
% Set the gravity to be the same as that in Gazebo.
IIWA.Gravity = [0 0 -9.80];


figure;
hold on;
show(IIWA);
view([150 12]);
axis([-1.4 1.4 -1.4 1.4 -0.3 1.35]);
camva(9);
daspect([1 1 1]);

pto_1_XYZ = directkinematic_IIWA14_R820(pto_1);
pto_2_XYZ = directkinematic_IIWA14_R820(pto_2);

plot3(pto_1_XYZ(1,4),pto_1_XYZ(2,4),pto_1_XYZ(3,4),'o','Color','b');
plot3(pto_2_XYZ(1,4),pto_2_XYZ(2,4),pto_2_XYZ(3,4),'o','Color','r');

% Establecemos tiempo y duración de la trayectoria
    
    % La duración de la trayectoria se establece teniendo en cuenta los
    % limites de velocidad articular de cada articulación
    
% q_trayectoria = [puntosRadianes(pto_1,:);
%                  puntosRadianes(pto_2,:)];

% distancia_q1 = abs(abs(puntosRadianes(pto_1,1))-abs(puntosRadianes(pto_2,1)));
% distancia_q2 = abs(abs(puntosRadianes(pto_1,2))-abs(puntosRadianes(pto_2,2)));

for i = 1:7
    
distancia_q0 = abs(pto_1);

if pto_1(i) > 0 & pto_2(i) > 0 | pto_1(i) < 0 & pto_2(i) < 0 % Si tienen mismo signo pto_1 y pto_2
    disp('Mismo signo de Mierda');
    if pto_1(i) < pto_2(i)
       distancia_q2(i) = abs(abs(pto_2(i))-abs(pto_1(i)));
    else
       distancia_q2(i) = abs(abs(pto_1(i))-abs(pto_2(i)));
    end
else % Si tiene distinto signo pto_1 y pto_2
    disp('Distinto signo me cago en tu padre');
    distancia_q2(i) = abs(abs(pto_1(i))+abs(pto_2(i)));
end

end

% - Trayectoria de recolocación - (de homeposition del robot a inicio de
% trayectoria)
tiempo1 = 0;

if pto_1(1) ~= 0 | pto_1(2) ~= 0 | pto_1(3) ~= 0 | pto_1(4) ~= 0 | pto_1(5) ~= 0 | pto_1(6) ~= 0 | pto_1(7) ~= 0
    % CONTINUAMOS CON EL PROGRAMA IJL_DataAcquisition.m
    tiempo1 = 1;
    while distancia_q0(1) > 1.31 | distancia_q0(2) > 1.31 | distancia_q0(3) > 1.31 | distancia_q0(4) > 1.31 | distancia_q0(5) > 1.31 | distancia_q0(6) > 1.31 | distancia_q0(7) > 1.31
        tiempo1 = tiempo1+1; 
        distancia_q0 = distancia_q0 - 1.31;
    end
end

% Si la trayectoria no comienza en el homePosition se añade la trayectoria
% desde el homeposition hasta el punto inicial de la trayectoria. Si
% comienza en el homeposition no se añade.

if pto_1(1) ~= 0 | pto_1(2) ~= 0 | pto_1(3) ~= 0 | pto_1(4) ~= 0 | pto_1(5) ~= 0 | pto_1(6) ~= 0 | pto_1(7) ~= 0
    q_trayectoria(1,:) = [0 0 0 0 0 0 0];
    t_trayectoria = [0];
        for i=1:size(tWaypoints,2)
            tiempo2 = tiempo1+tWaypoints(i);
            t_trayectoria(i+1) = tiempo2;
            
            q_trayectoria(i+1,:) = qWaypoints(i,:);
        end

    %t_trayectoria = [0,tiempo1,tiempo2];

else
    q_trayectoria = qWaypoints;
    t_trayectoria = tWaypoints;
end

end

