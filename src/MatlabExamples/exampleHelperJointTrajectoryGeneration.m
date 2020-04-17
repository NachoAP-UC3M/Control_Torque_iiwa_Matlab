function [ q, qdot, qddot, tt ] = exampleHelperJointTrajectoryGeneration( tWaypoints, qWaypoints, tt )
%EXAMPLEHELPERJOINTTRAJECTORYGENERATION Generate joint trajectory from
%   given time and joint configuration waypoints.
%
%   Input:
%   tWaypoints: 1-by-n vector, where n is number of waypoints
%   qWaypoints: n-by-ndof matrix, where ndof is the number of degrees of
%               freedom.
%   tt: 1-by-np vector containing time points where the piece-wise 
%       polynomial trajectories are evaluated.
%   
%   Output:
%   qDesired: joint positions, a (n+2)-by-ndof vector
%   qdotDesired: joint velocities, a (n+2)-by-ndof vector
%   qddotDesired: joint accelerations, a (n+2)-by-ndof vector
%   tt: time points for joint trajectory, a 1-by-(n+2) vector
%
%   Explanation for (n+2):
%   To force the trajectory to start and end with zero velocities, the method
%   will automatically prepend the first waypoint and append the last
%   waypoint to the original user-supplied tWaypoints and qWaypoints.
% 
%   The trajectory is generated using |pchip| (Piecewise Cubic Hermite Interpolating
%   Polynomial). |pchip| is not as smooth as |spline|, however, it guarantees
%   that the interpolated joint position does not violate joint limits, as
%   long as the waypoints do not.

% Copyright 2016 The MathWorks, Inc.

if nargout == 1
    q = pchip(tWaypoints,qWaypoints,tt); 
else
    ndof = size(qWaypoints,2);
    dt = 0.01;
    Q = [qWaypoints(1,:);  qWaypoints; qWaypoints(end,:)]';
    T = [0, tWaypoints+dt, tWaypoints(end)+2*dt] ;
    tt = [0, tt + dt, tt(end)+2*dt];

    pp_pos = pchip(T,Q);
    pieces = pp_pos.pieces;

    coefs_vel = zeros(ndof*pieces,3); 
    coefs_acc = zeros(ndof*pieces,2);
    for k = 1:ndof
        for i = 1:pieces
            c = pp_pos.coefs((k-1)*pieces+i,:);
            coefs_vel((k-1)*pieces+i,:) = [3*c(1) 2*c(2) c(3)];
            coefs_acc((k-1)*pieces+i,:) = [6*c(1) 2*c(2)];
        end
    end
    
    pp_vel = pp_pos;
    pp_vel.order = 3;
    pp_vel.coefs = coefs_vel; 
    
    pp_acc = pp_pos;
    pp_acc.order = 2;
    pp_acc.coefs = coefs_acc;

    q = ppval(pp_pos, tt)';
    qdot = ppval(pp_vel, tt)';
    qddot = ppval(pp_acc, tt)';
end


end

