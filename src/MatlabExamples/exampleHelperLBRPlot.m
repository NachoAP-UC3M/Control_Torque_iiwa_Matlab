function exampleHelperLBRPlot(count, timePoints, feedForwardTorque, pdTorque, Q, QDesired )
%exampleHelperLBRPlot Plot joint torque and position trajectories for
%   LBRTorqueControlExample

% Copyright 2016 The MathWorks, Inc.

figure(2)
ii = count;
timePoints = timePoints(1:ii);
feedForwardTorque = feedForwardTorque(1:ii,:);
pdTorque = pdTorque(1:ii,:);
Q = Q(1:ii, :);
QDesired = QDesired(1:ii, :);
for j = 1:7
    subplot(7,1,j);
    plot(timePoints, feedForwardTorque(:,j), 'm'); hold on
    plot(timePoints, pdTorque(:,j), 'r');
    plot(timePoints, pdTorque(:,j) + feedForwardTorque(:,j), 'k');
    if j == 1
        legend('feed-forward torque (N)','PD torque','total torque');
    end
    if j == 7
        xlabel('time (s)');
    end
    s = sprintf('jnt\\_%d',j);
    ylabel(s);
    grid on;
end

figure(3)
for j = 1:7
    subplot(7,1,j);
    plot(timePoints, Q(:,j), 'm'); hold on
    plot(timePoints, QDesired(:,j), 'b'); 
    if j == 1
        legend('actual joint position (rad)','desired joint position');
    end
    if j == 7
        xlabel('time (s)');
    end
    s = sprintf('jnt\\_%d',j);
    ylabel(s);
    grid on;
end

end

