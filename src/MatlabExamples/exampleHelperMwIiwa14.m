function robot = exampleHelperMwIiwa14()
%exampleHelperMwIiwa14 Create the same RigidBodyTree object as that is
%   imported from the iiwa14.urdf description as shown in LBRTorqueControlExample

% Copyright 2016 The MathWorks, Inc.

%#codegen
robot = robotics.RigidBodyTree('MaxNumBodies', 10, 'DataFormat', 'row');
robot.Gravity = [                     0,                      0,     -9.800000000000001];
robot.BaseName = 'world';


% Add body, 'iiwa_link_0', and joint, 'world_iiwa_joint'
bodyName = 'iiwa_link_0';
bodyMass =                      5;
bodyCoM = [                  -0.1,                      0,    0.07000000000000001];
bodyInertia = [   0.07450000000000001,                 0.1345,    0.08000000000000002,                      0,                  0.035,                      0];
parentName = 'world';
jointName = 'world_iiwa_joint';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_1', and joint, 'iiwa_joint_1'
bodyName = 'iiwa_link_1';
bodyMass =                      4;
bodyCoM = [                     0,                  -0.03,                   0.12];
bodyInertia = [                0.1612,                 0.1476,                 0.0236,                      0,                      0,                 0.0144];
parentName = 'iiwa_link_0';
jointName = 'iiwa_joint_1';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                 0.1575; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.96705972839,          2.96705972839];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_2', and joint, 'iiwa_joint_2'
bodyName = 'iiwa_link_2';
bodyMass =                      4;
bodyCoM = [                0.0003,                  0.059,                  0.042];
bodyInertia = [               0.07098,             0.02505636,    0.05792435999999999, -7.079999999999999e-05,              -5.04e-05,  -0.009912000000000001];
parentName = 'iiwa_link_1';
jointName = 'iiwa_joint_2';
jointType = 'revolute';
T_Joint_to_Parent = [                    -1,  1.012727722296814e-24, -2.068231071102144e-13,                      0; ...
                     -2.068231071102144e-13, -4.896588860146748e-12,                      1,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                 0.2025; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.09439510239,          2.09439510239];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_3', and joint, 'iiwa_joint_3'
bodyName = 'iiwa_link_3';
bodyMass =                      3;
bodyCoM = [                     0,                   0.03,                   0.13];
bodyInertia = [                0.1334,                 0.1257,                 0.0127,                      0,                      0,                -0.0117];
parentName = 'iiwa_link_2';
jointName = 'iiwa_joint_3';
jointType = 'revolute';
T_Joint_to_Parent = [                    -1,  1.012727722296814e-24, -2.068231071102144e-13,                      0; ...
                     -2.068231071102144e-13, -4.896588860146748e-12,                      1,                 0.2045; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.96705972839,          2.96705972839];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_4', and joint, 'iiwa_joint_4'
bodyName = 'iiwa_link_4';
bodyMass =                    2.7;
bodyCoM = [                     0,                  0.067,                  0.034];
bodyInertia = [             0.0452415,              0.0131212,    0.04112030000000001,                      0,                      0,  -0.006150600000000002];
parentName = 'iiwa_link_3';
jointName = 'iiwa_joint_4';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,  4.896588860146748e-12,                     -1,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                 0.2155; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.09439510239,          2.09439510239];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_5', and joint, 'iiwa_joint_5'
bodyName = 'iiwa_link_5';
bodyMass =                    1.7;
bodyCoM = [                0.0001,                  0.021,                  0.076];
bodyInertia = [             0.0305689,            0.027819217,            0.005749717,              -3.57e-06,             -1.292e-05,             -0.0027132];
parentName = 'iiwa_link_4';
jointName = 'iiwa_joint_5';
jointType = 'revolute';
T_Joint_to_Parent = [                    -1,  2.068231071102144e-13, -1.012727722296814e-24,                      0; ...
                                         -0,  4.896588860146748e-12,                      1,                 0.1845; ...
                      2.068231071102144e-13,                      1, -4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.96705972839,          2.96705972839];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_6', and joint, 'iiwa_joint_6'
bodyName = 'iiwa_link_6';
bodyMass =                    1.8;
bodyCoM = [                     0,                 0.0006,                 0.0004];
bodyInertia = [           0.005000936,            0.003600288,            0.004700648,                      0,                      0,              -4.32e-07];
parentName = 'iiwa_link_5';
jointName = 'iiwa_joint_6';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,  4.896588860146748e-12,                     -1,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                 0.2155; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -2.09439510239,          2.09439510239];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_7', and joint, 'iiwa_joint_7'
bodyName = 'iiwa_link_7';
bodyMass =                    0.3;
bodyCoM = [                     0,                      0,                   0.02];
bodyInertia = [               0.00112,                0.00112,                  0.001,                      0,                      0,                      0];
parentName = 'iiwa_link_6';
jointName = 'iiwa_joint_7';
jointType = 'revolute';
T_Joint_to_Parent = [                    -1,  2.068231071102144e-13, -1.012727722296814e-24,                      0; ...
                                         -0,  4.896588860146748e-12,                      1,                  0.081; ...
                      2.068231071102144e-13,                      1, -4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -3.05432619099,          3.05432619099];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_ee', and joint, 'iiwa_joint_ee'
bodyName = 'iiwa_link_ee';
bodyMass =                      0;
bodyCoM = [                     0,                      0,                      0];
bodyInertia = [                     0,                      0,                      0,                      0,                      0,                      0];
parentName = 'iiwa_link_7';
jointName = 'iiwa_joint_ee';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12,                     -0,                     -1,                      0; ...
                                          0,                      1,                     -0,                      0; ...
                                          1,                      0,  4.896588860146748e-12,                  0.045; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'iiwa_link_ee_kuka', and joint, 'iiwa_joint_ee_kuka'
bodyName = 'iiwa_link_ee_kuka';
bodyMass =                      0;
bodyCoM = [                     0,                      0,                      0];
bodyInertia = [                     0,                      0,                      0,                      0,                      0,                      0];
parentName = 'iiwa_link_7';
jointName = 'iiwa_joint_ee_kuka';
jointType = 'fixed';
T_Joint_to_Parent = [                     1, -2.068231071102572e-13, -2.068231071101717e-13,                      0; ...
                      2.068231071102144e-13,                      1, -2.068231071102572e-13,                      0; ...
                      2.068231071102144e-13,  2.068231071102144e-13,                      1,                  0.045; ...
                                          0,                      0,                      0,                      1];

joint = robotics.Joint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = robotics.RigidBody(bodyName);
body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


