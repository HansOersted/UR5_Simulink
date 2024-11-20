%%
clear
clc

%% DH convention

% The parameters are from 
% <Force Estimation in Robotic Manipulators: Modeling, Simulation and Experiments>
% and <Kinematic and dynamic modelling of UR5 manipulator>

% joint 1
alpha1 = pi/2;
a1 = 0.0;
d1 = 0.08916;

% joint 2
alpha2 = 0.0;
a2 = -0.425;
d2 = 0.0;

% joint 3
alpha3 = 0.0;
a3 = -0.3925;
d3 = 0.0;

% joint 4
alpha4 = pi/2;
a4 = 0.0;
d4 = 0.10915;

% joint 5
alpha5 = -pi/2;
a5 = 0.0;
d5 = 0.09465;

% joint 6
alpha6 = 0.0;
a6 = 0.0;
d6 = 0.0823;

alpha_vec = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6];
a_vec = [a1, a2, a3, a4, a5, a6];
d_vec = [d1, d2, d3, d4, d5, d6];

%% masses and inertias

% Link masses
masses = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879];

% Inertia tensors (3x3 matrices for each link in local coordinates)
inertias = {
    diag([0.0084, 0.0064, 0.0084]), ...
    diag([0.0078, 0.2100, 0.2100]), ...
    diag([0.0016, 0.0462, 0.0462]), ...
    diag([0.0016, 0.0016, 0.0009]), ...
    diag([0.0016, 0.0016, 0.0009]), ...
    diag([0.0001, 0.0001, 0.0001])
};

inertiaArray = zeros(3, 3, 6);
for i = 1:numel(inertias)
    inertiaArray(:, :, i) = inertias{i};
end

theta_initial_vec = [0.3, 0.2, 0, 0, 0, 0]; % initial angles
dtheta_initial_vec = [0, 0, 0.04, 0, 0, 0]; % initial angles

%% Setup the calculation of C*dq and tau_g in simulink

% create UR5 rigidBodyTree model
ur5 = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 6);
ur5.Gravity = [0, 0, -9.81]; % gravity (m/s^2)

% DH convention (alpha, a, d, theta)
dhparams = [alpha1       a1        d1       theta_initial_vec(1);       % Joint 1
            alpha2       a2        d2       theta_initial_vec(2);       % Joint 2
            alpha3       a3        d3       theta_initial_vec(3);       % Joint 3
            alpha4       a4        d4       theta_initial_vec(4);       % Joint 4
            alpha5       a5        d5       theta_initial_vec(5);       % Joint 5
            alpha6       a6        d6       theta_initial_vec(6)];      % Joint 6

% Add the components
previousBody = ur5.Base;
for i = 1:size(dhparams, 1)
    alpha = dhparams(i, 1);
    a = dhparams(i, 2);
    d = dhparams(i, 3);
    theta = dhparams(i, 4);
    
    % Create joints and links
    jointName = ['joint' num2str(i)];
    bodyName = ['body' num2str(i)];
    joint = rigidBodyJoint(jointName, 'revolute');
    body = rigidBody(bodyName);
    
    % set the joint transform
    setFixedTransform(joint, [a, alpha, d, theta], 'dh');
    
    % add the joint to the link
    body.Joint = joint;
    
    addBody(ur5, body, previousBody.Name);
    previousBody = body;
end

centerOfMasses = [ % COM w.r.t the local coordinate
    0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    0, 0, 0
];

for i = 1:numel(masses)

    bodyName = ['body' num2str(i)];
    body = getBody(ur5, bodyName);

    body.Mass = masses(i);
    body.CenterOfMass = centerOfMasses(i, :);
    inertiaMatrix = inertias{i};
    body.Inertia = [inertiaMatrix(1,1), inertiaMatrix(2,2), inertiaMatrix(3,3), ...
                    inertiaMatrix(1,2), inertiaMatrix(1,3), inertiaMatrix(2,3)];
    
    replaceBody(ur5, bodyName, body);
end

% display
showdetails(ur5);
% showdetails(ur5);
show(ur5);
