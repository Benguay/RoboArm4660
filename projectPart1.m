%% --- Robot Model Implementation and FK Verfication ---
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant for unit conversion 
deg = pi/180;

% define links using DH params for UFACTORY 850 (multiply by 10 for visibility)
L(1) = Link([0,  0.364*10, 0,     90*deg],  'offset', 0,      'R');
L(2) = Link([0,  0,     0.390*10, 0], 'offset', 90*deg, 'R');
L(3) = Link([0,  0,     0.150*10, -90*deg], 'offset', 90*deg, 'R');
L(4) = Link([0,  0.426*10, 0,     90*deg], 'offset', 0,      'R');
L(5) = Link([0,  0,     0,     -90*deg],  'offset', 0,      'R');
L(6) = Link([0,  0.090*10, 0,     0],       'offset', 0,      'R');

% apply joint limits
L(1).qlim = [-360*deg, 360*deg];
L(2).qlim = [-132*deg, 132*deg];
L(3).qlim = [-242*deg, 3.5*deg];
L(4).qlim = [-360*deg, 360*deg];
L(5).qlim = [-124*deg, 124*deg];
L(6).qlim = [-360*deg, 360*deg];

% create SerialLink object
robot = SerialLink(L, 'name', 'UFACTORY 850');

% verify FK
q_test = zeros(1,6);

T_fk_object = robot.fkine(q_test);
T_fk_matrix = T_fk_object.T;

disp('Forward Kinematics Verification at q = [0, 0, 0, 0, 0, 0]:');
disp('End-Effector Pose (T_fk):'); 
disp(T_fk_matrix);

%% Circle and Sphere
h = 6;
k = 4;
l = 8;
centerSphere = [h k l];

function normalVector = calculateNormal(inpX, inpY, inpZ)
% This function calculates the normal vector
normalVector = [2*(inpX-h) 2*(inpY-k) 2*(inpZ-l)];

end

sphereRadius = 2;
%sphereDepth = sqrt(sphereRadius - circleRadius);

[x,y,z] = sphere(100);
x = x * sphereRadius + centerSphere(1);
y = y * sphereRadius + centerSphere(2);
z = z * sphereRadius + centerSphere(3);

circleRadius = 1;
circleCenter = [h, k, l + sqrt(sphereRadius^2-circleRadius^2)];
normal = (circleCenter - centerSphere);
normal = normal / norm(normal);

if abs(dot(normal, [1 0 0])) < 0.9
    v1 = cross(normal, [1 0 0]);
else
    v1 = cross(normal, [0 1 0]);
end
v1 = v1 / norm(v1);
v2 = cross(normal, v1);

% Parametric circle in plane
theta = linspace(0, 2*pi, 200);
circlePoints = circleCenter' + circleRadius * (v1' * cos(theta) + v2' * sin(theta));


figure;
clf;
hold on;
surf(x,y,z, 'FaceColor', 'red', 'FaceAlpha', 0.3);
plot3(circlePoints(1,:), circlePoints(2,:), circlePoints(3,:), 'b-', 'LineWidth', 2);
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

position_xyz = T_fk_matrix(1:3, 4)';
fprintf('End-Effector Position (x,y,z) in m: [%.4f, %.4f, %.4f]\n', position_xyz);

%% Robot animation: make the end-effector z-axis align with sphere normal at each point
% initial guess for IK
q_seed = q_test;

% Set up robot plot window and workspace that includes the sphere
robot.plot(q_seed, 'workspace', [h-8 h+8 k-8 k+8 l-8 l+8], 'noa', 'floorlevel', -10);
hold on;
plot3(circlePoints(1,:), circlePoints(2,:), circlePoints(3,:), 'b-', 'LineWidth', 2);
title('Tracing');

% Preallocate for storing joint solutions
Q = zeros(length(theta), 6);
valid = true(length(theta),1);


fprintf('Done. %d/%d points had valid IK solutions.\n', sum(valid), length(theta));
