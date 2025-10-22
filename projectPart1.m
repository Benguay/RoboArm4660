% --- Robot Model Implementation and FK Verfication ---
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant for unit conversion 
deg = pi/180;

% define links using DH params for UFACTORY 850
L(1) = Link([0,  0.364, 0,     90*deg],  'offset', 0,      'R');
L(2) = Link([0,  0,     0.390, 0], 'offset', 90*deg, 'R');
L(3) = Link([0,  0,     0.150, -90*deg], 'offset', 90*deg, 'R');
L(4) = Link([0,  0.426, 0,     90*deg], 'offset', 0,      'R');
L(5) = Link([0,  0,     0,     -90*deg],  'offset', 0,      'R');
L(6) = Link([0,  0.090, 0,     0],       'offset', 0,      'R');

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

position_xyz = T_fk_matrix(1:3, 4)';
fprintf('End-Effector Position (x,y,z) in m: [%.4f, %.4f, %.4f]\n', position_xyz);