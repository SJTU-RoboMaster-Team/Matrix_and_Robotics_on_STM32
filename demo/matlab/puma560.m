clear; clc; close all;

% mass
m = [0.2645,0.17,0.1705,0,0,0];
% inertia 
I = cat(3,diag([1.542,0,1.542]*1e-3),diag([0,0.409,0.409]*1e-3),diag([0.413,0.413,0]*1e-3),...
    3*eye(3),2*eye(3),1*eye(3));
% 连杆质心位置(i坐标系下--i坐标系原点->i质心)
r_i_ci = [0,-8.5,0,0,0,0;13.225,0,0,0,0,0;0,3.7,8.525,0,0,0]*1e-2;

p560_L(1) = Revolute('d',26.45e-2,	'a',0,      'alpha',-pi/2,  'm',m(1),'r',r_i_ci(:,1),'I',I(:,:,1));
p560_L(2) = Revolute('d',5.5e-2,	'a',17e-2,	'alpha',0,      'm',m(2),'r',r_i_ci(:,2),'I',I(:,:,2));
p560_L(3) = Revolute('d',0,         'a',0,      'alpha',-pi/2,  'm',m(3),'r',r_i_ci(:,3),'I',I(:,:,3));
p560_L(4) = Revolute('d',17.05e-2,  'a',0,      'alpha',pi/2,   'm',m(4),'r',r_i_ci(:,4),'I',I(:,:,4));
p560_L(5) = Revolute('d',0,         'a',0,      'alpha',-pi/2,  'm',m(5),'r',r_i_ci(:,5),'I',I(:,:,5));
p560_L(6) = Revolute('d',0,         'a',0,      'alpha',0,      'm',m(6),'r',r_i_ci(:,6),'I',I(:,:,6));
p560 = SerialLink(p560_L, 'name', 'puma560');

% offset
p560.offset = [0,0,0,0,0,0];

% joint variable
% q = [0,0,0,0,0,0];
% qv = [0,0,0,0,0,0];
% qa = [0,0,0,0,0,0];
% he = [0,0,0,0,0,0]';
q = [0.2, -0.5, -0.3, -0.6, 0.5, 0.2];
qv = [1, 0.5, -1, 0.3, 0, -1];
qa = [0.2, -0.3, 0.1, 0, -1, 0];
he = [1, 2, -3, -0.5, -2, 1]';

% forward kinematic
T = p560.fkine(q);

% jacobian matrix
J0 = p560.jacob0(zeros(1,6));
J = p560.jacob0(q);

% innverse kinematic
% iter step J\Twist(T).S
q_ikine = p560.ikine(T);
err = Twist(T).S;

% inverse dynamic
torq = p560.rne(q,qv,qa,"fext",[T.R',zeros(3,3);zeros(3,3),T.R']*he);

% plot
figure(1); view(3);
p560.plot(q);
