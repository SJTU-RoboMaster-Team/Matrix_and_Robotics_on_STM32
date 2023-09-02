clear; clc; close all;

L(1) = Link([0,5,5,0]);
L(2) = Link([0,0,5,0]);
L(3) = Link([0,0,0,pi]);
L(4) = Link([0,0,0,0,1]); L(4).qlim = [0,5];
SCARA = SerialLink(L, 'name', 'SCARA');

% offset
SCARA.offset = [0,0,0,0];

% joint variable
q0 = [0,0,0,0];
q = [0.2,0.5,0.3,3];

% forward kinematic
T = SCARA.fkine(q);

% jacobian matrix
J0 = SCARA.jacob0(q0);
J = SCARA.jacob0(q);

% innverse kinematic
q_ikine = SCARA.ikine(T,"mask",[1,1,1,0,0,1]);
% [Q,R] = qr(J0);
% err = Twist(T).S;

% plot
figure(1); view(3);
SCARA.plot(q);
