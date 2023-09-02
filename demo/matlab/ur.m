clear; clc; close all;

% UR robot
L(1) = Link([0,0,0,pi/2]);
L(2) = Link([pi/2,0,5,0]);
L(3) = Link([-pi/2,0,5,0]);
L(4) = Link([0,1,0,-pi/2]);
L(5) = Link([-pi/2,1,0,-pi/2]);
L(6) = Link([0,1,0,0]);
UR = SerialLink(L,"name","UR");

% offset
UR.offset = [0,pi/2,-pi/2,0,-pi/2,0];

% joint variable
% q = [0,0,0,0,0,0];
q = [0.1,0.2,0.3,0.4,0.5,0.6];

% forward kinematic
T = UR.fkine(q);

% jacobian matrix
J = UR.jacob0(q);

% innverse kinematic
q_ikine = UR.ikine(T);

% plot
figure(1); view(3);
UR.plot(q);
