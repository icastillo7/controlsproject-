%% DC Motor Control with Feedback
clear all, close all, close
R = 2.0;      % Ohms
L = 0.5;      % Henrys
Km = 0.1;     % torque constant
Kb = 0.1;     % back emf constant
kf = 0.20;    % Nms
J = 0.02;     % kg.m^2/s^2 (Newton N)

%% State Space Representation model of DC motor
% two inputs, (Va, Td) and one output (w)
% variables:
%Va- applied voltage
%Td- torque disturbance
%w- angular velocity
h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J kf]);            % eqn of motion

dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
%dcm = feedback(dcm,Kv,1,1);   % close back emf loop

% Plot of angular velocity response to change in voltage Va
figure(1)
stepplot(dcm(1));

%% Feedback control
% enforce steady state error, use integral control 
% of the form C(s)=K/s, where K is to be determined

figure(2)
h = rlocusplot(tf(1,[1 0]) * dcm(1));
setoptions(h,'FreqUnits','rad/s');
xlim([-15 5]);
ylim([-15 15]);



%% Feed back design
t = 0:0.1:15;
Td = -0.1 * (t>5 & t<10);     % load disturbance
u = [ones(size(t)) ; Td];    % w_ref=1 and Td

Kb =5;
C = tf(Kb,[1 0]); % compensator K/s

figure(4)
cl_rloc = feedback(dcm * append(C,1),1,1,1);
h = lsimplot(cl_rloc,u,t);
title('Control feedback with Disturbance Rejection')
legend('feedback w/ rlocus','Location','NorthWest')

% Annotate plot
line([5,5],[.2,.3]);
line([10,10],[.2,.3]);
text(7,.25,{'disturbance','T_d = -0.1 Nm'},...
            'vertic','middle','horiz','center','color','r');

