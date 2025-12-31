%%% Robotic projects - Subject 2
clc;
clear all;
O = [0 0 0]; % origin for the RPP robot

%% Question 2 : make the robot travels 

% Trajectory planning from TD3-4
% 1 / 2.85 = 0.35
tf =20;
t = linspace(0,20,58);


% RPP part
figure;
%RPP3D(pi,0,0,[0,0,0]);
figure;
%RPP3D(3*pi/4,34,12,[0,0,0]);

% RPR part
figure;
t1_RPR = pi;
p2_RPR = 24;
t3_RPR = 35;
%RPR3D(t1_RPR,p2_RPR,t3_RPR);
figure;
%RPR3D(0,0,0);

%% TEST 

% Create subplots for RPR3D visualization
figure;

% Top view subplot
subplot(2, 1, 1);
view(0, 90); % Set view to top
title('Top View of RPR Robot');
xlabel('X-axis');
ylabel('Y-axis');
hold on;

% Side view subplot
subplot(2, 1, 2);
view(90, 0); % Set view to side
title('Side View of RPR Robot');
xlabel('X-axis');
ylabel('Z-axis');
hold on;

% Call RPR3D function for both views
subplot(2, 1, 1);
RPR3D(0,0,0);

% Update the RPR3D for side view
subplot(2, 1, 2);
RPR3D(0,0,0);


%%
figure;
RPR3D(t1_RPR,p2_RPR,t3_RPR);

%% Question 4 : DH matrix 
%RPP:
syms R1_RPP P2_RPP P3_RPP T1_RPP PIsym D2_RPP R2_RPP R3_RPP R4_RPP D3_RPP

d_RPP = [0;0;0;0];
r_RPP = [R1_RPP;P2_RPP+R2_RPP;P3_RPP+R3_RPP;R4_RPP];
theta_RPP= [T1_RPP;0;0;0];
alpha_RPP = [0;0;-PIsym/2;0];
    
[T0Tn_RPP, entities_RPP] = DenaHart(alpha_RPP,d_RPP,theta_RPP,r_RPP);
disp("POSE 1 RPP ");
Tnum_RPP = double(subs(T0Tn_RPP, {R1_RPP,T1_RPP,P2_RPP,R2_RPP,D2_RPP,P3_RPP,R3_RPP,R4_RPP,PIsym, D3_RPP }, {20,0,0,50,0,0,40,90,pi,0}));
joints_RPP = [T1_RPP,P2_RPP,P3_RPP];
disp(Tnum_RPP);

disp("POSE 2 RPP");
Tnum_RPP = double(subs(T0Tn_RPP, {R1_RPP,T1_RPP,P2_RPP,R2_RPP,D2_RPP,P3_RPP,R3_RPP,R4_RPP,PIsym,D3_RPP }, {30,3*pi/4,34,40,0,12,60,70,pi,0}));
disp(Tnum_RPP);


% RPR
syms  R1_RPR P2_RPR T3_RPR T1_RPR R4_RPR D1_RPR R2_RPR D3_RPR R3_RPR PIsym
disp("RPR Robot");
d_RPR = [0;D1_RPR;0;D3_RPR];
r_RPR = [R1_RPR;P2_RPR+R2_RPR;R3_RPR;0];
theta_RPR= [T1_RPR;0;T3_RPR;0];
alpha_RPR = [0;-PIsym/2;0;0];
    
[T0Tn_RPR, entities_RPR] = DenaHart(alpha_RPR,d_RPR,theta_RPR,r_RPR);
Tnum_RPR = double(subs(T0Tn_RPR, {R1_RPR,T1_RPR,D1_RPR,P2_RPR,R2_RPR,T3_RPR,R3_RPR,D3_RPR,PIsym}, {25,t1_RPR,20,p2_RPR,25,t3_RPR,30,25,pi}));
disp("POSE 1 RPR")
disp(Tnum_RPR)
Tnum_RPR = double(subs(T0Tn_RPR, {R1_RPR,T1_RPR,D1_RPR,P2_RPR,R2_RPR,T3_RPR,D3_RPR,R3_RPR,PIsym}, {45,0,0,0,25,0,25,30,pi}));
disp("POSE 2 RPR")
disp(Tnum_RPR)
joints_RPR = [T1_RPR,P2_RPR,T3_RPR];




%% Question 5 : Jacobian matrix 

% Jacobian matrix for RPP
end_effector_pos_RPP = [T0Tn_RPP(1,4); T0Tn_RPP(2,4); T0Tn_RPP(3,4)];
disp(' ');
disp('Jacobienne pour le robot RPP (expression symbolique):');
J_RPP = jacobian(end_effector_pos_RPP, joints_RPP);
disp(J_RPP);


% Jacobian matrix for RPR
end_effector_pos_RPR = [T0Tn_RPR(1,4); T0Tn_RPR(2,4); T0Tn_RPR(3,4)];
disp(' ');
disp('Jacobienne pour le robot RPR (expression symbolique):');
J_RPR = jacobian(end_effector_pos_RPR, joints_RPR);
disp(J_RPR);

det(J_RPR)

%% Question 6 : Try to demonstrate a joint space simulation
% frequency = 3Hz
% simulation time = 15 seconds
% step size = 1/3 
tf = 15;
frequency = 3;
number_of_elements = tf/(1/frequency);
t = linspace(0,15,number_of_elements);

%%

%RPP
t1_RPP = 0;
p2_RPP = 0;
p3_RPP = 0;
%figure;
for i = 1:number_of_elements
    rt = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
    t1_RPP_traj(i) = 0 + (3*pi/4 - 0)*rt;
    p2_RPP_traj(i) = 0 + (30 - 0)*rt; 
    p3_RPP_traj(i) = 0 + (22- 0)*rt; 
    subplot(3,3,[1 2 4 5 7 8])
    %RPP3D(t1_RPP_traj(i),p2_RPP_traj(i),p3_RPP_traj(i),O);
    % Right subplots : all the joints position
    %subplot(3,3,3); 
    %plot(t(1:i), t1_RPP_traj(1:i), 'b', 'LineWidth', 1.5)
    %xlabel('Time (s)'); ylabel('t1 (rad)');
    %title('Joint t1 trajectory'); legend('t1');

    %subplot(3,3,6); 
    %plot(t(1:i), p2_RPP_traj(1:i), 'r', 'LineWidth', 1.5)
    %xlabel('Time (s)'); ylabel('p2 (m)');
    %title('Joint p2 trajectory'); legend('p2');

    %subplot(3,3,9); 
    %plot(t(1:i), p3_RPP_traj(1:i), 'g', 'LineWidth', 1.5)
    %xlabel('Time (s)'); ylabel('p3 (m)');
    %title('Joint p3 trajectory'); legend('p3');

    drawnow;
    pause(0.0001); 
end

%RPR
t1_RPR = 0;
p2_RPR = 0;
t3_RPR = 0;
figure;
for i = 1:number_of_elements
    rt = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
    t1_RPR_traj(i) = 0 + (3*pi/4 - 0)*rt;
    p2_RPR_traj(i) = 0 + (30 - 0)*rt; 
    t3_RPR_traj(i) = 0 + (pi - 0)*rt; 
    subplot(3,3,[1 2 4 5 7 8])
    RPR3D(t1_RPR_traj(i),p2_RPR_traj(i),t3_RPR_traj(i));
    % Right subplots : all the joints position
    subplot(3,3,3); 
    plot(t(1:i), t1_RPR_traj(1:i), 'b', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('t1 (rad)');
    title('Joint t1 trajectory'); legend('t1');

    subplot(3,3,6); 
    plot(t(1:i), p2_RPR_traj(1:i), 'r', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('p2 (m)');
    title('Joint p2 trajectory'); legend('p2');

    subplot(3,3,9); 
    plot(t(1:i), t3_RPR_traj(1:i), 'g', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('t3 (rad)');
    title('Joint t3 trajectory'); legend('t3');

    drawnow;
    pause(0.0001); 
end


%% Question 7 : Inverse Kinematics 


% RPP 
r1 = 30;
X_corners = [-70 -90 -90 -70 -70 -70 -90 -90 -70 -70  -70];
Y_corners = [ 130  130  150  150  130  130  130  150  150  130  130];
Z_corners = [ 70  70   70   70 70  80  80   80   80 80  70];
total_segments = length(X_corners);
R1_RPP_val = 30;
R2_RPP_val = 40;
R3_RPP_val = 60;
R4_RPP_val = 70;

OFFSET_P2_Z = R1_RPP_val + R2_RPP_val; % Devrait être 70
OFFSET_P3_RADIAL = R3_RPP_val + R4_RPP_val; % Devrait être 130
N_total = number_of_elements * total_segments;
dt = tf / number_of_elements;     % seconds per point
time = (0:N_total-1) * dt;   % in seconds
global_counter = 1;
figure;

% Generate the trajectory for each segment
for k = 2:total_segments
    Pxi = X_corners(k-1);
    Pxf = X_corners(k);
    Pyi = Y_corners(k-1);
    Pyf = Y_corners(k);
    Pzi = Z_corners(k-1);
    Pzf = Z_corners(k);

    for i = 1:number_of_elements
        rt = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
        Px = Pxi +(Pxf - Pxi)*rt; % A+ (B-A)*rt
        Py = Pyi + (Pyf - Pyi)*rt;
        Pz = (Pzi + (Pzf - Pzi)*rt); % 30 is for the base of the frame
        
        [theta1_new, p2_new, p3_new] = InverseKinematicsRPP(Px, Py, Pz, OFFSET_P2_Z, OFFSET_P3_RADIAL);

        J = [cos(theta1_new), 0, -sin(pi/2)*sin(theta1_new),      -p3_new*sin(pi/2)*sin(theta1_new);
             sin(theta1_new),  0,  sin(pi/2)*cos(theta1_new),       p3_new*sin(pi/2)*cos(theta1_new);
             0,             -sin(pi/2),              0, p2_new + r1 ;
             0,                         0,                         0,                                     1];

        % Store the computed joint angles for plotting later
        t1_RPP_traj(global_counter) = theta1_new;
        p2_RPP_traj(global_counter) = p2_new;
        p3_RPP_traj(global_counter) = p3_new;
        global_counter = global_counter + 1;

        subplot(3, 3, [1 2 4 5 7 8]);
        RPP3D(theta1_new, p2_new, p3_new, O);
        hold on;
        path_vision(X_corners, Y_corners, Z_corners);
        hold off;

        % Right subplots: all the joints position
        subplot(3, 3, 3); 
        plot(t(1:global_counter-1), t1_RPP_traj(1:global_counter-1), 'b', 'LineWidth', 1.5);
        xlabel('Time (s)'); ylabel('t1 (rad)');
        title('Joint t1 trajectory'); legend('t1');

        subplot(3, 3, 6); 
        plot(t(1:global_counter-1), p2_RPP_traj(1:global_counter-1), 'r', 'LineWidth', 1.5);
        xlabel('Time (s)'); ylabel('p2 (m)');
        title('Joint p2 trajectory'); legend('p2');

        subplot(3, 3, 9); 
        plot(t(1:global_counter-1), p3_RPP_traj(1:global_counter-1), 'g', 'LineWidth', 1.5);
        xlabel('Time (s)'); ylabel('p3 (m)');
        title('Joint p3 trajectory'); legend('p3');

        drawnow;
        pause(0.00001);
    end
end

%% RPR
% Generate the trajectory for each segment for RPR
r1 = 30;
X_corners_RPR = [-70 -90 -90 -70 -70 -70 -90 -90 -70 -70  -70];
Y_corners_RPR = [ 130  130  150  150  130  130  130  150  150  130  130];
Z_corners_RPR = [ 70  70   70   70 70  80  80   80   80 80  70];
total_segments_RPR = length(X_corners_RPR);
R1_offset_RPR = 30; % Correspond à votre 'r1 = 30;'

N_total_RPR = number_of_elements * total_segments_RPR;
dt_RPR = tf / number_of_elements;     % seconds per point
time_RPR = (0:N_total_RPR-1) * dt_RPR;   % in seconds
global_counter_RPR = 1;

R1_RPR_const = 45;
R2_RPR_const = 25;
R3_RPR_const = 30;
D3_RPR_const = 25;
figure;
% Generate the trajectory for each segment
for k = 2:total_segments_RPR
    Pxi_RPR = X_corners_RPR(k-1);
    Pxf_RPR = X_corners_RPR(k);
    Pyi_RPR = Y_corners_RPR(k-1);
    Pyf_RPR = Y_corners_RPR(k);
    Pzi_RPR = Z_corners_RPR(k-1);
    Pzf_RPR = Z_corners_RPR(k);

    for i = 1:number_of_elements
        rt_RPR = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
        Px_RPR = Pxi_RPR +(Pxf_RPR - Pxi_RPR)*rt_RPR; % A+ (B-A)*rt
        Py_RPR = Pyi_RPR + (Pyf_RPR - Pyi_RPR)*rt_RPR;
        Pz_RPR = Pzi_RPR + (Pzf_RPR - Pzi_RPR)*rt_RPR; 
        
        [theta1_new_RPR, p2_new_RPR, t3_new_RPR] =  InverseKinematicsRPR(Px_RPR, Py_RPR, Pz_RPR, R1_RPR_const, R2_RPR_const, R3_RPR_const, 0,D3_RPR_const);
        % Store the computed joint angles for plotting later
        t1_RPR_traj(global_counter_RPR) = theta1_new_RPR;
        p2_RPR_traj(global_counter_RPR) = p2_new_RPR;
        t3_RPR_traj(global_counter_RPR) = t3_new_RPR;
        global_counter_RPR = global_counter_RPR + 1;

        RPR3D(theta1_new_RPR,p2_new_RPR,t3_new_RPR);
        hold on;
        path_vision(X_corners_RPR,Y_corners_RPR,Z_corners_RPR);
        hold off;
        pause(0.00001);
    end
end