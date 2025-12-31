clc;
clear all;
origin_RPP = [30,150,-60];
%% Displacement to the welding zone 

%% Mount the head of the robot
% Define time parameters for the trajectory
tf = 5;
frequency = 3;
number_of_elements = tf/(1/frequency);
t = linspace(0,tf,number_of_elements);
%figure;
for i = 1:number_of_elements
    rt = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
    t1_RPP_traj(i) = -pi/2;
    p2_RPP_traj(i) = (13)*rt; 
    p3_RPP_traj(i) = 0; 
    subplot(3,3,[1 2 4 5 7 8])
    prodline();
    RPP3D(t1_RPP_traj(i),p2_RPP_traj(i),p3_RPP_traj(i),origin_RPP);
    hold on;
    RPR3D(0,0,0);
    hold off;
    %Right subplots : all the joints position
    subplot(3,3,3); 
    plot(t(1:i), t1_RPP_traj(1:i), 'b', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('t1 (rad)');
    title('Joint t1 trajectory'); legend('t1');

    subplot(3,3,6); 
    plot(t(1:i), p2_RPP_traj(1:i), 'r', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('p2 (m)');
    title('Joint p2 trajectory'); legend('p2');

    subplot(3,3,9); 
    plot(t(1:i), p3_RPP_traj(1:i), 'g', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('p3 (m)');
    title('Joint p3 trajectory'); legend('p3');

    drawnow;
    pause(0.00000001); 
end

%% displace it on the shelves
for i = 1:number_of_elements
    rt = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;
    t1_RPP_traj(i) = -pi/2 + (0 -(pi/2))*rt;
    p2_RPP_traj(i) = 13; 
    p3_RPP_traj(i) = 0;
    subplot(3,3,[1 2 4 5 7 8])
    prodline();
    RPP3D(t1_RPP_traj(i),p2_RPP_traj(i),p3_RPP_traj(i),origin_RPP);
    hold on;
    RPR3D(0,0,0);
    hold off;
    %Right subplots : all the joints position
    subplot(3,3,3); 
    plot(t(1:i), t1_RPP_traj(1:i), 'b', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('t1 (rad)');
    title('Joint t1 trajectory'); legend('t1');

    subplot(3,3,6); 
    plot(t(1:i), p2_RPP_traj(1:i), 'r', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('p2 (m)');
    title('Joint p2 trajectory'); legend('p2');

    subplot(3,3,9); 
    plot(t(1:i), p3_RPP_traj(1:i), 'g', 'LineWidth', 1.5)
    xlabel('Time (s)'); ylabel('p3 (m)');
    title('Joint p3 trajectory'); legend('p3');

    drawnow;
    pause(0.00000001); 
end
%% Inverse kinematics
% trajectory of the end effector
% trajectory of the end effector
trajectory_X = [];
trajectory_Y = [];
trajectory_Z = [];
trajectory_handle = [];

% Define the jaw coordinates with simple point lists
center_x = 30;
center_y = 70;
center_z = 25;

% Liste 1 : Arc de la mâchoire (forme en U compacte) - 12 points
X_jaw = [24, 25, 26, 27, 29, 31, 33, 35, 36, 37, 38, 24];
Y_jaw = [65, 64, 63, 62, 62, 62, 62, 63, 64, 65, 66, 66];
Z_jaw = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25];

% Dent 1 (incisive gauche) - 4 points
X_dent1 = [26, 26, 27, 27];
Y_dent1 = [63, 66, 66, 63];
Z_dent1 = [25, 25, 25, 25];

% Dent 2 (canine) - 4 points
X_dent2 = [29, 29, 30, 30];
Y_dent2 = [62, 66, 66, 62];
Z_dent2 = [25, 25, 25, 25];

% Dent 3 (incisive centrale) - 4 points
X_dent3 = [31, 31, 32, 32];
Y_dent3 = [62, 65, 65, 62];
Z_dent3 = [25, 25, 25, 25];

% Dent 4 (canine) - 4 points
X_dent4 = [33, 33, 34, 34];
Y_dent4 = [62, 66, 66, 62];
Z_dent4 = [25, 25, 25, 25];

% Dent 5 (incisive droite) - 4 points
X_dent5 = [36, 36, 37, 37];
Y_dent5 = [64, 66, 66, 64];
Z_dent5 = [25, 25, 25, 25];

% Combine all coordinates to form the complete jaw trajectory
X_corners_RPR = [X_jaw, X_dent1];
Y_corners_RPR = [Y_jaw, Y_dent1];
Z_corners_RPR = [Z_jaw, Z_dent1];

%RPR
% Generate the trajectory for each segment for RPR
r1 = 30;
total_segments_RPR = length(X_corners_RPR);
R1_offset_RPR = 30;

N_total_RPR = number_of_elements * total_segments_RPR;
dt_RPR = tf / number_of_elements;
time_RPR = (0:N_total_RPR-1) * dt_RPR;
global_counter_RPR = 1;

R1_RPR_const = 45;
R2_RPR_const = 25;
R3_RPR_const = 30;
D3_RPR_const = 25;
D1_RPR_const = 0;
% Préallouer les tableaux de trajectoire
t1_RPR_traj = zeros(1, N_total_RPR);
p2_RPR_traj = zeros(1, N_total_RPR);
t3_RPR_traj = zeros(1, N_total_RPR);

t1_RPR_speed = zeros(1, N_total_RPR);
p2_RPR_speed = zeros(1, N_total_RPR);
t3_RPR_speed = zeros(1, N_total_RPR);

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
        rt_dot = 30*t(i)^2/(tf^3) - 60*(t(i)^3)/(tf^4) + 30*(t(i)^4)/(tf^5); % speed
        Px_RPR = Pxi_RPR + (Pxf_RPR - Pxi_RPR)*rt_RPR;
        Py_RPR = Pyi_RPR + (Pyf_RPR - Pyi_RPR)*rt_RPR;
        Pz_RPR = Pzi_RPR + (Pzf_RPR - Pzi_RPR)*rt_RPR;
        
        Px_dot = (Px_RPR - Pxi_RPR)*rt_dot;
        Py_dot = (Py_RPR - Pyi_RPR)*rt_dot;
        Pz_dot = (Pz_RPR - Pzi_RPR)*rt_dot;

        trajectory_X = [trajectory_X, Px_RPR];
        trajectory_Y = [trajectory_Y, Py_RPR];
        trajectory_Z = [trajectory_Z, Pz_RPR];

        [theta1_new_RPR, p2_new_RPR, t3_new_RPR] = InverseKinematicsRPR(Px_RPR, Py_RPR, Pz_RPR, R1_RPR_const, R2_RPR_const, R3_RPR_const, 0, D3_RPR_const);

        J_RPR = [ 
            -D1_RPR_const*sin(theta1_new_RPR) - D3_RPR_const*(cos(t3_new_RPR)*sin(theta1_new_RPR)) - cos(theta1_new_RPR)*(p2_new_RPR + R2_RPR_const) - R3_RPR_const*cos(theta1_new_RPR), -sin(theta1_new_RPR) , -D3_RPR_const*(cos(theta1_new_RPR)*sin(t3_new_RPR));
            D1_RPR_const*cos(theta1_new_RPR) + D3_RPR_const*(cos(t3_new_RPR)*cos(theta1_new_RPR)) - sin(theta1_new_RPR)*(p2_new_RPR + R2_RPR_const) - R3_RPR_const*sin(theta1_new_RPR), cos(theta1_new_RPR), -D3_RPR_const*(sin(theta1_new_RPR)*sin(t3_new_RPR));
            0, 0, -D3_RPR_const*cos(t3_new_RPR)
        ];

        %Velocity math : 
        X_dot = [Px_dot;Py_dot;Pz_dot];
        qdot_planar = pinv(J_RPR)*X_dot;

        theta1_dot = qdot_planar(1);
        p2_dot = qdot_planar(2);
        theta3_dot = qdot_planar(3);


        % Store the computed joint angles for plotting later
        t1_RPR_traj(global_counter_RPR) = theta1_new_RPR;
        p2_RPR_traj(global_counter_RPR) = p2_new_RPR;
        t3_RPR_traj(global_counter_RPR) = t3_new_RPR;
        
        t1_RPR_speed(global_counter_RPR) = theta1_dot;
        p2_RPR_speed(global_counter_RPR) = p2_dot;
        t3_RPR_speed(global_counter_RPR) = theta3_dot;

        % Plot
        subplot(3,3,[1 2 4 5 7 8])
        prodline();
        if isempty(trajectory_handle) || ~isvalid(trajectory_handle)
            trajectory_handle = plot3(trajectory_X, trajectory_Y, trajectory_Z, 'k-', 'LineWidth', 3);
        else
            set(trajectory_handle, 'XData', trajectory_X, 'YData', trajectory_Y, 'ZData', trajectory_Z);
        end
        RPP3D(-pi, 13, 0, origin_RPP);
        hold on;        
        RPR3D(theta1_new_RPR, p2_new_RPR, t3_new_RPR);    
        hold off;

        subplot(3,3,3); 
        plot(time_RPR(1:global_counter_RPR), t1_RPR_traj(1:global_counter_RPR), 'b', 'LineWidth', 1.5)
        xlabel('Time (s)'); ylabel('t1 (rad)');
        title('Joint t1 trajectory'); legend('t1');

        subplot(3,3,6); 
        plot(time_RPR(1:global_counter_RPR), p2_RPR_traj(1:global_counter_RPR), 'g', 'LineWidth', 1.5)
        xlabel('Time (s)'); ylabel('p2 (m)');
        title('Joint p2 trajectory'); legend('p2');

        subplot(3,3,9); 
        plot(time_RPR(1:global_counter_RPR), t3_RPR_traj(1:global_counter_RPR), 'r', 'LineWidth', 1.5)
        xlabel('Time (s)'); ylabel('t3 (rad)');
        title('Joint t3 trajectory'); legend('t3');


        global_counter_RPR = global_counter_RPR + 1;
        drawnow;
        pause(0.00001);
    end
end



%% Plot the data : 
figure;

subplot(3,1,1);
plot(time_RPR(1:global_counter_RPR), t1_RPR_traj(1:global_counter_RPR), 'b', 'LineWidth', 1.5)
xlabel('Time (s)'); ylabel('t1 (rad)');
title('Joint t1 trajectory'); legend('t1');

subplot(3,1,2);
plot(time_RPR(1:global_counter_RPR), p2_RPR_traj(1:global_counter_RPR), 'g', 'LineWidth', 1.5)
xlabel('Time (s)'); ylabel('p2 (m)');
title('Joint p2 trajectory'); legend('p2');

subplot(3,1,3);
plot(time_RPR(1:global_counter_RPR), t3_RPR_traj(1:global_counter_RPR), 'r', 'LineWidth', 1.5)
xlabel('Time (s)'); ylabel('t3 (rad)');
title('Joint t3 trajectory'); legend('t3');

figure;

% Left subplot: Position of joints
subplot(3,2,1);
plot(time_RPR(1:global_counter_RPR), t1_RPR_traj(1:global_counter_RPR), 'b')
xlabel('Time(s)'); ylabel('Position \theta_1 (rad)');
title('Position of Joint \theta_1'); grid on;

subplot(3,2,3);
plot(time_RPR(1:global_counter_RPR), p2_RPR_traj(1:global_counter_RPR), 'r')
xlabel('Time(s) '); ylabel('Position \p_2 (rad)');
title('Position of Joint \rho_2'); grid on;

subplot(3,2,5);
plot(time_RPR(1:global_counter_RPR), t3_RPR_traj(1:global_counter_RPR), 'g')
xlabel('Time(s) '); ylabel('Position \theta_3 (mm/s)');
title('Position of Joint \theta_3'); grid on;

% Right subplot: Velocity of joints
subplot(3,2,2);
plot(time_RPR(1:global_counter_RPR), t1_RPR_speed(1:global_counter_RPR), 'b')
xlabel('Time(s) '); ylabel('Velocity \theta_1 (rad/s)');
title('Velocity of Joint \theta_1'); grid on;

subplot(3,2,4);
plot(time_RPR(1:global_counter_RPR), p2_RPR_speed(1:global_counter_RPR), 'r')
xlabel('Time(s) '); ylabel('Velocity \rho_2 (mm/s)');
title('Velocity of Joint \rho_2'); grid on;

subplot(3,2,6);
plot(time_RPR(1:global_counter_RPR), t3_RPR_speed(1:global_counter_RPR), 'g')
xlabel('Time(s) '); ylabel('Velocity \theta_3 (rad/s)');
title('Velocity of Joint \theta_3'); grid on;

sgtitle('Joint Positions and Velocities Visualization - Joint Space');
