function path_vision(x, y, z)
% TRAJECTORY_RECTANGLE - Draws the 3D rectangular path of the end-effector
%
% Usage:
%   trajectory_rectangle(x, y, z)   % x, y, z are vectors of the same length representing segments

    if nargin < 3
        error('Please provide x, y, and z vectors of the same length.');
    end

    if length(x) ~= length(y) || length(x) ~= length(z)
        error('x, y, and z must be vectors of the same length.');
    end

    % Plot the trajectory in 3D
    plot3(x, y, z, 'b-', 'LineWidth', 3);
    hold on;
    plot3(x, y, z, 'ko', 'MarkerFaceColor', 'k'); % nodes

    % Annotate path numbers
    for i = 1:length(x)
        text(x(i), y(i), z(i)+1, num2str(i), 'FontSize', 10, 'Color', 'k');
    end

    % Axes formatting
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    grid on;
    axis equal;
    title('3D Rectangular End-Effector Trajectory');
    view(45, 25);
end
