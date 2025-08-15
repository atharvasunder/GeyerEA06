f1 = figure;
set(f1, 'Color', 'w'); % Set background color to white
set(f1, 'DefaultTextInterpreter', 'latex');


figure(f1)

r = 1; % Radius of the semicircle
theta = linspace(0, pi, 100); % Angles from 0 to π (upper half)

% Parametric equations for the semicircle
x = r * cos(theta);
y = r * sin(theta);

% Filter points above the line y = y_land
x_filtered = x(y > y_land);
y_filtered = y(y > y_land);

% Initialize arrays for x and y values
apex_x_vals = [];
apex_y_vals = [];

% Loop through the cell array to extract x, y values
for i = 1:length(apex_coordinates)
    apex_x_vals = [apex_x_vals, apex_coordinates{i}(1)]; % Extract x value
    apex_y_vals = [apex_y_vals, apex_coordinates{i}(2)]; % Extract y value
end

check1 = apex_x_vals
check_2 = apex_y_vals   

quiver(X_poincare, Y_poincare, U, V, 'Color', 'black', 'AutoScale', 'off', LineWidth=1.35); % Plot vectors

hold on;

% Plot the semicircle portion above y_land
plot(x_filtered, y_filtered, 'b-', 'LineWidth', 1.5);

% Plot the points and connect them with a line
plot(apex_x_vals, apex_y_vals, 'o-', 'MarkerFaceColor', 'r', 'LineWidth', 1);

xlabel('x tilda rel');
ylabel('y tilda');
title('Quiver Plot of Input to Output Points');

grid off;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);
