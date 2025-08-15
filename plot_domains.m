f1 = figure;
set(f1, 'Color', 'w'); % Set background color to white
set(f1, 'DefaultTextInterpreter', 'latex');

f2 = figure;
set(f2, 'Color', 'w'); % Set background color to white
set(f2, 'DefaultTextInterpreter', 'latex');

figure(f1)

% Get the indices of the non-zero elements
[k_indices, attack_indices, energy_indices] = ind2sub(size(domain_matrix), find(domain_matrix == 1));

% Convert the indices to the corresponding values in the ranges
k_vals = k_range(k_indices);  % Map to k_range
attack_angle_vals = attack_angle_range(attack_indices);  % Map to attack_angle_range
energy_vals = energy_range(energy_indices);  % Map to energy_range

% Create a 3D scatter plot
scatter3(attack_angle_vals, k_vals, energy_vals, 36, 'filled');

% Labels and title
xlabel('Attack angle range');
ylabel('k range');
zlabel('Energy range');
title('Walking domains');

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);


figure(f2)

% Desired z-value for the cross-section
z_value = 816; % Replace with your desired value
tolerance = 15; % Allow a small range around z_value

% Filter points near the desired z_value
cross_section_indices = abs(energy_vals - z_value) < tolerance;

% Extract the corresponding x and y values
y_cross_section = k_vals(cross_section_indices);
x_cross_section = attack_angle_vals(cross_section_indices);

% Plot the cross-section in 2D
scatter(x_cross_section, y_cross_section, 36, 'filled');

% Labels and title
xlabel('Attack angle range');
ylabel('k range');
% zlabel('Energy range');
title(['Cross-Section at z = ', num2str(z_value)]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);