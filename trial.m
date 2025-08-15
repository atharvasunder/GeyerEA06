% % Sample data for kinetic energy (KE) and potential energy (PE)
% t = linspace(0, 10, 100); % time vector
% KE = 100 + 50*sin(t);     % kinetic energy
% PE = 500 + 50*cos(t);     % potential energy
% 
% % Shift KE data up by an offset to bring it close to PE
% offset = 400; % Offset value to align the ranges visually
% KE_shifted = KE + offset;
% 
% % Plot both curves on the same axes
% figure;
% plot(t, KE_shifted, 'b', 'LineWidth', 1.5); hold on;
% plot(t, PE, 'r', 'LineWidth', 1.5);
% 
% % Customize y-axis to indicate the offset
% yticks([500, 600, 500 + offset, 600 + offset]); % Custom y-ticks for KE and PE ranges
% yticklabels({'500', '600', '100', '200'}); % Set labels to reflect original values
% 
% % Add labels and title
% xlabel('Time (s)');
% ylabel('Energy');
% title('Kinetic and Potential Energy vs Time with Adjusted Y-axis');
% 
% % Optional: Add break lines on y-axis to show the "kink" or break
% line([0, 0.5], [500, 500 + offset], 'Color', 'k', 'LineStyle', '--'); % Left break line
% line([10, 9.5], [600, 600 + offset], 'Color', 'k', 'LineStyle', '--'); % Right break line
% 
% legend('Kinetic Energy (shifted)', 'Potential Energy');

% 
% x = linspace(-1, 1, 100);
% y = linspace(0, 3, 100);
% 
% % Preallocate the cell array
% coordinates = cell(length(y), length(x));
% 
% % Store the coordinate pairs
% for i = 1:length(x)
%     for j = 1:length(y)
%         coordinates{j, i} = [x(i), y(j)];
%     end
% end
% 
% check = coordinates

% % Define start points
% X = [0, 1, 2, 3];
% Y = [0, 1, 0, 1];
% 
% % Define end points (arbitrary)
% X_end = [1, 2, 3, 4];
% Y_end = [0.5, 1.5, 0.5, 1.2];
% 
% % Calculate displacements (U, V)
% U = X_end - X; % Difference in x-coordinates
% V = Y_end - Y; % Difference in y-coordinates
% 
% % Quiver plot
% quiver(X, Y, U, V, 'Color', 'black', 'AutoScale', 'off', 'LineWidth', 1.35);
% 
% % Add axis labels for clarity
% xlabel('X');
% ylabel('Y');
% axis equal; % Optional: Ensures equal scaling for both axes


[X,Y] = meshgrid(-pi:pi/8:pi,-pi:pi/8:pi);
U = sin(Y);
V = cos(X);
quiver(X,Y,U,V,'r')