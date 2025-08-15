f1 = figure;
f2 = figure;

figure(f1);

set(f1, 'Color', 'w'); % Set background color to white
set(f1, 'DefaultTextInterpreter', 'latex');

plot(start_y_calc,start_y_calc, LineStyle="--", LineWidth=1) 

hold on

plot(start_y_calc, output_3(1,:), Color='red', LineWidth=1.5)

plot(start_y_calc, output_3(2,:), Color='blue', LineWidth=1.5)

plot(start_y_calc, output_3(3,:), Color='yellow', LineWidth=1.5)

plot(start_y_calc, output_3(4,:), Color='black', LineWidth=1.5)

% plot(start_y_calc, output_3(5,:), Color='green', LineWidth=1.5)

ylabel('Apex height step i + 1 (m)');
xlabel('Apex height step i');
title('Return map');

ylim([start_y_calc(1)-0.1 start_y_calc(end)]);  % Set y-axis range

legend('y_{i+1} = y_i','\alpha = 66^\circ', '\alpha = 67^\circ', '\alpha = 68^\circ', '\alpha = 69^\circ', Location='best')

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f2)

com_position_1 = plot(output_4.x1(1,:), output_4.y1(1,:), Color='red', LineWidth=1);  % Center of mass position, 66 deg

hold on;

com_position_2 = plot(output_4.x2(1,:), output_4.y2(1,:), Color='blue', LineWidth=1);  % Center of mass position, 67 deg
com_position_3 = plot(output_4.x3(1,:), output_4.y3(1,:), Color='yellow', LineWidth=1);  % Center of mass position, 68 deg
com_position_4 = plot(output_4.x4(1,:), output_4.y4(1,:), Color='cyan', LineWidth=1);  % Center of mass position, 69 deg

initial_height = plot(output_4.x1(1,:), ones(size(output_4.y1(1,:))), LineStyle='--');    % Initial height

plot(output_4.x1(1,:), output_4.y_land1*ones(size(output_4.y1(1,:))), LineStyle='--', Color='red');    % Touchdown height
plot(output_4.x1(1,:), output_4.y_land2*ones(size(output_4.y1(1,:))), LineStyle='--', Color='blue');    % Touchdown height
plot(output_4.x1(1,:), output_4.y_land3*ones(size(output_4.y1(1,:))), LineStyle='--', Color='yellow');    % Touchdown height
plot(output_4.x1(1,:), output_4.y_land4*ones(size(output_4.y1(1,:))), LineStyle='--', Color='cyan');    % Touchdown height

plot(output_4.takeoff1(1), output_4.takeoff1(2), Marker='*');    % Takeoff coordinates
plot(output_4.takeoff2(1), output_4.takeoff2(2), Marker='x');    % Takeoff coordinates
plot(output_4.takeoff3(1), output_4.takeoff3(2), Marker='o');    % Takeoff coordinates
plot(output_4.takeoff4(1), output_4.takeoff4(2), Marker='square');    % Takeoff coordinates

plot(output_4.landing1(1), output_4.landing1(2), Marker='*');    % Takeoff coordinates
plot(output_4.landing2(1), output_4.landing2(2), Marker='x');    % Takeoff coordinates
plot(output_4.landing3(1), output_4.landing3(2), Marker='o');    % Takeoff coordinates
plot(output_4.landing4(1), output_4.landing4(2), Marker='square');    % Takeoff coordinates

legend('\alpha = 66^\circ', '\alpha = 67^\circ', '\alpha = 68^\circ', '\alpha = 69^\circ', 'initial height','y_{land}(66^\circ)','y_{land}(67^\circ)','y_{land}(68^\circ)','y_{land}(69^\circ)', Location='best')

title("COM position in one stride");
xlabel("x(t) (m)");
ylabel("y(t) (m)");

xlim([0,2]);
ylim([0.8,1.15]);

% xlim([0,3]);
% ylim([0.7,1.3]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);