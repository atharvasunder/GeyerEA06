f1 = figure;
set(f1, 'Color', 'w'); % Set background color to white
set(f1, 'DefaultTextInterpreter', 'latex');

f2 = figure;
set(f2, 'Color', 'w'); % Set background color to white
set(f2, 'DefaultTextInterpreter', 'latex');

f3 = figure;
set(f3, 'Color', 'w'); % Set background color to white
set(f3, 'DefaultTextInterpreter', 'latex');

f4 = figure;
set(f4, 'Color', 'w'); % Set background color to white
set(f4, 'DefaultTextInterpreter', 'latex');

t = 0;      % Instantous time, initialized to 0

t_values = final_times;
x_values = final_solution(1,:);
y_values = final_solution(2,:);
vx_values = final_solution(3,:);
vy_values = final_solution(4,:);
ax_values = diff(vx_values)./diff(t_values);  % Reduces the length of array by 1 because differences are taken
ax_values = [ax_values , ax_values(end)];
ay_values = diff(vy_values)./diff(t_values);
ay_values = [ay_values , ay_values(end)];

figure(f1)   % Creates a window where the graph will be plotted

subplot(3,2,1)

x_vs_time = plot(t_values, x_values, 'blue','linewidth',1.5);

title("position x component vs time");
xlabel("t");
ylabel("x (m)");

xlim([0,t_end]);
ylim([0,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,2)

y_vs_time = plot(t_values, y_values, 'blue','linewidth',1.5);

hold on

disp(y_land)
plot(t_values, y_land*ones(size(t_values)), 'red')

title("position y component vs time");
xlabel("t");
ylabel("y (m)");

xlim([0,t_end]);
ylim([0,1.5]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,3)

vx_vs_time = plot(t_values, vx_values, 'red','linewidth',1.5);

title("velocity x component vs time");
xlabel("t");
ylabel("vx (m/s)");

xlim([0,t_end]);
ylim([0,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,4)

vy_vs_time = plot(t_values, vy_values, 'red','linewidth',1.5);

title("velocity y component vs time");
xlabel("t");
ylabel("vy (m/s)");

xlim([0,t_end]);
ylim([-15,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,5)

ax_vs_time = plot(t_values, ax_values, 'green','linewidth',1.5);

title("acceleration x component vs time");
xlabel("t");
ylabel("ax (m/s2)");

xlim([0,t_end]);
ylim([-20,20]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(3,2,6)

ay_vs_time = plot(t_values, ay_values, 'green','linewidth',1.5);

hold on

plot(t_values, -g*ones(size(t_values)), 'red')

title("acceleration y component vs time");
xlabel("t");
ylabel("ay (m/s2)");

xlim([0,t_end]);
ylim([-20,20]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f2)

TE = KE + PE;   % Total energy

subplot(2,1,1)

KE_vs_t = plot(t_values,KE,'blue','linewidth',1,DisplayName='KE');

hold on

PE_vs_t = plot(t_values,PE,'red','linewidth',1,DisplayName='PE');

TE_vs_t = plot(t_values,TE,'yellow','linewidth',1.5,DisplayName='TE');

title("Energy variation in walking/running");
xlabel("t (s)");
ylabel("KE , PE , TE");

legend('show')

% xlim([-15,15]);
% ylim([-15,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

subplot(2,1,2)

delta_KE_x = KE_x - mean(KE_x);
delta_PE_g = PE_g - mean(PE_g);

delta_KE_x_vs_t = plot(t_values,delta_KE_x,'blue','linewidth',1,DisplayName='\delta KE_x');

hold on

delta_PE_g_vs_t = plot(t_values,delta_PE_g,'red','linewidth',1,DisplayName='\delta PE_g');

title("Energy variation in walking/running");
xlabel("t (s)");
ylabel("Forward KE , Gravitational PE");

legend('show')

% xlim([-15,15]);
% ylim([-15,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

figure(f3)

% subplot(2,1,1)

grf_x_vs_t = plot(t_values,grf_x,'blue','linewidth',1,DisplayName='GRF_x');

hold on

grf_y_vs_t = plot(t_values,grf_y,'red','linewidth',1,DisplayName='GRF_y');

title("Ground reaction forces");
xlabel("t (s)");
ylabel("GRF (bw)");

legend('show')

% xlim([-15,15]);
% ylim([-15,15]);

grid on;
box on;

set(gca, 'FontSize', 12);
set(gca, 'Box', 'on');
set(gca, 'LineWidth', 1.2);

% subplot(2,1,2)
% 
% spring_length_vs_time = plot(t_values , spring_l,'blue','linewidth',1);
% 
% title("stance spring length vs time");
% xlabel("t (s)");
% ylabel("Length (m)");
% 
% % legend('show')
% 
% % xlim([-15,15]);
% % ylim([-15,15]);
% 
% grid on;
% box on;
% 
% set(gca, 'FontSize', 12);
% set(gca, 'Box', 'on');
% set(gca, 'LineWidth', 1.2);


figure(f4)

hold on;

% Plot each time interval at y = 1
for i = 1:length(time_0)
    % Extract the current time values
    time_values = time_0{i};
    
    % Plot the time values as horizontal lines at y = 0
    y = zeros(size(time_values));
    plot(time_values, y,'red', 'LineWidth', 1.5);
end

% Plot each time interval at y = 1
for i = 1:length(time_1)
    % Extract the current time values
    time_values = time_1{i};
    
    % Plot the time values as horizontal lines at y = 0
    y = ones(size(time_values));
    plot(time_values, y, 'yellow', 'LineWidth', 1.5);
end

% Plot each time interval at y = 1
for i = 1:length(time_2)
    % Extract the current time values
    time_values = time_2{i};
    
    % Plot the time values as horizontal lines at y = 0
    y = 2*ones(size(time_values));
    plot(time_values, y,'blue', 'LineWidth', 1.5);
end

title("Time spent in each phase");
xlabel("t (s)");
ylabel("Phase");

grid on;
box on;
hold off;