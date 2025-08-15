clc;
close all;
clear;

%% System Parameters

leg_length = 1;     % In meters
g = 9.81;   % m/s^2
k = 13969.44;     % N/m
mass = 80;   % Mass of runner, kg

k_tilda = k*leg_length/(mass*g);    % Dimensionless parameter

E = 816.192;    % Desired system energy

E_tilda = E/(mass*g*leg_length);

%% Initial conditions

attack_angle = 69;  % In degrees
start_x = 0;
start_y = 0.965;
% start_vx = 1;   % m/s
start_vy = 0;
start_vx = sqrt(2*(E/mass - g*start_y) - (k/mass)*(start_y - leg_length)^2)

% system_energy = 0.5*mass*start_vx^2 + mass*g*start_y + 0.5*k*(start_y - leg_length)^2   % Total energy of system

start_KE = 0.5*mass*start_vx^2;

start_PE = mass*g*start_y + 0.5*k*(start_y - leg_length)^2;

% beta is the x coordinate of the foot placement point
beta_old = 0;   % This is an assumption, we always start with the spring vertical
beta_new = 0;

initial_condition_og = [start_x;start_y;start_vx;start_vy];

%% Mode switching conditions from single support to double support

y_land = leg_length*sind(attack_angle)  % Height at which leg lands and leaves the ground (beginning and end of stance phase)

phase = 1;  % 1: single support, 2: double support, 0: flight

%% Defining simulation duration parameters

animation_speed = 0.1;    % Animation speed

t_start = 0;
t_end = 20;

t_span = [t_start,t_end];     % Inputted into ode45, the span over which the numerical solution is computed

%% Return map analysis

% return_map_running

% return_map_walking

%% Computing walking and running domains

% parameter_domain_calculation

%% ODE solution

final_times = [];   % Initialize arrays to store the solution
final_solution = [];

PE = [];       % Energy terms
KE = [];

KE_x = [];  % Forward KE
PE_g = [];  % Gravitational PE

grf_x = [];     % Ground reaction forces
grf_y = [];

spring_l = [];  % Spring length

time_1 = {};
time_2 = {};    % Using a cell array as each timespan will have a different length
time_0 = {};    % For time spent in each phase

apex_coordinates = {[start_x/leg_length , start_y/leg_length]};  % Stores all apex coordinates (delta_x,y) at each apex of the motion up to 10 steps

% Loop over time until the entire time span is covered
while t_start < t_end

    % ODE45 settings
    tolerance_val = 1e-12;
    options = odeset('AbsTol',tolerance_val,'RelTol',tolerance_val,'Events', @(t, y) switching_conditions(t, y, phase, y_land, beta_old, beta_new, leg_length));

    if phase == 1

        ode = ode45(@(t,r)single_support_dynamics(t,r,leg_length,k,g,mass,beta_new),t_span,initial_condition_og,options);

    elseif phase == 2

        ode = ode45(@(t,r)double_support_dynamics(t,r,leg_length,k,g,mass,beta_old,beta_new),t_span,initial_condition_og,options);

    elseif phase == 0

        ode = ode45(@(t,r)flight_dynamics(t,r,g),t_span,initial_condition_og,options);

    end

    T = ode.x;  % Time spent at the current phase
    Y = ode.y;  % State values
    TE = ode.xe;    % Time of event
    YE = ode.ye;
    IE = ode.ie;   % Indices of the events that were triggered (e.g., 1 for the first event, 2 for the second event)

    % Store the results
    final_times = [final_times, T(2:end)];    % Append time points, till end - 1 to avoid duplicates while combining arrays
    final_solution = [final_solution, Y(:, 2: end)];    % Append state values

    if phase == 1

        time_1{end + 1} = T;

    elseif phase == 2   % Store time spent at each contact mode

        time_2{end + 1} = T;

    elseif phase == 0

        time_0{end + 1} = T;

    end

    temp_y = Y(:, 1: end - 1);  % Stores all the states solved for in this ode run

    for i = 1:size(temp_y,2)    % loop through all columns

        x_com = temp_y(1,i);
        y_com = temp_y(2,i);
        vx_com = temp_y(3,i);
        vy_com = temp_y(4,i);

        if phase == 1

            spring_length = sqrt((x_com - beta_new)^2 + y_com^2);   % Gives the instantaneous length of the leg that is on ground
            state_PE = 0.5*k*(spring_length - leg_length)^2 + mass*g*y_com;

            state_grf_x = k*(leg_length - spring_length)*(-beta_new + x_com)/spring_length;  % Ground reaction force magnitude
            state_grf_y = k*(leg_length - spring_length)*y_com/spring_length ;

        elseif phase == 2

            spring_length = sqrt((x_com - beta_old)^2 + y_com^2);  % Back leg
            front_spring_length = sqrt((x_com - beta_new)^2 + y_com^2); % Leading leg
            state_PE = 0.5*k*(spring_length - leg_length)^2 + 0.5*k*(front_spring_length - leg_length)^2 + mass*g*y_com;

            state_grf_x = (k*(leg_length - spring_length)*(-beta_old + x_com)/spring_length + k*(leg_length - front_spring_length)*(-beta_new + x_com)/front_spring_length);
            state_grf_y = (k*(leg_length - spring_length)*y_com/spring_length + k*(leg_length - front_spring_length)*y_com/front_spring_length);

        elseif phase == 0

            spring_length = leg_length;   % In the air, so spring is fully expanded
            state_PE = 0.5*k*(spring_length - leg_length)^2 + mass*g*y_com;

            state_grf_x = k*(leg_length - spring_length)*(-beta_new + x_com)/spring_length;  % Ground reaction force magnitude
            state_grf_y = k*(leg_length - spring_length)*y_com/spring_length ;

        end

        state_KE = 0.5*mass*(vx_com^2 + vy_com^2);
        state_KE_x = 0.5*mass*(vx_com^2);
        state_PE_g =  mass*g*y_com;

        KE = [KE , state_KE];
        PE = [PE , state_PE];

        KE_x = [KE_x , state_KE_x];
        PE_g = [PE_g , state_PE_g];

        grf_x = [grf_x , state_grf_x/(mass*g)];     % Normalizing for different weights
        grf_y = [grf_y , state_grf_y/(mass*g)];

        spring_l = [spring_l , spring_length];

    end

    % Check if an event occurred (condition translates to if is not empty)
    if ~isempty(TE)

        if phase == 1

            if IE(end) == 1

                phase = 2;

                x_land = Y(1,end);   % x position of COM at which landing occurs
                beta_old = beta_new;
                beta_new = x_land + leg_length*cosd(attack_angle);

            elseif IE(end) == 2

                phase = 0;

            end

        elseif phase == 2

            if IE(end) == 1

                % Do nothing as this is just written that way for
                % consistency in indices

            elseif IE(end) == 2

                phase = 1;

            end

        elseif phase == 0

            if IE(end) == 1

                phase = 1;

                x_land = Y(1,end);   % x position of COM at which landing occurs
                beta_new = x_land + leg_length*cosd(attack_angle);

            end

        end

        if IE(end) == 3 % When vy = 0 and com is at apex

            x_apex = Y(1,end);
            y_apex = Y(2,end);

            x_rel_apex = x_apex - beta_new;

            x_tilda_rel_apex = x_rel_apex/leg_length;
            y_tilda_apex = y_apex/leg_length;

            if length(apex_coordinates) < 100    % Calculate trajectry over 10 steps

                apex_coordinates{end + 1} = [x_tilda_rel_apex , y_tilda_apex];

            else

                break

            end

            % break
        end

        % if IE(end) == 4 % When vy = 0 and com is at apex
        % 
        %     x_apex = Y(1,end);
        %     y_apex = Y(2,end);
        % 
        %     break
        % end

        % Update initial conditions after event
        initial_condition_og = Y(:,end);

        % Update the time span for the next iteration
        t_start = T(end);
        t_span = [t_start t_end];

    else
        t_start = t_end;   % Only for ending of t_span where there is no event invoked

    end
end

% check4 = apex_coordinates

%% Plots and animation

plots

% plot_poincare_running

% plot_domains

% plot_poincare_walking