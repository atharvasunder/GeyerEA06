%% Function that takes input as the initial (x_rel,y) returns (x_rel,y) after 1 step

function [x_rel , y] = poincare_function_walking(phase,leg_length,k,g,mass,y_land,attack_angle,beta_old,beta_new,t_start,t_end,t_span,initial_condition)

    final_times = [];   % Initialize arrays to store the solution
    final_solution = [];
    
    takeoff_coordinates = [];
    landing_coordinates = [];

    og_initial_condition = initial_condition;
    
    y = 0;    % Next apex height; storing it as a dummy variable because I am assigning the actual value to y in an if condition
    x_rel = 0;

    % Loop over time until the entire time span is covered
    while t_start < t_end
        
        % ODE45 settings
        tolerance_val = 1e-12;
        options = odeset('AbsTol',tolerance_val,'RelTol',tolerance_val,'Events', @(t, y) switching_conditions(t, y, phase, y_land, beta_old, beta_new, leg_length));
    
        if phase == 1
    
            ode = ode45(@(t,r)single_support_dynamics(t,r,leg_length,k,g,mass,beta_new),t_span,initial_condition,options);
    
        elseif phase == 2
    
            ode = ode45(@(t,r)double_support_dynamics(t,r,leg_length,k,g,mass,beta_old,beta_new),t_span,initial_condition,options);
        
        elseif phase == 0
    
            ode = ode45(@(t,r)flight_dynamics(t,r,g),t_span,initial_condition,options);
    
        end
    
        T = ode.x;  % Time spent at the current phase
        Y = ode.y;  % State values
        TE = ode.xe;    % Time of event
        YE = ode.ye;
        IE = ode.ie;   % Indices of the events that were triggered (e.g., 1 for the first event, 2 for the second event)
    
        % Store the results
        final_times = [final_times, T(2:end)];    % Append time points, till end - 1 to avoid duplicates while combining arrays
        final_solution = [final_solution, Y(:, 2: end)];    % Append state values
    
        % Check if an event occurred (condition translates to if is not empty)
        if ~isempty(TE)
    
            if phase == 1
    
                if IE(end) == 1
    
                    phase = 2;
    
                    x_land = Y(1,end);   % x position of COM at which landing occurs
                    beta_old = beta_new;
                    beta_new = x_land + leg_length*cosd(attack_angle);
    
                elseif IE(end) == 2
                    
                    x_rel = og_initial_condition(1);
                    y = og_initial_condition(2);
                    break   % We dont want phase zero right here

                    % phase = 0;
                    % 
                    % x_takeoff = Y(1,end);   % x position of COM at which landing occurs
                    % y_takeoff = Y(2,end);
                     
                    % takeoff_coordinates = [x_takeoff,y_takeoff];
    
                end
    
            elseif phase == 2
    
                if IE(end) == 1
    
                    % Do nothing as this is just written that way for
                    % consistency in indices
    
                elseif IE(end) == 2
    
                    phase = 1;
    
                end
    
            elseif phase == 0
                
                x_rel = og_initial_condition(1);
                y = og_initial_condition(2);
                break
    
                % if IE(end) == 1
                % 
                %     phase = 1;
                % 
                %     x_land = Y(1,end);   % x position of COM at which landing occurs
                %     y_land = Y(2,end);
                %     beta_new = x_land + leg_length*cosd(attack_angle);
                % 
                %     landing_coordinates = [x_land,y_land];
                % 
                % end
    
            end
    
            if IE(end) == 3 % When vy = 0 and com is at apex
                
                x_apex = Y(1,end);
                y_apex = Y(2,end);

                y = y_apex;
                x_rel = x_apex - beta_new;  % Always ends a step at single support
    
                break
            end

            if IE(end) == 4 % If vx = 0 it is an incomplete step
                x_rel = og_initial_condition(1);
                y = og_initial_condition(2);
                break
            end

            if IE(end) == 5 % If vx = 0 it is an incomplete step
                x_rel = og_initial_condition(1);
                y = og_initial_condition(2);
                break
            end
    
            % Update initial conditions after event
            initial_condition = Y(:,end);
    
            % Update the time span for the next iteration
            t_start = T(end);
            t_span = [t_start t_end];
    
        else
            t_start = t_end;   % Only for ending of t_span where there is no event invoked
    
        end
    end

end