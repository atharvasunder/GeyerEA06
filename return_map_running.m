%%  Initial conditions for poincare analysis

% Define a grid of attack angle and k values
attack_angle = linspace(66, 69, 4);
k = 20000; % linspace(0, 50000, 200);
no_of_y_vals = 200;

beta_old = 0;
beta_new = 0;

phase = 0;

% Initialize a matrix to store the results
output_3 = zeros(length(attack_angle), no_of_y_vals);

output_4.x1 = [];
output_4.y1 = [];   % Defined multiple variables because different ode initial conditions give outputs of different lengths, so cannot combine them to one array

output_4.x2 = [];
output_4.y2 = [];   % Creating a struct variable to store all output arrays
        
output_4.x3 = [];
output_4.y3 = [];

output_4.x4 = [];
output_4.y4 = [];

output_4.y_land1 = 0;
output_4.y_land2 = 0;
output_4.y_land3 = 0;
output_4.y_land4 = 0;

output_4.takeoff1 = 0;
output_4.takeoff2 = 0;
output_4.takeoff3 = 0;
output_4.takeoff4 = 0;

output_4.landing1 = 0;
output_4.landing2 = 0;
output_4.landing3 = 0;
output_4.landing4 = 0;

% Loop through all combinations
for i = 1:length(attack_angle)

    % Get the current values of attack_angle and k
    current_attack_angle = attack_angle(i);

    y_land = leg_length*sind(current_attack_angle);  % Height at which leg lands and leaves the ground (beginning and end of stance phase)

    start_y_calc = linspace(y_land, 2.1, no_of_y_vals);

    % Store final solution, takeoff coordinates
    [dummy,final_solution,takeoff_coordinates,landing_coordinates] = poincare_function_running(phase,leg_length,k,g,mass,y_land,current_attack_angle,beta_old,beta_new,t_start,t_end,t_span,initial_condition); 

    output_4.(['x', num2str(i)]) = final_solution(1,:);
    output_4.(['y', num2str(i)]) = final_solution(2,:);
    output_4.(['y_land', num2str(i)]) = y_land;
    output_4.(['takeoff', num2str(i)]) = takeoff_coordinates;
    output_4.(['landing', num2str(i)]) = landing_coordinates;

    for j = 1:size(start_y_calc,2)
        
        current_start_y = start_y_calc(j);

        current_start_vx = sqrt(2*system_energy/mass - 2*g*current_start_y);
        
        current_initial_condition = [initial_condition(1);current_start_y;current_start_vx;initial_condition(4)];

        % Store next height
        [output_3(i, j),dummy,takeoff_coordinates,landing_coordinates] = poincare_function_running(phase,leg_length,k,g,mass,y_land,current_attack_angle,beta_old,beta_new,t_start,t_end,t_span,current_initial_condition); 

    end
end

