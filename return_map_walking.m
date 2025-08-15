k = 13969.44;   % N/m
k_tilda = k*leg_length/(mass*g);    % Dimensionless parameter

E = 816.192;    % Desired system energy
E_tilda = E/(mass*g*leg_length);

x_tilda_rel = linspace(-cosd(attack_angle) , cosd(attack_angle) , 30);
y_tilda = linspace(sind(attack_angle) , 1 , 60);

% ceck2 = x_tilda_rel
% 
% check3 = y_tilda

phase = 1;

input = cell(length(y_tilda), length(x_tilda_rel));

output = cell(length(y_tilda), length(x_tilda_rel));   % Creating a cell array to store coordinates

for i = 1:length(x_tilda_rel)   % At every row
    for j = 1:length(y_tilda)   % For each column in the respective row

        input{j,i} = [x_tilda_rel(i) , y_tilda(j)];

    end

end

% Loop through the values

for i = 1:length(x_tilda_rel)   
    for j = 1:length(y_tilda)

        % Start conditions
        x_val = x_tilda_rel(i)*leg_length;  % We only study 1 step, so assuming foot placement point is x=0
        y_val = y_tilda(j)*leg_length;  
    
        l_spring = sqrt(x_val^2 + y_val^2); % Start length of spring

        if l_spring > leg_length
            
            output{j,i} = input{j,i};
            continue

        end

        if (2*(E/mass - g*y_val) - (k/mass)*(l_spring - leg_length)^2) > 0

            vx_val = sqrt(2*(E/mass - g*y_val) - (k/mass)*(l_spring - leg_length)^2);

        else

            output{j,i} = input{j,i};

            continue % Skip this initial condition if there is no velocity that satisfies the energy requirement

        end
        
        vy_val = 0; % By definition

        initial_condition = [x_val ; y_val ; vx_val ; vy_val];

        [x_result , y_result] = poincare_function_walking(phase,leg_length,k,g,mass,y_land,attack_angle,beta_old,beta_new,t_start,t_end,t_span,initial_condition);

        x_tilda_rel_result = x_result/leg_length;
        y_tilda_result = y_result/leg_length;
        
        output{j,i} = [x_tilda_rel_result , y_tilda_result];

    end
end

check = output

% Prepare data for the quiver plot
[X_poincare, Y_poincare] = meshgrid(x_tilda_rel, y_tilda); % Grid for quiver plot

U = zeros(size(X_poincare)); % x-component of vectors
V = zeros(size(Y_poincare)); % y-component of vectors

% Compute vector components
for i = 1:length(x_tilda_rel)
    for j = 1:length(y_tilda)
        input_point = input{j, i};   % Input point (x, y)
        output_point = output{j, i}; % Output point (x, y)
        
        vector = output_point - input_point;

        magnitude = norm(vector);

        if magnitude > 0
            unit_vector = vector/(150*magnitude);
        else
            unit_vector = vector;
        end
 
        U(j, i) = unit_vector(1); % x-component of vector
        V(j, i) = unit_vector(2); % y-component of vector
    end
end

