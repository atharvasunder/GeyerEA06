k_range = linspace(0, 50000, 100);
attack_angle_range = linspace(50, 80, 50);
energy_range = linspace(770, 870, 50);

phase = 1;  % Assume we start at phase 1 always

domain_matrix = zeros(length(k_range), length(attack_angle_range), length(energy_range));

tic
ticBytes(gcp);

for i_k = 1:length(k_range)
    k = k_range(i_k)
    for i_attack_angle = 1:length(attack_angle_range)
        attack_angle = attack_angle_range(i_attack_angle);
        % Compute y_min and y_max for this attack_angle
        y_min = leg_length * sind(attack_angle);
        y_max = leg_length;
        
        parfor i_energy = 1:length(energy_range)
            energy = energy_range(i_energy);
            % Generate 50 y-values between y_min and y_max
            y_values = linspace(y_min, y_max, 50);

            result = false;
            
            % Loop through each y-value
            for i_y = 1:length(y_values)
                
                y = y_values(i_y);

                spring_length = y;
                
                if (2*energy/mass - (k*(spring_length - leg_length)^2)/mass - 2*g*y) > 0
                    
                    vx = sqrt(2*energy/mass - (k*(spring_length - leg_length)^2)/mass - 2*g*y);

                else

                    break

                end

                initial_condition = [0 ; y ; vx ; 0];

                result = stable_gait_calculator(phase,leg_length,k,g,mass,y_min,attack_angle,beta_old,beta_new,t_start,t_end,initial_condition);

                if result == true

                    domain_matrix(i_k , i_attack_angle , i_energy) = 1;
                    break   % Exit the loop if there is a fixed point (we assume it is stable)

                end
                
            end
        end
    end
end

tocBytes(gcp)
toc