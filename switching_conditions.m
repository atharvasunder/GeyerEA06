function [value, isterminal, direction] = switching_conditions(t, y, phase, y_land, beta_old, beta_new, leg_length)
    x_com = y(1);
    y_com = y(2);

    vx_com = y(3);
    vy_com = y(4);
    
    if phase == 1

        instantaneous_leg_length = sqrt((x_com - beta_new)^2 + y_com^2);   % Gives the instantaneous length of the leg that is going to takeoff
        
        value(1) = y_com - y_land;   % When y(2) - y_land changes sign and is decreasing, trigger landing event to start double support
        value(2) = instantaneous_leg_length - leg_length;  % When spring length = leg length, switch from single support to flight

    elseif phase == 0
        
        instantaneous_leg_length = leg_length;
        
        value(1) = y_com - y_land;   % When y(2) - y_land changes sign and is decreasing, trigger landing event to start single support
    
    elseif phase == 2

        instantaneous_leg_length = sqrt((x_com - beta_old)^2 + y_com^2);   % Gives the instantaneous length of the leg that is going to takeoff
        value(1) = 0;    % Irrelevant here, just added as a dummy to keep the indexing consistent
        value(2) = instantaneous_leg_length - leg_length;  % When spring length = leg length, switch from single support to flight

    end
    
    value(3) = vy_com;  % When apex is reached (completion of a step), stop simulation
    value(4) = vx_com;  % If com halts, stop simulation
    value(5) = y_com;   % If COM falls to ground, stop

    isterminal(1) = 1;       % Stop the integration when event occurs
    isterminal(2) = 1;
    isterminal(3) = 1;
    isterminal(4) = 1;
    isterminal(5) = 1;

    direction(1) = -1;        % Detects zero-crossings only when the event function is decreasing, i.e., when it crosses zero from positive to negative.
    direction(2) = 1;         % Detects zero-crossings only when the event function is increasing, i.e., when it crosses zero from negative to positive.
    direction(3) = -1;
    direction(4) = -1;
    direction(5) = -1;

end