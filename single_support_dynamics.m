function dr = single_support_dynamics(t,r,leg_length,k,g,mass,beta_new)

x = r(1);
y = r(2);

x_dot = r(3);
y_dot = r(4);

velocity = [x_dot;y_dot];

p = (k/mass)*(-1 + leg_length/sqrt((x - beta_new)^2 + y^2));
        
x_ddot = p*(x - beta_new);
y_ddot = -g + p*y;

acceleration = [x_ddot;y_ddot];

dr = [velocity;acceleration];

end