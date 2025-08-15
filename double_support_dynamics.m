function dr = double_support_dynamics(t,r,leg_length,k,g,mass,beta_old,beta_new)

x = r(1);
y = r(2);

x_dot = r(3);
y_dot = r(4);

velocity = [x_dot;y_dot];

d = beta_new - beta_old;

p = (k/mass)*(-1 + leg_length/sqrt((x - beta_old)^2 + y^2));

q = (k/mass)*(-1 + leg_length/sqrt((x - d - beta_old)^2 + y^2));
        
x_ddot = p*(x - beta_old) + q*(x - d - beta_old);
y_ddot = -g + p*y + q*y;

acceleration = [x_ddot;y_ddot];

dr = [velocity;acceleration];

end