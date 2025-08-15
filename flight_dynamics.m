function dr = flight_dynamics(t,r,g)

x_dot = r(3);
y_dot = r(4);

velocity = [x_dot;y_dot];

x_ddot = 0;
y_ddot = -g;

acceleration = [x_ddot ; y_ddot];

dr = [velocity;acceleration];

end