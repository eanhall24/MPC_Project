function [x_des, u_des] = path_gen(t, t_start, t_final, r, h)

% [r_com_des, v_com_des, u_com_des] = rcom_traj(t, t_stance, t_stance_start, v_stance_init)

%constants
g = 9.81;
m = 32e-03;

%extract values from inputs
nt = length(t);
nx = 12;
nu = 5;
% t_start = t(1);
% t_final = t(end);
% t_step = .1; %t(2)-t(1);
% r = sqrt(x0(1)^2 + x0(5)^2);
% t_vec = t_start:t_step:t_final;
% theta_vec = linspace(0 , 2*pi , ((t_final-t_start)/t_step)+1 );
x_des = zeros(nx, nt);
% x_des(:,1) = x0;
% theta = atan2(x0(5),x0(1)) + linspace(0,2*pi,nt+1);
% dtheta = (theta(2)-theta(1))/t_step;
% dtheta = (2*pi)/(t_final - t_start);

for i = 1:(nt)

    if t(i) >= t_start % && t(i) <= t_final
        
        x_des(1,i) = r*cos(t(i)*((2*pi)/(t_final-t_start)));
        x_des(2,i) = -r*((2*pi)/(t_final-t_start))*sin(t(i)*((2*pi)/(t_final-t_start)));
        x_des(3,i) = 0;
        x_des(4,i) = 0;
        x_des(5,i) = r*sin(t(i)*((2*pi)/(t_final-t_start)));
        x_des(6,i) = r*((2*pi)/(t_final-t_start))*cos(t(i)*((2*pi)/(t_final-t_start)));
        x_des(7,i) = 0;
        x_des(8,i) = 0;
        x_des(9,i) = h + t(i)*.1; %constant height
        x_des(10,i) = 0;
        x_des(11,i) = 0;
        x_des(12,i) = 0;
        
    else
        
        x_des(1,i) = r;
        x_des(2,i) = 0;
        x_des(3,i) = 0;
        x_des(4,i) = 0;
        x_des(5,i) = 0;
        x_des(6,i) = 0;
        x_des(7,i) = 0;
        x_des(8,i) = 0;
        x_des(9,i) = h; %constant height
        x_des(10,i) = 0;
        x_des(11,i) = 0;
        x_des(12,i) = 0;
        
    end
    
u_des = .25*m*g*ones(nu, nt);
u_des(5,:) = g;

end