clear
clc
close all
% Solve DE numerically using ode45
m_num   = -1;
b_num = 5;
l_num = 0.5;
h_num = 2;
k_num = 1;
m_b_num = 0.5;
m_c_num = 0.5;
S_num = 2;
theta_num = atan(-m_num);
g_num   = 9.81;
tspan   = [0 10];
Y0      = [4 0 0 0];
options = odeset('RelTol',1e-6);
% Use created .m file to solve DE 
[t, Y]  = ode45(@PC2_sys,tspan,Y0,options,m_num,b_num,l_num,h_num,k_num,m_b_num,m_c_num,g_num,S_num);

% Position vectors
r_c = @(rz)[rz*cos(theta_num);b_num-rz*sin(theta_num)];
r_b = @(rz, phi) [rz*cos(theta_num)+h_num*sin(theta_num)+l_num*sin(phi-theta_num);b_num-rz*sin(theta_num)+h_num*cos(theta_num)-l_num*cos(phi-theta_num)];
% r_k = @(rz) [rz*cos(theta_num)+b_num/m_num;b_num-rz*sin(theta_num)];
r_z = @(rz) [rz*cos(theta_num);-rz*sin(theta_num)];

% Draw the figure
figure

% Plot options (change if necessary)
DEBUG = false;

n_points = length(Y(:,1));
for k=1:n_points
    % Wipe the slate clean
    clf

    % Plot inclined plane
    line_x = linspace(-30, 30, n_points);
    line_y = m_num * line_x + b_num;
    plot(line_x, line_y, 'k', 'LineWidth', 3)
    

    
    % Body C
    hold on
    posC = r_c(Y(k,1));
    quiver(0, 0, posC(1), posC(2), 'Color', '#0072BD', 'LineWidth', 2);
    plot(posC(1), posC(2), 's', 'MarkerSize', 30, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', '#BBBBBB')
    
    % Body B
    hold on
    posB = r_b(Y(k,1), Y(k,3));
    quiver(0, 0, posB(1), posB(2), 'Color', '#7E2F8E', 'LineWidth', 2);
    plot(posB(1), posB(2), 'o', 'MarkerSize', 20, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', '#BBBBBB');

    
    
    if (DEBUG)
        % r0 vector
        hold on
        quiver(0, 0, -b_num/m_num, 0, 'Color', '#A2142F', 'LineWidth', 2);

        % rk vector
        % posK = r_k(Y(k,1));
        % hold on
        % quiver(-b_num/m_num, 0, posK(1), posK(2), 'Color', '#EDB120', 'LineWidth', 2);

        % ry vector
        hold on
        posY = [0;b_num];
        quiver(0, 0, posY(1), posY(2), 'Color', '#77AC30', 'LineWidth', 2);

        % rz vector
        posZ = r_z(Y(k,1));
        hold on
        quiver(posY(1), posY(2), posZ(1), posZ(2), 'Color', '#00FFFF', 'LineWidth', 2);
    end
    
    
    % Decorate the plot
    grid on
    xlabel('x')
    ylabel('y')
    title(["Particle at frame ", k])

    % Force matlab to draw the image
    drawnow
end

