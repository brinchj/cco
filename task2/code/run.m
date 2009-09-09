% Make sure we got a clean environment to work in
close all;
clear all;

% Setup a default configuration
t      = [ 0 0 0; 1 1 1];
angles = [pi/4; pi/4; pi/4];

% Try to find end-effector position
e = f(t, angles);

% Get x and y coordinates
x = e(1);
y = e(2);

% Verify if f worked as we expected
if ( (x  + 1.7071) > 0.001 )
    error('x-test failed.');
end

if (  (y  - 1.7071) > 0.001 )
    error('y-test failed.');
end

figure(1);

clf;
hold on;
title('Inverse Kinematic Chain');
xlabel('X');
ylabel('Y');

subplot(2,2,1);
draw_chain( t, angles );
subplot(2,2,2);
draw_chain( t, angles );

theaxis = [-4 4 -4 4];
angles = ones(size(angles));
for n = 1:20
    % Newton
    subplot(2,2,1);
    
    [gx,gy] = ginput(1);
    plot(gx,gy,'bo');

    new_angles = nonlinear_newton([gx; gy; 1],t,angles);
    subplot(2,2,1);
    draw_chain( t, new_angles );
    
    axis(theaxis);
    
    % Levenberg
    subplot(2,2,2);
    plot(gx,gy,'bo');
    angles = levenberg_marquardt([gx; gy; 1],t,angles);
    subplot(2,2,2);
    draw_chain( t, angles );
    
    axis(theaxis);
end
hold off;
