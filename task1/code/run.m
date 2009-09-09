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
subplot(2,2,1);
axis([0,10,0,10]);

clf;
hold on;
title('Inverse Kinematic Chain');
xlabel('X');
ylabel('Y');
draw_chain( t, angles );
for n = 1:20
    [gx,gy] = ginput(1);
    plot(gx,gy,'bo');
    angles = nonlinear_newton([gx; gy; 1],t,angles);
    draw_chain( t, angles );
end
hold off;
