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



fig = figure(1);

title('Inverse Kinematic Chain');

x = -1.3415;
y =  3.1415;

plotit('Nonlinear Newton',    @nonlinear_newton,    x, y, angles, 0, 3);
plotit('Levenberg Marquardt', @levenberg_marquardt, x, y, angles, 1, 3);
plotit('BFGS',                @bfgs,                x, y, angles, 2, 3);

%saveas(fig, 'graph.eps', 'eps2c');
