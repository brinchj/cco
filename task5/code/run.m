% Make sure we got a clean environment to work in
close all;
clear all;

% Setup a default configuration
num_angles = 100;
t      = [ zeros(1,num_angles); ones(1,num_angles) ];
angles = ones(num_angles,1) * pi/4;

% Try to find end-effector position
e = f(t, angles);

% Get x and y coordinates
x = e(1);
y = e(2);

% Verify if f worked as we expected
%if ( (x  + 1.7071) > 0.001 )
%    error('x-test failed.');
%end

%if (  (y  - 1.7071) > 0.001 )
%    error('y-test failed.');
%end


% points to test
a = num_angles;
points = [
    -0.5*a 0.29*a
    %.6501*a -.5502*a
    %0.16*a 0.33*a
    %-0.0331*a -0.4115*a;
    %-0.33*a -1*a;           % outside reach
    %0.221*a -0.90*a
];


for i = 1:length(points)
    fig = figure(i);

    x = points(i,1);
    y = points(i,2);

    plotit('Nonlinear Newton',    @nonlinear_newton,    t, x, y, angles, 0, 4);
    plotit('Levenberg Marquardt', @levenberg_marquardt, t, x, y, angles, 1, 4);
    plotit('BFGS',                @bfgs,                t, x, y, angles, 2, 4);
    plotit('DogLeg',              @dogleg,              t, x, y, angles, 3, 4);
end
%saveas(fig, 'graph.eps', 'eps2c');
