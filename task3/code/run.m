% Make sure we got a clean environment to work in
close all;
clear all;

% Setup a default configuration
num_angles = 10;
t      = [ zeros(1,num_angles); ones(1,num_angles) ];
angles = [ ones(num_angles,1) * pi/4 ];

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
points = [
    1 1; % simple points
    0.1415 3.1415;    % funky points
    2.1011 -1.2345;
    .7501 -.6502;
    3 3               % outside reach
];


for i = 1:1 %length(points)
    fig = figure(i);

    x = points(i,1);
    y = points(i,2);

    plotit('Nonlinear Newton',    @nonlinear_newton,    t, x, y, angles, 0, 3);
    plotit('Levenberg Marquardt', @levenberg_marquardt, t, x, y, angles, 1, 3);
    plotit('BFGS',                @bfgs,                t, x, y, angles, 2, 3);
end
%saveas(fig, 'graph.eps', 'eps2c');
