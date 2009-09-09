function draw_chain( t, angles )
% DRAW_CHAIN Draw an inverse kinematic chain
%   Given an initial configuration of a inverse kinematics chain
%   with a root fixed at the origin this function draws
%   an illustration of the pose of the chain.
% input
%     t     : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
N = length(angles);  % Number of joints/links in chain
x = 0;
y = 0;
for i=1:N
    tt = t(:,1:i);
    aa = angles(1:i);
    C = f(tt,aa, [0.0;  0.0; 1.0]);
    new_x = C(1);
    new_y = C(2);
    line([x new_x],[y new_y],'Color','k','LineWidth',2);
    x = new_x;
    y = new_y;
end
end
