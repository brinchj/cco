function [ J ] = jacobian( t, angles, e )
% JACOBIAN The jacobian function
%   Given an initial configuration of a inverse kinematics chain
%   with a root fixed at the origin this function computes
%   the Jacobian of the end-effector.
% input
%     t      : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
%     e      : Position/vector (specified in homogenuous coordinates)
% output
%     J : The Jacobian matrix
if nargin < 3
    e = [0.0;  0.0; 1.0];
end
N = length(angles)  % Number of joints/links in chain
J = zeros(3,N);
for i=N:-1:1
    % Compute position of end-effector wrt. i'th joint frame
    tt  = t(:,i+1:N);
    aa  = angles(i+1:N);
    e_i = f(tt,aa,e);
    % Compute differential of end-effector wrt. i'th frame
    angle = angles(i);
    ca    = cos( angle );
    sa    = sin( angle );
    dR  = [ -sa  -ca   0; ca -sa   0; 0 0 0 ];
    de_i  = dR*e_i;
    % Transform differential from i'th frame to root frame and store in
    % Jacobian
    tt = t(:,1:i-1);
    aa = angles(1:i-1);
    de_r = f(tt,aa,de_i);
    J(:,i) = de_r;
end
end
