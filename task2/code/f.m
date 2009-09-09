function [ e ] = f( t, angles, e )
% F The end-effector function
%   Given an initial configuration of a inverse kinematics chain
%   with a root fixed at the origin this function computes
%   the x-y position of the end-effector.
% input
%     t      : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
%     e      : Position/vector (specified in homogenuous coordinates)
% output
%     e : The end-effector position/vector given in root frame
if nargin < 3
  e = [0.0;  0.0; 1.0];
end

%
% TODO write code that compute e wrt. the root frame
%

% Generate transformation matrices
for i=1:length(angles)
    j = length(angles)-i+1;
    angle = angles(j);
    m = [ 
        cos(angle), -sin(angle), t(1,j);
        sin(angle), cos(angle),  t(2,j);
        0, 0, 1
        ];
    e = m * e;
end

if( ~isequal(e(3),0))
    e = e./e(3);
end
end

