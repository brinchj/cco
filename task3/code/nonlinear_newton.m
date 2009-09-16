function [ err, angles ] = nonlinear_newton( g, t, angles )
%
% input
%     g      : Goal position/vector (specified in homogenuous coordinates)
%     t      : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
%     e      : Position/vector (specified in homogenuous coordinates)
% output
%     angles : The updated pose which will reach the specified goal position.

e  = [0;0;1];
ep = f(t, angles);

err  = [];

J = 1;
count = 0;
while dot(J'*(g-ep),J'*(g-ep)) > 0.001 && count < 50
    J = jacobian(t, angles, e);
    da = pinv(J)*(g-ep);
    angles = angles + da;
    ep = f(t, angles);
    error = g-ep;
    err = [err log(dot(error,error))];
    count = count + 1;
end

end
