function [ angles ] = nonlinear_newton( g, t, angles )
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

iter = zeros(1,50);
err  = zeros(1,50);
for i=1:50
    da = pinv(jacobian(t, angles, e))*(g - ep);
    angles = angles + da;
    ep = f(t, angles);
    error = g-ep;
    err(i)  = log(dot(error,error));
    iter(i) = i;
end

subplot(2,2,3);
plot(iter, err);
axis([0 50 -10 2]);

end
