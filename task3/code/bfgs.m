function [ reserr angles ] = bfgs( goal, t, angles )
%
% input
%     g      : Goal position/vector (specified in homogenuous coordinates)
%     t      : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
%     e      : Position/vector (specified in homogenuous coordinates)
% output
%     angles : The updated pose which will reach the specified goal
%     position.

e  = [0;0;1];

reserr = [];

% gradient
J  = jacobian(t, angles, e);

% compute first endpoint
ep = f(t, angles, e);

% approximation of inverse hessian
Bi = eye(size(J));

count = 0;
gradient = J' * (ep-goal);
while dot(gradient,gradient) > 0.001 && count < 50
    % -gradient works as search direction!
    %pk = -transpose(J) * (ep-goal);


    % BFGS Search Direction
    p = - Bi * gradient;


    % Armijo Backtrack
    alpha  = armijo_backtrack(@f, t, angles, J, p, goal);

    % Update EndPoint
    newAngles = angles + alpha*p;
    newEp = f(t, newAngles);

    % Compute Error
    error  = goal-ep;
    error  = dot(error, error);
    reserr = [reserr log(error)];

    % Update Gradient and Angles
    newJ = jacobian(t, newAngles, e);

    % Update Approximation of Inverse Hessian
    y   = newJ'*(newEp-goal) - gradient;
    %vA  = (y*y')/(y'*p);
    %tmp = Bi*p;
    %vB  = (tmp*tmp')/(p'*Bi*p);
    vA = (p*p')*(p'*y + y'*Bi*y)/((p'*y)^2);
    vB = (Bi*y*p' + p*y'*Bi)/(p'*y);
    Bi = Bi + vA - vB;

    J = newJ;
    ep = newEp;
    angles = newAngles;

    gradient = J' * (ep-goal);

    count = count + 1;
end


end
