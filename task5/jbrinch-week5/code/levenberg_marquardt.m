function [ err, angles ] = levenberg_marquardt( goal, t, angles )
%
% input
%     g      : Goal position/vector (specified in homogenuous coordinates)
%     t      : A vector of fixed rod-link vectors
%     angles : A vector with joint angles
%     e      : Position/vector (specified in homogenuous coordinates)
% output
%     angles : The updated pose which will reach the specified goal position.

e  = [0;0;1];
endPoint = f(t, angles);

err  = [];

lambda   = 0.0001;

oldError = dot(goal-endPoint,goal-endPoint);

count = 0;
J = jacobian(t, angles, e);
while dot(J'*(goal-endPoint), J'*(goal-endPoint)) > 0.01 && count < 50
    % Compute new angles, endpoint and error
    r = goal - endPoint;
    J = jacobian(t, angles, e);

    % Keep doubling lambda until error is reduced
    newError = oldError + 1;
    while newError >= oldError
        % REJECT
        lambda = lambda * 2;
        delta = LMupdate(lambda, J, r);
        newAngles   = angles + delta;
        newEndPoint = f(t, newAngles);
        error = goal-newEndPoint;
        newError = dot(error, error);
    end

    % ACCEPT
    oldError = newError;
    angles   = newAngles;
    endPoint = newEndPoint;
    lambda = lambda / 2;

    % Remember observed error
    err = [ err oldError ];
    count = count + 1;
end

end
