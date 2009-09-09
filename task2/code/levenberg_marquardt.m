function [ angles ] = levenberg_marquardt( goal, t, angles )
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

iter = zeros(1,50);
err  = zeros(1,50);

lambda   = 0.0001;

oldError = dot(goal-endPoint,goal-endPoint);

for i = 1:50
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
    err(i)  = log(oldError);
    iter(i) = i;
end

subplot(2,2,4);
plot(iter, err);
axis([0 50 -10 2]);
subplot(2,2,1);

end
