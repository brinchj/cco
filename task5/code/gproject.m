function [ reserr angles ] = gproject( goal, t, angles, bounds)
    if nargin < 4
        bound  = ones(length(angles),1) * 2*pi;
        bounds = [ -bound bound ];
    end

    reserr = [];

    % gradient
    J  = jacobian(t, angles);
    % compute first endpoint
    ep = f(t, angles);

    count = 0;
    gradient = J' * (ep-goal);
    while dot(gradient,gradient) > 0.01 && count < 200
        % Gradient Descent
        p = -gradient;

        % Armijo Backtrack wrt. lower and uppers bounds
        newAngles = gproject_backtrack(@f, t, angles, J, p, goal, bounds);

        % Update EndPoint
        newEp = f(t, newAngles);

        % Compute Error
        error  = goal-newEp;
        error  = dot(error, error);
        reserr = [reserr error];

        % Update State
        J = jacobian(t, newAngles);;
        ep = newEp;
        angles = newAngles;
        gradient = J' * (ep-goal);

        count = count + 1;
    end
end
