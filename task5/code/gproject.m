function [ reserr angles ] = gproject( goal, t, angles )
    reserr = [];

    % gradient
    J  = jacobian(t, angles);
    % compute first endpoint
    ep = f(t, angles);

    count = 0;
    gradient = J' * (ep-goal);
    while dot(gradient,gradient) > 0.01 && count < 500
        % Gradient Descent
        p = - gradient;

        % Armijo Backtrack
        newAngles = gproject_backtrack(@f, t, angles, J, p, goal);

        % Update EndPoint
        newEp = f(t, newAngles);

        % Compute Error
        error  = goal-newEp;
        error  = dot(error, error);
        reserr = [reserr error];

        % Update Gradient and Angles
        newJ = jacobian(t, newAngles);

        % Update State
        J = newJ;
        ep = newEp;
        angles = newAngles;
        gradient = J' * (ep-goal);

        count = count + 1;
    end
end
