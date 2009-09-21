function [ reserr angles ] = bfgs( goal, t, angles )
    reserr = [];

    % gradient
    J  = jacobian(t, angles);
    % compute first endpoint
    ep = f(t, angles);
    % approximation of inverse hessian
    Bi = eye(size(J, 2));

    count = 0;
    gradient = J' * (ep-goal);
    while dot(gradient,gradient) > 0.01 && count < 50
        % BFGS Search Direction
        p = - Bi * gradient;
        % Armijo Backtrack
        alpha  = armijo_backtrack(@f, t, angles, J, p, goal);

        % Update EndPoint
        newAngles = angles + alpha*p;
        newEp = f(t, newAngles);

        % Compute Error
        error  = goal-newEp;
        error  = dot(error, error);
        reserr = [reserr error];

        % Update Gradient and Angles
        newJ = jacobian(t, newAngles);

        % Update Approximation of Inverse Hessian
        y  = newJ'*(newEp-goal) - gradient;
        vA = (p*p')*(p'*y + y'*Bi*y)/((p'*y)^2);
        vB = (Bi*y*p' + p*y'*Bi)/(p'*y);
        Bi = Bi + vA - vB;

        % Update State
        J = newJ;
        ep = newEp;
        angles = newAngles;
        gradient = J' * (ep-goal);

        count = count + 1;
    end
end
