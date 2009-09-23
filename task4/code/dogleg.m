function [ reserr angles ] = dogleg( goal, t, angles )

    % Initial configuration
    c_delta = 0.5
    delta   = 0.5

    % Compute first endpoint
    ep = f(t, angles);

    % Gradient
    J = jacobian(t, angles);
    g = J' * (ep-goal);

    % Initial Hessian and its Inverse
    B  = eye(size(J, 2));
    Bi = eye(size(J, 2));

    % Init Error Variables
    reserr = [];
    error  = dot(ep-goal,ep-goal);

    % Iteration Counter
    count = 0;
    q = 1;
    % Approximation local minimum
    while dot(g,g) > 0.01 && count < 50
        count = count + 1

        % Newton and Cauchy points
        pB = - Bi * g;
        pU = - ((g'*g) / (g' * B * g)) * g;

        g'*B*g
        ((g'*g) / (g' * B * g))
        assert((g'*g) / (g'*B*g) >= 0)

        % Compute direction vector
        p = zeros(length(angles));
        if norm(pB) <= delta
            p = pB;
        elseif norm(pU) >= delta
            p = (delta / norm(pU)) * pU;
        else
            alpha = (delta - norm(pU)) / ...
                    (norm(pB) + 2*dot(pU,pB));
            p = pU + alpha*pB;
        end

        % New Angles and Endpoint
        newAngles = angles + p;
        newEp = f(t, newAngles);

        % Compute new Error
        newError  = goal-newEp;
        newError  = dot(newError, newError);

        % Expected error
        newExpError = error + g' * p + .5 * p' * B * p;
        error = error
        newExpError = newExpError

        % Compare new error with previous
        q = (error - newError) / (error - newExpError);
        if q < 0.25
            % change region
            delta = 0.25 * norm(p);
        else
            if q > 0.75 && norm(p) == delta
                delta = min(2*delta, 2)
            end
        end
        if q < 0
            % REJECT
            continue;
        end

        % Update Approximation of Hessian
        newJ = jacobian(t, newAngles);
        y  = newJ'*(newEp-goal) - g;
        vA = (y * y') / (y' * p);
        vB = (B*p * p'*B) / (p' * B * p);
        B = B + vA - vB;
        B = abs(B);

        % Update Approximation of Inverse Hessian
        vA = ((p*p')*(p'*y + y'*Bi*y))/((p'*y)^2);
        vB = (Bi*y*p' + p*y'*Bi)/(p'*y);
        Bi = Bi + vA - vB;

        % Update Gradient
        g  = newJ' * (newEp - goal);

        % Update State
        J  = newJ;
        ep = newEp;
        angles = newAngles;
        error = newError;

        reserr = [ reserr error ];
    end
    error
    dot(g,g)
end