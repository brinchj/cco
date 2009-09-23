function [ reserr angles ] = dogleg( goal, t, angles )

    % Initial configuration
    c_delta = 0.5
    delta   = 0.1

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
    while dot(g,g) > 0.01 && count < 2

        % Newton and Cauchy points
        pB = - Bi * g;
        pU = - (g'*g / (g' * B * g)) * g;

        % Compute direction vector
        p = zeros(length(angles));
        if dot(pB, pB) <= delta
            p = pB;
        elseif dot(pU, pU) >= delta
            p = (delta / (pU' * pU)) * pU;
            choice2 = 1
        else
            alpha = (delta - dot(pU, pU)) / ...
                    (dot(pB, pB) + 2*pU'*pB);
            p = pU + alpha*pB;
            choice2 = 2
        end

        error

        % New Angles and Endpoint
        newAngles = angles + p;
        newEp = f(t, newAngles);

        % Compute new Error
        newError  = goal-newEp;
        newError  = dot(newError, newError)

        % Expected error
        newExpError = error + g' * p + .5 * p' * B * p
        assert(error > newExpError)

        % Compare new error with previous
        q = (error - newError) / (error - newExpError)
        if q < 0.25
            % change region
            delta = 0.25 * dot(p, p);
            choice = 0
        else
            if q > 0.75 && dot(p, p) == delta
                choice = 1
                delta = min(2*delta, 1)
            end
        end
        if q < 0
            % REJECT
            count = count + 1;
            continue;
        end

        % Update Gradient
        newJ = jacobian(t, newAngles);

        % Update Approximation of Hessian
        y  = newJ'*(newEp-goal) - g;
        vA = (y * y') / (y' * p);
        vB = (B * p * p' * B) / (p' * B * p);
        B = B + vA - vB;

        % Update Approximation of Inverse Hessian
        vA = (p*p')*(p'*y + y'*Bi*y)/((p'*y)^2);
        vB = (Bi*y*p' + p*y'*Bi)/(p'*y);
        Bi = Bi + vA - vB;

        % Update State
        J  = newJ;
        ep = newEp;
        g  = J' * (ep-goal);
        angles = newAngles;
        error = newError;

        reserr = [ reserr error ];

        % Update Iteration Counter
        count = count + 1;
    end
end
