function [angles] = gproject_backtrack(f, t, x, J, pk, goal, bounds)

    ro = 0.90;
    alpha = 2;
    e = [0;0;1];

    limit = f(t, x, e);
    limit = dot(goal - limit, goal - limit);
    point = limit + 1;

    % normalize angles
    for i = 1:length(x)
        while x(i) > pi
            x(i) = x(i) - 2*pi;
        end
        while x(i) < -pi
            x(i) = x(i) + 2*pi;
        end
    end

    while (point > limit)
        point = f(t, max(min(x + alpha*pk, ...
                             bounds(:,2)), bounds(:, 1)), e);
        point = dot(goal - point, goal - point);

        alpha = alpha * ro;
    end

    angles = max(min(x + alpha*pk, bounds(:,2)), bounds(:,1));
end