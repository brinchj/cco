function [alpha] = armijo_backtrack(f, t, x, J, pk, goal)
    ro = 0.90;
    c1 = 0.00001;
    alpha = 1;
    e = [0;0;1];
    limit = 1;
    point = 2;
    while (point > limit) && (alpha > 0.01)
        point = f(t, x + alpha*pk, e);
        point = dot(goal - point, goal - point);
        limit = f(t, x, e) + c1*alpha*transpose(pk)*transpose(J)*(f(t,x,e)-goal);
        limit = dot(goal - limit, goal - limit);
        alpha = alpha * ro;
    end
end