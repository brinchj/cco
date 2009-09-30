function [angles] = gproject_backtrack(f, t, x, J, pk, goal)
    lowerBound = -pi/4;
    upperBound =  pi/4;

    ro = 0.90;
    alpha = 1;
    e = [0;0;1];

    limit = f(t, x, e);
    limit = dot(goal - limit, goal - limit);
    point = limit + 1;
    
    

    while (point >= limit) && (alpha > 0.0001)
        point = f(t, max(min(x + alpha*pk, upperBound), lowerBound), e);
        point = dot(goal - point, goal - point);
        %limit = f(t, x, e) + c1*alpha*transpose(pk)*transpose(J)*(f(t,x,e)-goal);
        %limit = dot(goal - limit, goal - limit);
        
        alpha = alpha * ro;
    end
    
    angles = max(min(x + alpha*pk, upperBound), lowerBound);
end