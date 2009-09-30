function [frames] = gproject_movie(num_angles, iters)

    pointx = -0.75*num_angles/2;
    pointy = 0.75*num_angles/2;

    M = moviein(iters);

    f = figure(1);
    angles = ones(num_angles,1) * pi/4;

    for i = 1:iters
        i

        dom = pi / i;

        bound  = ones(num_angles, 1) * dom;
        bounds = [ -bound bound ];

        bounds( 5,:) = [ -pi pi ];
        bounds(10,:) = [ -pi pi ];
        bounds(15,:) = [ -pi pi ];

        t = [ zeros(1,num_angles); ones(1,num_angles) ];

        % enforce the new constraints
        angles = max(min(angles, bound), -bound);
        [res angles] = gproject([pointx; pointy; 1], t, angles, ...
                                bounds);

        plot(pointx, pointy, 'bo');
        draw_chain(t, angles);
        axis([-12 2 0 12]);

        M(:,i) = getframe;
    end

    mpgwrite(M, jet, 'movie.mpg');
end