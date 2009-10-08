close all;
clear all;
config = setup_config( 100, 300, 300, 10 );


lambda = 1;

iters = 1000;

M = moviein(iters);
fig = figure(1);

%set(fig, 'Renderer', 'OpenGL');

for i=1:iters
    info   = collision_detection(config);
    [config lambda] = integrate(config, info, 0.015, lambda);

    if mod(i, 1) == 0
        figure(fig);
        clf;

        hold on;
        draw_config( config );
        draw_info( config, info );
        hold off;
        axis square;
    end

    %name = strcat('plots_num/s',sprintf('%04d',i),'.png');
    %print('-dpng', name);

    %M(:,i) = getframe;
end

%mpgwrite(M, jet, 'movie.mpg');
