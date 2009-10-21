close all;
clear all;
config = setup_config( 100, 300, 300, 10 );


lambda = 1;

iters = 1000;

%M = moviein(iters);
fig = figure(1);

%set(fig, 'Renderer', 'OpenGL');

counts = []

for i=1:iters
    info   = collision_detection(config);
    [config lambda count] = integrate(config, info, 0.015, lambda);
    counts = [ counts count ];

    % if mod(i, 16) == 0
    %     name = 'plots_conv/total.eps'
    %     fig = figure(2);
    %     semilogy(counts);
    %     print('-deps', name);
    % end

    if mod(i, 8) == 0
        figure(fig);
        clf;

        hold on;
        draw_config( config );
        draw_info( config, info );
        hold off;
        axis square;

        %name = strcat('plots_num/s',sprintf('%04d',i),'.png');
        %print('-dpng', name);
    end

    %M(:,i) = getframe;
end

% name = 'plots_conv/total.eps'
% fig = figure(2);
% semilogy(counts);
% print('-deps', name);



