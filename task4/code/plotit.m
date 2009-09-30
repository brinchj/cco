function plotit(name, method, t, gx, gy, angles, i, N)
    time_start = cputime();
    [res new_angles] = method([gx; gy; 1], t, angles);
    time_used  = cputime() - time_start;

    subplot(N, 2, 2*i+1);
    plot(gx, gy, 'bo');
    legend(['Goal: ( ', num2str(gx), ', ', num2str(gy), ' )']);
    draw_chain(t, new_angles);

    NA = length(angles);
    axis([-NA NA -NA NA]);
    title([ name, ' ( ', num2str(time_used), 's )' ]);

    subplot(N, 2, 2*i+2);
    semilogy(res);
    legend('Error');
    title([ name, ' ( ', num2str(time_used), 's )' ]);
end