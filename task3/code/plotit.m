function plotit(name, method, t, gx, gy, angles, i, N)
    [res new_angles] = method([gx; gy; 1], t, angles);

    subplot(N, 2, 2*i+1);
    plot(gx, gy, 'bo');
    legend(['Goal: ( ', num2str(gx), ', ', num2str(gy), ' )']);
    draw_chain(t, new_angles);

    NA = length(angles);
    axis([-NA NA -NA NA]);
    title(name);

    subplot(N, 2, 2*i+2);
    semilogy(res);
    legend('Error');
    title(name);
end