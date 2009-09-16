function plotit(name, method, gx, gy, angles, i, N)
    t = [ 0 0 0; 1 1 1];

    [res new_angles] = method([gx; gy; 1], t, angles);

    subplot(N, 2, 2*i+1);
    plot(gx, gy, 'bo');
    legend(name);
    draw_chain(t, new_angles);
    axis([-4 4 -4 4]);
    subplot(N, 2, 2*i+2);
    plot(res);
    legend([name,' ERR']);
end