function h = draw_line(q1,q2)
    x1i = 0.50*cos(q1);
    y1i = 0.50*sin(q1);
    x2i = x1i + 0.40*cos(q1 + q2);
    y2i = y1i + 0.40*sin(q1 + q2);
    hold on
    draw_circle(-0.6,0.7,0.2); draw_circle(0.6,0.7,0.2);
    hold on
    plot([-1 1], [-0.1 -0.1], 'r');
    hold on
    plot(0,0, '*b', 'LineWidth', 2);
    hold on
    plot([0 x1i], [0 y1i], 'k');
    hold on
    plot([x1i x2i], [y1i y2i], 'g');
    axis([-1 1 -1 1]);

end