function animate_crane_sim(sim)
    fig = figure;
    fig.Position = [1030 400 560 420];
    ax = axes;
    
    xlim([-5 5]);
    ylim([-11 1]);
    xlabel('x in m');
    ylabel('y in m');
    hold on;
    
    L = 10;
    for i=1:length(sim.T)
        cla(ax);
        if length(fieldnames(sim.agents)) > 1
            draw_crane(ax, sim.agents.leader, L, i);
            draw_crane(ax, sim.agents.follower, L, i);
        else
            draw_crane(ax, sim.agents.crane, L, i);
        end
        
        drawnow;
        %pause(dt);
    end
    disp("done");
end

function draw_crane(ax, crane, L, i)
    
    x = crane.history.x(:, i);
    cat_pos = [x(1) 0];
    container_pos = L * [sin(x(3)) -cos(x(3))] + cat_pos;
    hold on
    rectangle(ax, 'Position', [cat_pos(1)-0.25 cat_pos(2) 0.5 0.25]);
    line(ax, [container_pos(1) cat_pos(1)], [container_pos(2) cat_pos(2)]);
    line(ax, [-5 5], [0 0], 'Color', [0 0 0])

    w = 0.6;
    h = 0.3;

    poly_x = [0 0 w w];
    poly_y = [0 h h 0];
    container_poly = translate( rotate( polyshape(poly_x, poly_y), x(3)*180/pi ), container_pos - [w h]/2);
    %plot(container_poly)

    rectangle(ax, 'Position', [container_pos(1)-0.3 container_pos(2)-0.15 0.6 0.3], 'FaceColor', [1 1 1]);

end