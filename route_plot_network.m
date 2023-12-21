function route_plot_network(Xp,Yp,RCp)

global maze_fig;
global maze_fig_ax;
global fig_walkcount;
global fig_mouse;

global  g_walk_count;

maze_step =  9;% [mm]

Xp=double(Xp);
Yp=double(Yp);

%  ax = gca;
%  ax.TickLength = [0.0 0.0];
maze_fig_ax.TickLength = [0.0 0.0];

figure(maze_fig);
% hold on


for i=2:501
    if RCp(i-1)==0
        x0=(Xp(i-1)+0.5)*9;
        y0=(Yp(i-1)+1)*9;
    else
        x0=(Xp(i-1)+1)*9
        y0=(Yp(i-1)+0.5)*9
    end

    if RCp(i)==0
        x1=(Xp(i)+0.5)*9;
        y1=(Yp(i)+1)*9;
    else
        x1=(Xp(i)+1)*9
        y1=(Yp(i)+0.5)*9
    end


    plot([x0 x1],[y0 y1],'Color',[0 0.4 0],'LineWidth',2,'Marker','*',MarkerSize=20)

    if Xp(i+1)==0 && Yp(i+1)==0
        break
    end
    %text(4.5+(Xp(i-1))*9,4.5+(y)*9,'M','VerticalAlignment','middle','HorizontalAlignment','center','FontSize',20,'EdgeColor','none');
end



% hold off


end