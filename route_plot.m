function route_plot(Xp,Yp)

global maze_fig;
global maze_fig_ax;
global fig_walkcount;
global fig_mouse;

global  g_walk_count;

maze_step =  9;% [mm]


%  ax = gca;
%  ax.TickLength = [0.0 0.0];
maze_fig_ax.TickLength = [0.0 0.0];

figure(maze_fig);
% hold on

for i=2:501
    plot([4.5+(Xp(i-1))*9 4.5+(Xp(i))*9],[4.5+(Yp(i-1))*9 4.5+(Yp(i))*9],'Color',[0 0.4 0],'LineWidth',2)
    if Xp(i+1)==0 && Yp(i+1)==0
        break
    end
    %text(4.5+(Xp(i-1))*9,4.5+(y)*9,'M','VerticalAlignment','middle','HorizontalAlignment','center','FontSize',20,'EdgeColor','none');
end



% hold off


end