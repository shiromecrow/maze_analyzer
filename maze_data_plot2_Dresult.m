function maze_data_plot2_Dresult(wall_row,maze_row_size,wall_column,maze_col_size,D_row_map,D_column_map)

global maze_fig;
global maze_fig_ax;
global fig_walkcount;
global fig_walkcount_D_row;
global fig_walkcount_D_column;
global fig_mouse;

global  g_walk_count;

maze_step =  9;% [mm]
maze_row_size = double(maze_row_size);
maze_col_size = double(maze_col_size);

[poleX,poleY] = meshgrid(0:maze_step:(maze_col_size-1)*maze_step,0:maze_step:(maze_row_size-1)*maze_step);
plot(poleX,poleY,'s','MarkerSize',5,...
    'MarkerEdgeColor',[0 0 0],...
    'MarkerFaceColor',[0.4 0 0]);
xlim([0 (maze_col_size-1)*maze_step])
ylim([0 (maze_row_size-1)*maze_step])
xticks(0:maze_step:(maze_col_size-1)*maze_step)
yticks(0:maze_step:(maze_row_size-1)*maze_step)
xticklabels({})
yticklabels({})
pbaspect([1 1 1])
grid on

%  ax = gca;
%  ax.TickLength = [0.0 0.0];
maze_fig_ax.TickLength = [0.0 0.0];

figure(maze_fig);
% hold on

delete(fig_walkcount);

%行方向壁情報プロット
for i = 2:maze_row_size-1
    for j = 1:1:(maze_col_size-1)
       if (bitand(wall_row(i-1),bitshift(1,j-1)) ~= bitshift(1,j-1))
           %plot([j-1,j].*maze_step,[i-1,i-1].*maze_step,'Color',[0 0 0.9],'LineWidth',1.5)
           fig_walkcount_D_row(j,i-1)=text((j-0.5)*9,(i-1)*9,num2str(D_row_map(j,i-1)),'VerticalAlignment','middle','HorizontalAlignment','center','FontSize',11,'EdgeColor','none');
       end
    end
end


%列方向壁情報プロット
for i = 1:maze_row_size-1
    for j = 2:maze_col_size-1
       if (bitand(wall_column(j-1),bitshift(1,i-1)) ~= bitshift(1,i-1))
           %plot([j-1,j-1].*maze_step,[i-1,i].*maze_step,'Color',[0 0 0.9],'LineWidth',1.5)
           fig_walkcount_D_column(j-1,i)=text((j-1)*9,(i-0.5)*9,num2str(D_column_map(i,j-1)),'VerticalAlignment','middle','HorizontalAlignment','center','FontSize',11,'EdgeColor','none');
       end
    end
end



%delete(fig_mouse);
%fig_mouse=text(4.5+(x)*9,4.5+(y)*9,'M','VerticalAlignment','middle','HorizontalAlignment','center','FontSize',20,'EdgeColor','none');

% hold off


end
