%mex run_maze_solve_front.c
%mex run_maze_solve_back.c
mex run_pass_maker.c
mex run_pass_maker2.c

clearvars
clearvars -global
close all
clc
%% パス定義
currentdir = cd;
%addpath(strcat(currentdir,'/src'),'-end')

%% デバッグモード定義
global maze_data_get_debug;
maze_data_get_debug = 0;
global video_flg;
global vidObj; %動画作成用のビデオオブジェクト
%ビデオフラグ
video_flg = 0;
%画像なら0,数字直打ち1
get_maze_mode=1;
global WalkCount_display_flg;
global Network_display_flg;
WalkCount_display_flg=1;
Network_display_flg=0;
%% ビデオ記録用変数の宣言
if video_flg
    vidObj = VideoWriter('result.mp4','MPEG-4');
    vidObj.FrameRate = 1/0.1;
    open(vidObj);
end
%% シミュレーションモード選択
sim_mode.unknown = uint8(0); %壁情報がない状態から、探索、最短の実行
sim_mode.known = uint8(1); %壁情報を既知として、最短の実行

%% mode定義
%走行モード(run_mode_1)
run_mode1.search = uint8(0);
run_mode1.fust_run = uint8(1);
%探索モード(run_mode_2)
run_mode2.adachi = uint8(0);
run_mode2.all = uint8(1);
run_mode2.short = uint8(2);
run_mode2.straight = uint8(0);
run_mode2.diagonal = uint8(1);

%% シミュレーションモードを設定（手入力）
sim_mode_flg = sim_mode.unknown;
run_mode1_flg =  run_mode1.search;
run_mode2_flg = run_mode2.adachi;
%% 迷路パラメータ設定(手入力)
% global maze_goal


%指定する迷路に合わせてゴール座標、サイズを変更
%大会名　　　x  y size
%2019全日本　18 14 9
%2019中部　　7 10 4
%2018関東　 10 10 4
%2018全セミ 13 13 4
%2017全日本 20 21 9
%2020全日本学生 4 4 4
%no_test1 7 7 4
%no_test2 3 3 4
%no_test3 10 9 4
%no_test4 6 9 4
%no_test5 7 7 4
%2019全日本　17 13 9
%2022全日本　14 14 9

goal_x = 6;%610;%ゴール左下のx座標
goal_y = 9;%911;%ゴール左下のy座標
goal_size = uint8(4);%ゴールサイズを入力する

goal_size_d = double(goal_size);

tic

%% 迷路データ取得
global  maze_serial;

[maze_row_size,maze_col_size,maze_row_data,maze_col_data] = maze_data_get;
maze_row_size = uint8(maze_row_size);
maze_col_size = uint8(maze_col_size);

global  g_maze_row;
global  g_maze_column;
global  g_walk_count;

if get_maze_mode==0
maze_data_to_serial2(maze_row_size,maze_col_size,maze_row_data,maze_col_data);
else
maze_data_to_serial_in(maze_row_size,maze_col_size);
end

%g_walk_count=cal_walkcount_queue(g_maze_row,g_maze_column,uint32(goal_x),uint32(goal_y));
%% 迷路をプロット

%プロットするfigure,axisを定義
global maze_fig;
global maze_fig_ax;


maze_fig_ax = gca;
set(maze_fig_ax,'color','none','NextPlot','add')
maze_fig = gcf;
set(gcf,'doublebuffer','off');

%figureの出力位置
maze_fig.Position = [2,42,958,954];

%%
%maze_data_plot2(maze_row_size,maze_col_size);
%maze_data_plot(maze_row_size,maze_col_size,maze_row_data,maze_col_data);
figure(maze_fig);

wall_row = uint32(zeros(maze_col_size-2,1));
wall_column = uint32(zeros(maze_row_size-2,1));
wall_row_look = uint32(zeros(maze_col_size-2,1));
wall_column_look = uint32(zeros(maze_row_size-2,1));
wall_row(1) = 0;
wall_column(1) = 1;
wall_row_look(1) = 1;
wall_column_look(1) = 1;
coordinate=[int32(0);int32(0);int32(1)];
Sensor_front = uint16(0);
Sensor_right = uint16(1);
Sensor_left = uint16(1);
Dijkstra_maker = uint8(0);
Error = uint8(0);
all_mode=[Dijkstra_maker;Error];
D_row_map=uint16(zeros((maze_row_size-1),(maze_row_size-2)));
D_column_map=uint16(zeros((maze_row_size-1),(maze_row_size-2)));
D_Network=uint16(zeros(2*uint16(maze_row_size-1)*uint16(maze_row_size-2),2*uint16(maze_row_size-1)*uint16(maze_row_size-2)));
maze_data_plot2_solve_0(wall_row,maze_row_size,wall_column,maze_col_size,coordinate(1),coordinate(2),coordinate(3));

Straight_all=[];
Turning_all=[];
T_all=0;

while (1)%Dijkstra_maker,Error

    [Sensor_front,Sensor_right,Sensor_left]=maze_get_wall(coordinate(1),coordinate(2),coordinate(3),g_maze_row,g_maze_column,maze_row_size,maze_col_size);

    wall_row_0=wall_row;
    wall_column_0 =wall_column;
[wall_row,wall_column,wall_row_look,wall_column_look,coordinate,g_walk_count,D_row_map,D_column_map,all_mode,Straight_log,Turning_log]=run_maze_solve_front( ...
    wall_row,wall_column,wall_row_look,wall_column_look, ...
    uint32(goal_x),uint32(goal_y),coordinate(1),coordinate(2),coordinate(3),Sensor_front,Sensor_right,Sensor_left, ...
    all_mode(1),all_mode(2),D_row_map,D_column_map);
D_row_map=D_row_map';
D_column_map=D_column_map';
del_wall_row=wall_row - wall_row_0;
del_wall_column=wall_column - wall_column_0 ;

%Straight_all=[Straight_all Straight_log(1:end-1)];
%Turning_all=[Turning_all Turning_log(1:end-1)];
T_all=T_all+Straight_log(end);
maze_data_plot2_solve(del_wall_row,maze_row_size,del_wall_column,maze_col_size,coordinate(1),coordinate(2),coordinate(3));

%coordinate
if (coordinate(1)==goal_x || coordinate(1)==goal_x+1) && (coordinate(2)==goal_y || coordinate(2)==goal_y+1)
break
end

end

T_all
while (1)%Dijkstra_maker,Error

    [Sensor_front,Sensor_right,Sensor_left]=maze_get_wall(coordinate(1),coordinate(2),coordinate(3),g_maze_row,g_maze_column,maze_row_size,maze_col_size);

    wall_row_0=wall_row;
    wall_column_0 =wall_column;
[wall_row,wall_column,wall_row_look,wall_column_look,coordinate,g_walk_count,D_row_map,D_column_map,all_mode,Straight_log,Turning_log]=run_maze_solve_back( ...
    wall_row,wall_column,wall_row_look,wall_column_look, ...
    uint32(goal_x),uint32(goal_y),coordinate(1),coordinate(2),coordinate(3),Sensor_front,Sensor_right,Sensor_left, ...
    all_mode(1),all_mode(2),D_row_map,D_column_map);
D_row_map=D_row_map';
D_column_map=D_column_map';
del_wall_row=wall_row - wall_row_0;
del_wall_column=wall_column - wall_column_0 ;

%Straight_all=[Straight_all Straight_log(1:end-1)];
%Turning_all=[Turning_all Turning_log(1:end-1)];
T_all=T_all+Straight_log(end);
maze_data_plot2_solve(del_wall_row,maze_row_size,del_wall_column,maze_col_size,coordinate(1),coordinate(2),coordinate(3));

%coordinate
if (coordinate(1)==0) && (coordinate(2)==0)
break
end
end
T_all


% [Pass,Xp,Yp,D_row_map,D_column_map]=run_pass_maker( ...
%     wall_row,wall_column,wall_row_look,wall_column_look, ...
%     uint32(goal_x),uint32(goal_y),coordinate(1),coordinate(2),coordinate(3),Sensor_front,Sensor_right,Sensor_left, ...
%     all_mode(1),all_mode(2),D_row_map,D_column_map);

[Pass,Xp,Yp,D_row_map,D_column_map,D_Network,Xp2,Yp2,RCp,Disp]=run_pass_maker2( ...
    wall_row,wall_column,wall_row_look,wall_column_look, ...
    uint32(goal_x),uint32(goal_y),coordinate(1),coordinate(2),coordinate(3),Sensor_front,Sensor_right,Sensor_left, ...
    all_mode(1),all_mode(2),D_row_map,D_column_map);

D_row_map=D_row_map';
D_column_map=D_column_map';
maze_data_plot2_Dresult(wall_row,maze_row_size,wall_column,maze_col_size,D_row_map,D_column_map,D_Network);

%route_plot(Xp,Yp);
route_plot_network(Xp2,Yp2,RCp);
%route_maker(Pass);

%% ビデオ作成の完了
if video_flg
    close(vidObj);
end

toc
