function maze_data_to_serial_in(maze_row_size,maze_col_size,maze_row_data,maze_col_data)
%% 画像から得た迷路データをbitで保存

global  g_maze_row;
global  g_maze_column;

g_maze_row = uint32(zeros(maze_col_size-2,1));
g_maze_column = uint32(zeros(maze_row_size-2,1));


g_maze_row(1) = 6936;
g_maze_column(1) = 30427;
g_maze_row(2) = 9802;
g_maze_column(2) = 8395;
g_maze_row(3) = 20730;
g_maze_column(3) = 29094;
g_maze_row(4) = 10348;
g_maze_column(4) = 30640;
g_maze_row(5) = 24167;
g_maze_column(5) = 2334;
g_maze_row(6) = 20090;
g_maze_column(6) = 24579;
g_maze_row(7) = 48744;
g_maze_column(7) = 45395;
g_maze_row(8) = 45922;
g_maze_column(8) = 15358;
g_maze_row(9) = 5734;
g_maze_column(9) = 48732;
g_maze_row(10) = 5364;
g_maze_column(10) = 55304;
g_maze_row(11) = 238;
g_maze_column(11) = 3469;
g_maze_row(12) = 6246;
g_maze_column(12) = 49748;
g_maze_row(13) = 7216;
g_maze_column(13) = 28266;
g_maze_row(14) = 3472;
g_maze_column(14) = 16375;
g_maze_row(15) = 26038;
g_maze_column(15) = 30382;

end