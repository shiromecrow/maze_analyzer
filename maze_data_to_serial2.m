function maze_data_to_serial2(maze_row_size,maze_col_size,maze_row_data,maze_col_data)
%% 画像から得た迷路データをbitで保存

global  g_maze_row;
global  g_maze_column;

g_maze_row = uint32(zeros(maze_col_size-2,1));
g_maze_column = uint32(zeros(maze_row_size-2,1));

trans_Bit_row=zeros(1,maze_row_size-1);
trans_Bit_column=zeros(1,maze_col_size-1);

for i=1:maze_row_size-1
    trans_Bit_row(i) = bitshift(uint32(1),i-1);
end
for i = 2:maze_col_size-1
    g_maze_row(i-1)=trans_Bit_row*maze_row_data(i,:)';
end

for i=1:maze_col_size-1
    trans_Bit_column(i) = bitshift(uint32(1),i-1);
end
for i = 2:maze_row_size-1
    g_maze_column(i-1)=trans_Bit_column*maze_col_data(:,i);
end


end