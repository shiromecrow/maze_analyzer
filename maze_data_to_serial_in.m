function maze_data_to_serial_in(maze_row_size,maze_col_size,maze_row_data,maze_col_data)
%% 画像から得た迷路データをbitで保存

global  g_maze_row;
global  g_maze_column;

g_maze_row = uint32(zeros(maze_col_size-2,1));
g_maze_column = uint32(zeros(maze_row_size-2,1));

g_maze_row(1) = 340+2^10+2^12+2^14;
g_maze_column(1) = 32257;
g_maze_row(2) = 475;
g_maze_column(2) = 64128;
g_maze_row(3) = 556+2;
g_maze_column(3) = 1088;
g_maze_row(4) = 4128+1+2+4+8;
g_maze_column(4) = 7200;
g_maze_row(5) = 8272+7;
g_maze_column(5) = 9744;
g_maze_row(6) = 23912;
g_maze_column(6) = 24072;
g_maze_row(7) = 44148;
g_maze_column(7) = 14356;
g_maze_row(8) = 23546;
g_maze_column(8) = 48888;
g_maze_row(9) = 16382;
g_maze_column(9) = 14900;
g_maze_row(10) = 16384;
g_maze_column(10) = 22074;
g_maze_row(11) = 8256;
g_maze_column(11) = 27152;
g_maze_row(12) = 4128;
g_maze_column(12) = 54816;
g_maze_row(13) = 2048;
g_maze_column(13) = 10816;
g_maze_row(14) = 0;
g_maze_column(14) = 5248;
g_maze_row(15) = 1600;
g_maze_column(15) = 2816;



% g_maze_row(1) = 0b0101101111010100;
% g_maze_row(2) = 0b1010110101001110;
% g_maze_row(3) = 0b0101011011011010;
% g_maze_row(4) = 0b0010101010000100;
% g_maze_row(5) = 0b0001010101000000;
% g_maze_row(6) = 0b0000101010000000;
% g_maze_row(7) = 0b0011010111000000;
% g_maze_row(8) = 0b0111111000100000;
% g_maze_row(9) = 0b0001111110000000;
% g_maze_row(10) = 0b1100100101000000;
% g_maze_row(11) = 0b0110111010000000;
% g_maze_row(12) = 0b0011110100100000;
% g_maze_row(13) = 0b0101010100100000;
% g_maze_row(14) = 0b1111101011000000;
% g_maze_row(15) = 0b0011111111100000;
% 
% g_maze_column(1) = 0b0111111111111101;
% g_maze_column(2) = 0b0110111010110010;
% g_maze_column(3) = 0b0111011011010100;
% g_maze_column(4) = 0b1111111111110000;
% g_maze_column(5) = 0b0110111111111010;
% g_maze_column(6) = 0b0001010100110100;
% g_maze_column(7) = 0b0011111110101000;
% g_maze_column(8) = 0b0001000000010000;
% g_maze_column(9) = 0b0110110011010000;
% g_maze_column(10) = 0b0000001000101000;
% g_maze_column(11) = 0b0000010001010100;
% g_maze_column(12) = 0b0000010000101010;
% g_maze_column(13) = 0b0000111001010101;
% g_maze_column(14) = 0b0000001000101010;
% g_maze_column(15) = 0b0101000111110100;

% g_maze_row(1) = 6936;
% g_maze_column(1) = 30427;
% g_maze_row(2) = 9802;
% g_maze_column(2) = 8395;
% g_maze_row(3) = 20730;
% g_maze_column(3) = 29094;
% g_maze_row(4) = 10348;
% g_maze_column(4) = 30640;
% g_maze_row(5) = 24167;
% g_maze_column(5) = 2334;
% g_maze_row(6) = 20090;
% g_maze_column(6) = 24579;
% g_maze_row(7) = 48744;
% g_maze_column(7) = 45395;
% g_maze_row(8) = 45922;
% g_maze_column(8) = 15358;
% g_maze_row(9) = 5734;
% g_maze_column(9) = 48732;
% g_maze_row(10) = 5364;
% g_maze_column(10) = 55304;
% g_maze_row(11) = 238;
% g_maze_column(11) = 3469;
% g_maze_row(12) = 6246;
% g_maze_column(12) = 49748;
% g_maze_row(13) = 7216;
% g_maze_column(13) = 28266;
% g_maze_row(14) = 3472;
% g_maze_column(14) = 16375;
% g_maze_row(15) = 26038;
% g_maze_column(15) = 30382;

end