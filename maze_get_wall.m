function [front_wall,right_wall,left_wall]=maze_get_wall(x,y,direction,wall_row,wall_column)
%% 壁の取得

	front_wall=1;
	right_wall=1;
	left_wall=1;

switch direction
    case 1
        y=y+1;
        if y <= 14
			front_wall=(bitand(wall_row(y+1),bitshift(1,x))==bitshift(1,x));
        end
		if x >= 1 
            left_wall=(bitand(wall_column(x),bitshift(1,y))==bitshift(1,y));
        end
		if x <= 14 
            right_wall=(bitand(wall_column(x+1),bitshift(1,y))==bitshift(1,y));
        end
    case 2
        x=x+1;
        if x <= 14 
			front_wall=(bitand(wall_column(x+1),bitshift(1,y))==bitshift(1,y));
        end
		if y <= 14 
			left_wall=(bitand(wall_row(y+1),bitshift(1,x))==bitshift(1,x));
        end
		if y >= 1 
			right_wall=(bitand(wall_row(y),bitshift(1,x))==bitshift(1,x));
        end
    case 3
         y=y-1;
        if y >= 1
			front_wall=(bitand(wall_row(y),bitshift(1,x))==bitshift(1,x));
        end
		if x <= 14 
            left_wall=(bitand(wall_column(x+1),bitshift(1,y))==bitshift(1,y));
        end
		if x >= 1 
            right_wall=(bitand(wall_column(x),bitshift(1,y))==bitshift(1,y));
        end
    case 4
         x=x-1;
        if x >= 1 
			front_wall=(bitand(wall_column(x),bitshift(1,y))==bitshift(1,y));
        end
		if y >= 1 
			left_wall=(bitand(wall_row(y),bitshift(1,x))==bitshift(1,x));
        end
		if y <= 14 
			right_wall=(bitand(wall_row(y+1),bitshift(1,x))==bitshift(1,x));
        end
end    

	front_wall=uint16(front_wall);
	right_wall=uint16(right_wall);
	left_wall=uint16(left_wall);


end