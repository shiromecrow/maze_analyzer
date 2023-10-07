#include <string.h> /* needed for memcpy() */
#include <stdint.h> /* needed for memcpy() */
#include "mex.h"

/*
 * run_maze_solve.c - example found in API guide
 *
 */


#define NDIMS 2
#define TOTAL_ELEMENTS 256

#define MAX_QUEUE_NUM 1000
#define ROW 0
#define COLUMN 1
#define MAX_WALKCOUNT 255
#define MAX_WALKCOUNT_DIJKSTRA 65535
#define ON 1
#define OFF 0
#define MAZE_SECTION 90
#define MAZE_OFFSET 8
#define BACK_TO_CENTER 20.5
#define BACK_TO_CENTER_FRONT 12.5
#define BACK_TO_CENTER_SLANT 42.5
#define BACK_TO_CENTER_FRONT_SLANT 34.5
#define EXPLORATION 0
#define SHORTEST 1



#define VERTICALCOST 180
#define DIAGONALCOST 127
#define MIN_VERTICALCOST 12
#define MIN_DIAGONALCOST 10
#define DISCOUNTCOST_V 1//絶対1
#define DISCOUNTCOST_D 1//絶対1
#define V_NUM_MAX 5
#define D_NUM_MAX 6

#define SLANT_NORTH 0
#define SLANT_NORTH_EAST 1
#define SLANT_EAST 2
#define SLANT_SOUTH_EAST 3
#define SLANT_SOUTH 4
#define SLANT_SOUTH_WEST 5
#define SLANT_WEST 6
#define SLANT_NORTH_WEST 7


// スタック構造体
typedef struct{
	/* データの最前列 */
	int head;
    /* データの最後尾 */
    int tail;
    /* スタックされているデータ */
    int data[MAX_QUEUE_NUM];
} STACK_T;

typedef struct{
	uint32_t row[15];
	uint32_t column[15];
	uint32_t row_look[15];
	uint32_t column_look[15];

}WALL;

typedef struct{
	uint16_t row_count[16][15];
	uint16_t column_count[16][15];
}DIJKSTRA;

typedef struct {
	float g_speed;
		float f_ofset;
		float e_ofset;
		float t_speed;
		float t_acc;
} parameter;


typedef struct {

float SlalomCentervelocity;
float TurnCentervelocity;

parameter slalom_R;
parameter slalom_L;
parameter turn90_R;
parameter turn90_L;
parameter turn180_R;
parameter turn180_L;
parameter turn45in_R;
parameter turn45in_L;
parameter turn135in_R;
parameter turn135in_L;
parameter turn45out_R;
parameter turn45out_L;
parameter turn135out_R;
parameter turn135out_L;
parameter V90_R;
parameter V90_L;

}parameter_speed;

typedef struct {
	uint8_t WallControlMode;//0で壁制御なし、1で通常の壁制御、2で斜めの制御
	uint8_t WallControlStatus;
	uint8_t calMazeMode;
	uint8_t WallCutMode;
	//uint8_T BreakMode;

}MOTOR_MODE;

//合わせるように定義
char maze_mode;
WALL wall;
uint32_t GOAL_X,GOAL_Y;
int kitikukan;
int g_timCount_sec=10;
int noGoalPillarMode;
uint8_t g_WallControl_mode;

//入出力変数
STACK_T g_Goal_x;
STACK_T g_Goal_y;
char Dijkstra_maker_flag;
uint32_t walk_count[16][16]; //歩数いれる箱
char error_mode;
DIJKSTRA Dijkstra;
uint16_t g_sensor_front,g_sensor_right,g_sensor_left;

void initStack_walk(STACK_T *stack){
//	for(int i=0;i<=MAX_QUEUE_NUM-1;i++){
//		stack->data[i] = 0;
//	}
    /* スタックを空に設定 */
	stack->head = 0;
    stack->tail = 0;
}

//  fake関数

void pl_yellow_LED_count(unsigned char yy){}
void pl_DriveMotor_standby(int pin){}
void pl_DriveMotor_start(void){}
void pl_DriveMotor_stop(void){}
void record_in(void) {}
void record_out(void) {}
void wait_ms_NoReset(uint32_t waitTime) {}
void maze_display(void) {}
void maze_display_Dijkstra(void) {}
void clear_Ierror(void) {}
void reset_speed(void) {}
void reset_gyro(void) {}
void reset_distance(void) {}

void straight_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration,MOTOR_MODE motor_mode) {

}
void End_straight(float input_displacement,MOTOR_MODE motor_mode,_Bool right_wall,_Bool left_wall){
	
}

float turning_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

}


void backTurn_controlWall(float input_TurningVelocity,float input_TurningAcceleration,_Bool front_wall,_Bool left_wall,_Bool right_wall){

}


void slalomR(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {

}

void slalomL(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {

}


void get_wallData_sensor(_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){

	*front_wall = (g_sensor_front > 0);
	*right_wall = (g_sensor_right > 0);
	*left_wall  = (g_sensor_left > 0);

}




void pushStack_walk(STACK_T *stack, unsigned short input){

    /* データをデータの最後尾の１つ後ろに格納 */
    stack->data[stack->tail] = input;

    /* データの最後尾を１つ後ろに移動 */
    stack->tail = stack->tail + 1;

    /* 巡回シフト */
    if(stack->tail == MAX_QUEUE_NUM) stack->tail = 0;

    /* スタックが満杯なら何もせず関数終了 */
    if(stack->tail == stack->head ){
    	printf("stack_full\n");
        return;
    }
}


unsigned short popStack_walk(STACK_T *stack){
    unsigned short ret = 0;

    /* スタックが空なら何もせずに関数終了 */
    if(stack->tail == stack->head){
    	//printf("stack_empty\n");
        return MAX_WALKCOUNT_DIJKSTRA;
    }

    /* データの最前列からデータを取得 */
    ret = stack->data[stack->head];

    /* データの最前列を１つ前にずらす */
    stack->head = stack->head + 1;

    /* 巡回シフト */
    if(stack->head == MAX_QUEUE_NUM) stack->head = 0;

    /* 取得したデータを返却 */
    return ret;
}


void update_wall(int x,int y,int direction,_Bool front_wall,_Bool right_wall,_Bool left_wall){
// x:x座標, y:y座標, direction:向き(北1東2南3西4),
//front_wall:前壁の有無(Ture=1 false=0), right_wall:右壁の有無(Ture=1 false=0), left_wall:左壁の有無(Ture=1 false=0)

	switch (direction) {
	case 1:
		if (y <= 14) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(front_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(left_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		if (x <= 14) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(right_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		break;
	case 2:
		if (x <= 14) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(front_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		if (y <= 14) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(left_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(right_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		break;
	case 3:
		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(front_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		if (x <= 14) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(left_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(right_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		break;
	case 4:
		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(front_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(left_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		if (y <= 14) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(right_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		break;

	}


}



void get_wall(int x,int y,int direction,_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){
	*front_wall=1;
	*right_wall=1;
	*left_wall=1;
	switch (direction) {
	case 1:
		if (y <= 14) {
			*front_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= 14) {
			*right_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= 14) {
			*front_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		if (y <= 14) {
			*left_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		if (y >= 1) {
			*right_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		break;
	case 3:
		if (y >= 1) {
			*front_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		if (x <= 14) {
			*left_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		if (x >= 1) {
			*right_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		break;
	case 4:
		if (x >= 1) {
			*front_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		if (y >= 1) {
			*left_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		if (y <= 14) {
			*right_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		break;
	}

}


void get_wall_look(int x,int y,int direction,_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){
	*front_wall=1;
	*right_wall=1;
	*left_wall=1;
	switch (direction) {
	case 1:
		if (y <= 14) {
			*front_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= 14) {
			*right_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= 14) {
			*front_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		if (y <= 14) {
			*left_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		if (y >= 1) {
			*right_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		break;
	case 3:
		if (y >= 1) {
			*front_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		if (x <= 14) {
			*left_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		if (x >= 1) {
			*right_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		break;
	case 4:
		if (x >= 1) {
			*front_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		if (y >= 1) {
			*left_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		if (y <= 14) {
			*right_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		break;
	}

}








void search_AroundWalkCount(unsigned short *front_count,unsigned short *right_count,unsigned short *back_count,unsigned short *left_count,int x,int y,int direction){
//int direction,int x_coordinate,int y_coordinate
	unsigned short north_count,east_count,south_count,west_count;
//	unsigned short front_count, right_count, back_count, left_count;

	if (y >= 15) {north_count = MAX_WALKCOUNT;}
	else {north_count = walk_count[x][y + 1];}

	if (x >= 15) {east_count = MAX_WALKCOUNT;}
	else {east_count = walk_count[x + 1][y];}

	if (y <= 0) {south_count = MAX_WALKCOUNT;}
	else {south_count = walk_count[x][y - 1];}

	if (x <= 0) {west_count = MAX_WALKCOUNT;}
	else {west_count = walk_count[x - 1][y];}


	switch (direction) {		//
	case 1:
		*front_count = north_count;
		*right_count = east_count;
		*back_count = south_count;
		*left_count = west_count;
		break;
	case 2:
		*front_count = east_count;
		*right_count = south_count;
		*back_count = west_count;
		*left_count = north_count;
		break;
	case 3:
		*front_count = south_count;
		*right_count = west_count;
		*back_count = north_count;
		*left_count = east_count;
		break;
	case 4:
		*front_count = west_count;
		*right_count = north_count;
		*back_count = east_count;
		*left_count = south_count;
		break;

	}


}


void search_AroundDijkstraCount(unsigned short *front_count,unsigned short *right_count,unsigned short *back_count,unsigned short *left_count,int x,int y,int direction){
//int direction,int x_coordinate,int y_coordinate
	unsigned short north_count,east_count,south_count,west_count;
//	unsigned short front_count, right_count, back_count, left_count;

	if (y >= 15) {north_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {north_count = Dijkstra.row_count[x][y];}

	if (x >= 15) {east_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {east_count = Dijkstra.column_count[y][x];}

	if (y <= 0) {south_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {south_count = Dijkstra.row_count[x][y-1];}

	if (x <= 0) {west_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {west_count = Dijkstra.column_count[y][x-1];}


	switch (direction) {		//
	case 1:
		*front_count = north_count;
		*right_count = east_count;
		*back_count = south_count;
		*left_count = west_count;
		break;
	case 2:
		*front_count = east_count;
		*right_count = south_count;
		*back_count = west_count;
		*left_count = north_count;
		break;
	case 3:
		*front_count = south_count;
		*right_count = west_count;
		*back_count = north_count;
		*left_count = east_count;
		break;
	case 4:
		*front_count = west_count;
		*right_count = north_count;
		*back_count = east_count;
		*left_count = south_count;
		break;

	}


}


void update_coordinate(int *x,int *y,int direction){
// int direction,int *x_coordinate,int *y_coordinate
//	*direction = *direction % 4;
//	if (*direction <= 0) {
//		*direction = *direction+4;
//	}
	switch (direction) {
	case 1://北
		*y += 1;
		break;
	case 2://東
		*x += 1;
		break;
	case 3://南
		*y -= 1;
		break;
	case 4://西
		*x -= 1;
		break;
	}



}





void decision_kitiku(int x,int y,int direction,unsigned short front_count,unsigned short right_count,unsigned short back_count,unsigned short left_count){
	_Bool front_wall=1;
	_Bool right_wall=1;
	_Bool left_wall=1;
	int x_front=x;
	int y_front=y;
	update_coordinate(&x_front,&y_front,direction);
	get_wall_look(x_front,y_front,direction,&front_wall,&right_wall,&left_wall);
	_Bool look_f=(front_wall && right_wall && left_wall);

	//ここに壁条件がない
	if (look_f && front_count <= right_count
			&& front_count <= left_count && front_count <= back_count) {
		if ((direction==1 && y>=14) ||
			(direction==2 && x>=14) ||
			(direction==3 && y<=1) ||
			(direction==4 && x<=1) ){
			kitikukan = 0;
		}else{
			kitikukan = 1;
		}

	} else {
		kitikukan = 0;
	}


}

void compress_kitiku(int *x,int *y,int *direction,int *kitiku_distance) {
	*kitiku_distance = 1;
	int kitiku = 1;
	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;
	_Bool look_f,look_r,look_l;
	int x_now,y_now,direction_now;
	int x_front,y_front,x_right,y_right,x_left,y_left;
	int direction_right,direction_left;
	unsigned short front_count, right_count, back_count, left_count;
	x_now=*x;y_now=*y;direction_now=*direction;
	while (1) {
		update_coordinate(&x_now,&y_now,direction_now);
		x_front=x_now;y_front=y_now;x_right=x_now;y_right=y_now;x_left=x_now;y_left=y_now;

		update_coordinate(&x_front,&y_front,direction_now);
		get_wall_look(x_front,y_front,direction_now,&front_wall,&right_wall,&left_wall);
		look_f=(front_wall && right_wall && left_wall);


		if(direction_now==4){direction_right=1;}else{direction_right=direction_now+1;}
		update_coordinate(&x_right,&y_right,direction_right);
		get_wall_look(x_right,y_right,direction_right,&front_wall,&right_wall,&left_wall);
		look_r=(front_wall && right_wall && left_wall);


		if(direction_now==1){direction_left=4;}else{direction_left=direction_now-1;}
		update_coordinate(&x_left,&y_left,direction_left);
		get_wall_look(x_left,y_left,direction_left,&front_wall,&right_wall,&left_wall);
		look_l=(front_wall && right_wall && left_wall);

		get_wall(x_now,y_now,direction_now,&front_wall,&right_wall,&left_wall);

		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x_now,y_now,direction_now);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		// 移動の優先順位 ： 前→右→左→後
		if (walk_count[x_now][y_now] <= 1) {
			//goal間近で停止
			break;
		}
		if (direction_now==1 && y_now>=14) {break;}
		if (direction_now==2 && x_now>=14) {break;}
		if (direction_now==3 && y_now<=1) {break;}
		if (direction_now==4 && x_now<=1) {break;}
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
		// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode=1;
		break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
		// 直進
			if(look_f){
				*kitiku_distance += 2;
			}else{
				kitiku = 0;
				break;
			}
		}
		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
		// 右旋回
			if(look_r){
				kitiku = 0;
				break;
			}else{
				kitiku = 0;
				break;
			}
			direction_now++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
		// 左旋回
			if(look_l){
				kitiku = 0;
				break;
			}else{
				kitiku = 0;
				break;
			}
			direction_now--;
		}
		if(back_count < front_count && back_count < right_count
								&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
			kitiku = 0;
			break;
			direction_now+=2;
		}


		if (direction_now == 5) {
			direction_now = 1;
		}
		if (direction_now == 6) {
			direction_now = 2;
		}
		if (direction_now == 0) {
			direction_now = 4;
		}
		if (direction_now == -1) {
			direction_now = 3;
		}
		if (kitiku == 0) {

			break;
		}

	}

	int direction2=direction_now+2;
	if (direction2 == 5) {
				direction2 = 1;
			}
			if (direction2 == 6) {
				direction2 = 2;
			}
			if (direction2 == 0) {
				direction2 = 4;
			}
			if (direction2 == -1) {
				direction2 = 3;
			}

	update_coordinate(&x_now,&y_now,direction2);

	*x=x_now;
	*y=y_now;
	*direction=direction_now;

}


void create_StepCountMap_queue(void){

	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	for(uint8_t xx = 0;xx <= 15;xx++){
		for(uint8_t yy = 0;yy <= 15;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);


	walk_count[GOAL_X][GOAL_Y] = 0;
	walk_count[GOAL_X + 1][GOAL_Y] = 0;
	walk_count[GOAL_X][GOAL_Y + 1] = 0;
	walk_count[GOAL_X + 1][GOAL_Y + 1] = 0;
	pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y);
	pushStack_walk(&stack_x,GOAL_X + 1);pushStack_walk(&stack_y,GOAL_Y);
	pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y + 1);
	pushStack_walk(&stack_x,GOAL_X + 1);pushStack_walk(&stack_y,GOAL_Y + 1);
	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	unsigned short count_number = 1;
	unsigned short Xcoordinate,Ycoordinate;
	unsigned short wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= 254) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= 14) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= 14) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != 15 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 15 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		count_number=walk_count[Xcoordinate][Ycoordinate] + 1;

		}

}

void create_StepCountMapBack_queue(void){
	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	for(uint8_t xx = 0;xx <= 15;xx++){
		for(uint8_t yy = 0;yy <= 15;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);


	walk_count[0][0] = 0;
	pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	unsigned short coordinate;
	unsigned short count_number = 1;
	unsigned short Xcoordinate,Ycoordinate;
	unsigned short wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= 254) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		coordinate = (Xcoordinate * 16) + Ycoordinate;
		if (Ycoordinate <= 14) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= 14) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != 15 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 15 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		count_number=walk_count[Xcoordinate][Ycoordinate] + 1;

		}

}



void create_StepCountMap_unknown(void){
	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	unsigned short goalX,goalY;
	for(uint8_t xx = 0;xx <= 15;xx++){
		for(uint8_t yy = 0;yy <= 15;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}

	initStack_walk(&stack_x);
	initStack_walk(&stack_y);

	while (1) {

			goalX = popStack_walk(&g_Goal_x);
			goalY = popStack_walk(&g_Goal_y);
			//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
			//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
			if (goalX == MAX_WALKCOUNT_DIJKSTRA || goalY == MAX_WALKCOUNT_DIJKSTRA) {
				//printf("stack_end\n");
				break;
			}
			walk_count[goalX][goalY] = 0;
			pushStack_walk(&stack_x,goalX);pushStack_walk(&stack_y,goalY);
	}
	if(stack_x.tail == stack_x.head){
		walk_count[0][0] = 0;
		pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
		if (Dijkstra_maker_flag>=1){
			Dijkstra_maker_flag=2;
		}else{
			Dijkstra_maker_flag=1;
		}
	}else{
		Dijkstra_maker_flag=0;
	}
	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	unsigned short count_number = 1;
	unsigned short Xcoordinate,Ycoordinate;
	unsigned short wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= 254) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= 14) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= 14) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != 15 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 15 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		count_number=walk_count[Xcoordinate][Ycoordinate] + 1;

		}

}




void route_Dijkstra(void){
	STACK_T stack_x;
	STACK_T stack_y;
	STACK_T stack_matrix;//行列
	STACK_T stack_x_unknow;
	STACK_T stack_y_unknow;
	STACK_T stack_matrix_unknow;//行列
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);
	initStack_walk(&stack_matrix);
	initStack_walk(&g_Goal_x);
	initStack_walk(&g_Goal_y);

	pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
	pushStack_walk(&stack_matrix,ROW);

	unsigned short front_count, right_count, back_count, left_count;

	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;

	int x = 0;
	int y = 0;
	int direction = 1;


	while (1) {
//		if (mode_safty == 1) {break;}
		update_coordinate(&x,&y,direction);

		if((x == GOAL_X || x == GOAL_X+1) && (y == GOAL_Y || y == GOAL_Y+1)){
					break;
		}


		search_AroundDijkstraCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		//get_wall(x,y,direction,&front_wall,&right_wall,&left_wall);
		//if (front_wall) {front_count = MAX_WALKCOUNT_DIJKSTRA;}
		//if (right_wall) {right_count = MAX_WALKCOUNT_DIJKSTRA;}
		//if (left_wall) {left_count = MAX_WALKCOUNT_DIJKSTRA;}

		if (front_count==MAX_WALKCOUNT_DIJKSTRA && right_count==MAX_WALKCOUNT_DIJKSTRA && left_count==MAX_WALKCOUNT_DIJKSTRA && back_count==MAX_WALKCOUNT_DIJKSTRA){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)

			break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
			// 直進
			switch (direction) {		//
			case 1:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 2:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 3:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 4:
				pushStack_walk(&stack_x,x-1);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			}

		}

		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			switch (direction) {		//
			case 1:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,x-1);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			switch (direction) {		//
			case 1:
				pushStack_walk(&stack_x,x-1);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,x);
				pushStack_walk(&stack_y,y-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction--;
		}

		if (direction == 5) {
			direction = 1;
		}
		if (direction == 6) {
			direction = 2;
		}
		if (direction == 0) {
			direction = 4;
		}
		if (direction == -1) {
			direction = 3;
		}

	}

	unsigned short Xcoordinate,Ycoordinate,Row_or_Column;
	while (1) {

			Xcoordinate = popStack_walk(&stack_x);
			Ycoordinate = popStack_walk(&stack_y);
			Row_or_Column = popStack_walk(&stack_matrix);
			//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
			//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
			if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
				//printf("stack_end\n");
				break;
			}
			if(Row_or_Column==ROW && ((wall.row_look[Ycoordinate] & (1 << Xcoordinate)) == 0)){
				//pushStack_walk(&stack_x_unknow,Xcoordinate);
				//pushStack_walk(&stack_y_unknow,Ycoordinate);
				//pushStack_walk(&stack_matrix_unknow,Row_or_Column);
				walk_count[Xcoordinate][Ycoordinate] = 0;
				walk_count[Xcoordinate][Ycoordinate + 1] = 0;
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate);
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate+1);
			}
			if(Row_or_Column==COLUMN && ((wall.column_look[Xcoordinate] & (1 << Ycoordinate)) == 0)){
				//pushStack_walk(&stack_x_unknow,Xcoordinate);
				//pushStack_walk(&stack_y_unknow,Ycoordinate);
				//pushStack_walk(&stack_matrix_unknow,Row_or_Column);
				walk_count[Xcoordinate][Ycoordinate] = 0;
				walk_count[Xcoordinate + 1][Ycoordinate] = 0;
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate);
				pushStack_walk(&g_Goal_x,Xcoordinate+1);pushStack_walk(&g_Goal_y,Ycoordinate);
			}
	}


}



void run_movement_continuity(int *direction,unsigned short front_count,unsigned short right_count,
		unsigned short back_count,unsigned short left_count,float input_StraightVelocity,
		float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,_Bool front_wall,_Bool right_wall,_Bool left_wall){
	MOTOR_MODE mode;
	// 移動の優先順位 ： 前→右→左→後
	if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
		// 直進
		mode.WallControlMode=1;
		mode.calMazeMode=0;
		mode.WallCutMode=0;
		straight_table2(MAZE_SECTION-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
	}
	if(right_count < front_count && right_count <= left_count && right_count <= back_count){
		// 右旋回
		slalomR(howspeed.slalom_R, OFF,EXPLORATION,0,input_StraightVelocity);
		*direction += 1;
	}
	if(left_count < front_count && left_count < right_count && left_count <= back_count){
		// 左旋回
		slalomL(howspeed.slalom_L, OFF,EXPLORATION,0,input_StraightVelocity);
		*direction -= 1;
	}
	if(back_count < front_count && back_count < right_count
			&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		mode.WallControlMode=1;
		mode.calMazeMode=0;
		mode.WallCutMode=0;
		straight_table2(MAZE_SECTION/2-MAZE_OFFSET, input_StraightVelocity,0,input_StraightVelocity,input_StraightAcceleration, mode);
		create_DijkstraMap();
		backTurn_controlWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//backTurn_hitWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		wait_ms_NoReset(100);
		mode.WallControlMode=0;
		if(front_wall){
		straight_table2(-BACK_TO_CENTER_FRONT, 0,0,-150,1000, mode);
		wait_ms_NoReset(100);
		clear_Ierror();
		reset_speed();
		mode.WallControlMode=1;
		straight_table2(BACK_TO_CENTER +MAZE_SECTION/2,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}else{
			clear_Ierror();
			mode.WallControlMode=0;
			straight_table2(MAZE_SECTION/2+BACK_TO_CENTER-BACK_TO_CENTER_FRONT,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}
		*direction = *direction + 2;
	}

}

void create_DijkstraMap(void){
	STACK_T stack_x;
	STACK_T stack_y;
	STACK_T stack_matrix;//行列
	STACK_T stack_direction;//向き(0北　1北東　2東　3南東　4南　5南西　6西　7北西　8エラー)
	STACK_T stack_cost;//引かれるコスト
	int16_t VerticalCost=VERTICALCOST;
	int16_t DiagonalCost=DIAGONALCOST;
	int16_t discount_v[V_NUM_MAX]={180,118,100,91,90};
	int16_t discount_d[D_NUM_MAX]={127,91,79,71,65,64};
	int16_t dis_cost_in;
	//printf("%d,%d,%d,%d,%d\n",discount_v[0],discount_v[1],discount_v[2],discount_v[3],discount_v[4]);
	//printf("%d,%d,%d,%d,%d,%d\n",discount_d[0],discount_d[1],discount_d[2],discount_d[3],discount_d[4],discount_d[5]);
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);
	initStack_walk(&stack_matrix);
	initStack_walk(&stack_direction);
	initStack_walk(&stack_cost);
	for(int i=0;i<=15;i++){
		for(int j=0;j<=14;j++){
			Dijkstra.column_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
			Dijkstra.row_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
		}
	}
	Dijkstra.row_count[GOAL_X][GOAL_Y]=0;
	Dijkstra.row_count[GOAL_X+1][GOAL_Y]=0;
	Dijkstra.column_count[GOAL_Y][GOAL_X]=0;
	Dijkstra.column_count[GOAL_Y+1][GOAL_X]=0;
	pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y);
	pushStack_walk(&stack_matrix,ROW);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);
	pushStack_walk(&stack_x,GOAL_X+1);pushStack_walk(&stack_y,GOAL_Y);
	pushStack_walk(&stack_matrix,ROW);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);
	pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y);
	pushStack_walk(&stack_matrix,COLUMN);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);
	pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y+1);
	pushStack_walk(&stack_matrix,COLUMN);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);



	unsigned short count_number = 1;
	unsigned short Xcoordinate,Ycoordinate,Row_or_Column,Direction,dis_cost;
	while (1) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		Row_or_Column = popStack_walk(&stack_matrix);
		Direction = popStack_walk(&stack_direction);
		dis_cost = popStack_walk(&stack_cost);
		//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
		//printf("cost_num %d\n",dis_cost);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}
		if(Row_or_Column==ROW){
			if(Ycoordinate <= 13){
				if(Direction==SLANT_NORTH){
					dis_cost_in=dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
					VerticalCost=discount_v[dis_cost_in];
				}else{VerticalCost=discount_v[0];dis_cost_in=0;}
				if((wall.row[Ycoordinate+1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate+1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost){
					Dijkstra.row_count[Xcoordinate][Ycoordinate+1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate + 1);
					pushStack_walk(&stack_matrix,ROW);
					pushStack_walk(&stack_direction,SLANT_NORTH);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Ycoordinate >= 1) {
				if(Direction==SLANT_SOUTH){
					dis_cost_in=dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
					VerticalCost=discount_v[dis_cost_in];
				}else{VerticalCost=discount_v[0];dis_cost_in=0;}
				if((wall.row[Ycoordinate-1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost){
					Dijkstra.row_count[Xcoordinate][Ycoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate - 1);
					pushStack_walk(&stack_matrix,ROW);
					pushStack_walk(&stack_direction,SLANT_SOUTH);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Xcoordinate <= 14) {
				if(Direction==SLANT_SOUTH_EAST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate][Xcoordinate]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_SOUTH_EAST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
				if(Direction==SLANT_NORTH_EAST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate] & (1 << (Ycoordinate+1)))==0 && Dijkstra.column_count[Ycoordinate+1][Xcoordinate]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate+1][Xcoordinate]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate+1);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_NORTH_EAST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Xcoordinate >= 1) {
				if(Direction==SLANT_SOUTH_WEST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate-1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate][Xcoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate-1);
					pushStack_walk(&stack_y,Ycoordinate);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_SOUTH_WEST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
				if(Direction==SLANT_NORTH_WEST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate-1] & (1 << (Ycoordinate+1)))==0 && Dijkstra.column_count[Ycoordinate+1][Xcoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate+1][Xcoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate-1);
					pushStack_walk(&stack_y,Ycoordinate+1);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_NORTH_WEST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}

		}
		if(Row_or_Column==COLUMN){
					if(Xcoordinate <= 13){
						if(Direction==SLANT_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
							VerticalCost=discount_v[dis_cost_in];
						}else{VerticalCost=discount_v[0];dis_cost_in=0;}
						if((wall.column[Xcoordinate+1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate+1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost){
							Dijkstra.column_count[Ycoordinate][Xcoordinate+1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost;
							pushStack_walk(&stack_x,Xcoordinate + 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,COLUMN);
							pushStack_walk(&stack_direction,SLANT_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Xcoordinate >= 1) {
						if(Direction==SLANT_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
							VerticalCost=discount_v[dis_cost_in];
						}else{VerticalCost=discount_v[0];dis_cost_in=0;}
						if((wall.column[Xcoordinate-1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost){
							Dijkstra.column_count[Ycoordinate][Xcoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost;
							pushStack_walk(&stack_x,Xcoordinate - 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,COLUMN);
							pushStack_walk(&stack_direction,SLANT_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Ycoordinate <= 14) {
						if(Direction==SLANT_NORTH_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate][Ycoordinate]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_NORTH_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
						if(Direction==SLANT_NORTH_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate] & (1 << (Xcoordinate+1)))==0 && Dijkstra.row_count[Xcoordinate+1][Ycoordinate]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate+1][Ycoordinate]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate + 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_NORTH_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Ycoordinate >= 1) {
						if(Direction==SLANT_SOUTH_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate-1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate][Ycoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate);
							pushStack_walk(&stack_y,Ycoordinate - 1);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_SOUTH_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
						if(Direction==SLANT_SOUTH_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate-1] & (1 << (Xcoordinate+1)))==0 && Dijkstra.row_count[Xcoordinate+1][Ycoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate+1][Ycoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate+1);
							pushStack_walk(&stack_y,Ycoordinate-1);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_SOUTH_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}

				}

		count_number+=1;

		}



}



void run_movement_suspension(int *direction, unsigned short front_count,
		unsigned short right_count, unsigned short back_count,
		unsigned short left_count, float input_StraightVelocity,
		float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,
		_Bool front_wall, _Bool right_wall, _Bool left_wall,int x,int y,uint8_t MazeRecord_mode,uint8_t Dijkstra_mode) {
	MOTOR_MODE mode;
	// 移動の優先順位 ： 前→右→左→後
	mode.WallControlMode = 1;
	mode.calMazeMode = 0;
	mode.WallCutMode = 0;
	straight_table2(MAZE_SECTION / 2 - MAZE_OFFSET - (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), input_StraightVelocity, 0, input_StraightVelocity, input_StraightAcceleration, mode);

	if(MazeRecord_mode==1){
		if(error_mode==0){
		record_in();
		}
		clear_Ierror();
		reset_gyro();
		reset_speed();
		maze_mode = 1;
	}

	if(Dijkstra_mode==1){
		create_DijkstraMap();
		route_Dijkstra(); //ダイクストラ法の結果から最短ルートをスタックに入れる
		create_StepCountMap_unknown();
		search_AroundWalkCount(&front_count, &right_count, &back_count, &left_count, x, y, *direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		if (front_count == MAX_WALKCOUNT && right_count == MAX_WALKCOUNT && left_count == MAX_WALKCOUNT && back_count == MAX_WALKCOUNT) {
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode = 1;
			g_WallControl_mode = 0;
			pl_yellow_LED_count(2 * 2 * 2 * 2 * 2);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			//break;
		}
		if (x < 0 || y < 0 || x > 15 || y > 15) {
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode = 1;
			g_WallControl_mode = 0;
			pl_yellow_LED_count(2 * 2 * 2 * 2);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			//break;
		}
	}

if(error_mode==0){
	mode.WallControlMode = 0;
	mode.WallCutMode = 0;
	mode.calMazeMode = 0;
	if (front_count <= right_count && front_count <= left_count && front_count <= back_count) {
		// 直進
		straight_table2(MAZE_SECTION / 2 + (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
	}
	if (right_count < front_count && right_count <= left_count && right_count <= back_count) {
		// 右旋回
		turning_table2(-90, 0, 0, -input_TurningVelocity, input_TurningAcceleration);
		straight_table2(MAZE_SECTION / 2 + (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
		*direction += 1;
	}
	if (left_count < front_count && left_count < right_count && left_count <= back_count) {
		// 左旋回
		turning_table2(90, 0, 0, input_TurningVelocity, input_TurningAcceleration);
		straight_table2(MAZE_SECTION / 2 + (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
		*direction -= 1;
	}
	if(back_count < front_count && back_count < right_count
			&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		backTurn_controlWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//backTurn_hitWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		wait_ms_NoReset(100);
		mode.WallControlMode=0;
		if(front_wall){
		straight_table2(-BACK_TO_CENTER_FRONT, 0,0,-150,1000, mode);
		wait_ms_NoReset(100);
		clear_Ierror();
		reset_speed();
		mode.WallControlMode=1;
		straight_table2(BACK_TO_CENTER +MAZE_SECTION/2,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}else{
			clear_Ierror();
			mode.WallControlMode=0;
			straight_table2(MAZE_SECTION/2+BACK_TO_CENTER-BACK_TO_CENTER_FRONT,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}
		*direction = *direction + 2;
	}
}

}





void AdatiWayReturn(float input_StraightVelocity, float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,int know_mode,uint8_t Dijkstra_mode) {



	//初期化
	maze_mode = 1; //迷路探索開始フラグ
	unsigned short front_count, right_count, back_count, left_count;
	int x=0;//構造体にしたい
	int y=0;
	int direction=1;
	_Bool front_wall,right_wall,left_wall;
	char timer_end_mode=0;
	int kitiku_distance;
	MOTOR_MODE mode;
	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;

	//モータenable
	pl_DriveMotor_standby(ON);
	wait_ms_NoReset(500);
	//初期位置のセンサー確認
	get_wallData_sensor(&front_wall,&right_wall,&left_wall);
	//初期位置での壁更新
	update_wall(x,y,direction,front_wall,right_wall,left_wall);
	//初期位置での迷路展開
	create_StepCountMap_queue();
	straight_table2(MAZE_SECTION/2+BACK_TO_CENTER,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);

	while (1) {

		update_coordinate(&x,&y,direction);

		get_wallData_sensor(&front_wall,&right_wall,&left_wall);

		mode.WallControlMode=1;
		mode.calMazeMode=1;
		mode.WallCutMode=0;
		straight_table2(MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		//走行中計算
		update_wall(x,y,direction,front_wall,right_wall,left_wall);
		create_StepCountMap_queue();
		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		decision_kitiku(x,y,direction,front_count,right_count,back_count,left_count);

		mode.WallCutMode=1;
		End_straight(MAZE_OFFSET, mode,right_wall,left_wall);


		//異常終了
		if (x == 0 && y == 0) {
			error_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//　時間制限
		if (g_timCount_sec>240){
			timer_end_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//正常終了
		if((x == GOAL_X || x == GOAL_X+1) && (y == GOAL_Y || y == GOAL_Y+1)){
			break;
		}

		if(know_mode==0){kitikukan = 0;}
		if (kitikukan == OFF) {

			run_movement_continuity(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall);

		} else {
			mode.WallControlMode=1;
			mode.calMazeMode=1;
			mode.WallCutMode=0;
			straight_table2(MAZE_SECTION/2-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
			compress_kitiku(&x,&y,&direction,&kitiku_distance);
			End_straight(MAZE_SECTION/2-MAZE_OFFSET,mode,1,1);
			mode.calMazeMode=0;
			straight_table2((MAZE_SECTION/2 * kitiku_distance),input_StraightVelocity,input_StraightVelocity,900,input_StraightAcceleration, mode);
		}

		if (direction >= 5) {direction = direction-4;}
		if (direction <= 0) {direction = direction+4;}

		if(error_mode==1){break;}

	}

	run_movement_suspension(&direction,front_count,right_count,back_count,left_count,
			input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
			front_wall, right_wall, left_wall, x, y, 1, 1);
	if (direction >= 5) {direction = direction-4;}
	if (direction <= 0) {direction = direction+4;}
	//straight_table2(MAZE_SECTION/2+BACK_TO_CENTER,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);

	while (1) {
		update_coordinate(&x,&y,direction);

		get_wallData_sensor(&front_wall,&right_wall,&left_wall);

		if((x == GOAL_X || x == GOAL_X+1) && (y == GOAL_Y || y == GOAL_Y+1)){
			noGoalPillarMode=1;
		}else{
			noGoalPillarMode=0;
		}

		mode.WallControlMode=1;
		mode.calMazeMode=1;
		mode.WallCutMode=0;
		straight_table2(MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		update_wall(x,y,direction,front_wall,right_wall,left_wall);
		if(Dijkstra_mode==1){
			route_Dijkstra();//ダイクストラ法の結果から最短ルートをスタックに入れる
			create_StepCountMap_unknown();
		}else{
			create_StepCountMapBack_queue();
		}
		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		decision_kitiku(x,y,direction,front_count,right_count,back_count,left_count);
		mode.WallCutMode=1;
		End_straight(MAZE_OFFSET,mode,right_wall,left_wall);

		//異常終了
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
			// 迷路破損のため、ダイクストラ法更新
			Dijkstra_maker_flag=1;
		}
		if (x<0 || y<0 || x>15 || y>15){
			// 自己位置の破損
			error_mode=1;
			g_WallControl_mode=0;
			pl_yellow_LED_count(2*2*2*2);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		if (g_timCount_sec>240){
			// 秒数エンド
			timer_end_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//正常終了
		if(x == 0 && y == 0) {
			break;
		}


		if(Dijkstra_maker_flag==1){
			run_movement_suspension(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall, x, y, 0, 1);
		}else{

			if(know_mode==0){kitikukan = 0;}
			if (kitikukan == OFF) {
				run_movement_continuity(&direction,front_count,right_count,back_count,left_count,
						input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
						front_wall, right_wall, left_wall);
			} else {
				mode.WallControlMode=1;
				mode.calMazeMode=1;
				mode.WallCutMode=0;
				straight_table2(MAZE_SECTION/2-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
				compress_kitiku(&x,&y,&direction,&kitiku_distance);
				End_straight(MAZE_SECTION/2-MAZE_OFFSET,mode,1,1);
				mode.calMazeMode=0;
				straight_table2((MAZE_SECTION/2 * kitiku_distance),input_StraightVelocity,input_StraightVelocity,1000,input_StraightAcceleration, mode);
			}

		}

		if (direction >= 5) {direction = direction-4;}
		if (direction <= 0) {direction = direction+4;}

		if(error_mode==1){break;}

		}


	mode.WallControlMode=0;
	mode.calMazeMode=0;
	mode.WallCutMode=0;
	straight_table2(MAZE_SECTION/2-MAZE_OFFSET, input_StraightVelocity,0,input_StraightVelocity,input_StraightAcceleration, mode);
	turning_table2(180,0,0,input_TurningVelocity,input_TurningAcceleration);
	pl_DriveMotor_standby(OFF); //MTU2.TSTR.BIT.CST0 = 0;
	maze_mode = 0;
	wait_ms_NoReset(100);
	maze_display();
	create_StepCountMap_queue();
	if(walk_count[0][0] == MAX_WALKCOUNT){
		error_mode = 1;
	}
   if (error_mode == 0) {
		record_in();
	} else if(timer_end_mode==0) {
		record_out();
	}else{
		record_in();
	}



}







/* the gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  const mwSize dims[]={16,16};
  unsigned char *start_of_pr;
  unsigned short data[]={1,2,3,4};
  size_t bytes_to_copy;

  (void) nlhs; (void) nrhs;  /* unused parameters */

   uint32_t* row = mxGetPr(prhs[0]);
   uint32_t* column = mxGetPr(prhs[1]);
   uint32_t* goal_x = mxGetPr(prhs[2]);
   uint32_t* goal_y = mxGetPr(prhs[3]);
    for(int t=0;t<15;t++){
        wall.row[t] = row[t];
        wall.column[t] = column[t];
    }
    GOAL_X = *goal_x;
    GOAL_Y = *goal_y;
    error_mode=0;
    Dijkstra_maker_flag=0;
  /* call the computational subroutine */
    create_StepCountMap_queue();

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[0] = mxCreateNumericArray(NDIMS,dims,mxUINT32_CLASS,mxREAL);

  /* populate the real part of the created array */
  start_of_pr = (unsigned char *)mxGetData(plhs[0]);
  bytes_to_copy = TOTAL_ELEMENTS * mxGetElementSize(plhs[0]);
  memcpy(start_of_pr,walk_count,bytes_to_copy);
}
