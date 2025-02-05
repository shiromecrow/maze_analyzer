#include <string.h> /* needed for memcpy() */
#include <stdint.h> /* needed for memcpy() */
#include <math.h> 
#include "mex.h"

/*
 * run_maze_solve.c - example found in API guide
 *
 */


#define MAZE_SQUARE_NUM 16
#define PASS_NUM 501

#define NDIMS 2
#define TOTAL_ELEMENTS0 501
#define TOTAL_ELEMENTS3 (MAZE_SQUARE_NUM*(MAZE_SQUARE_NUM-1))

#define SLANT_PASS_COUNT 500


#define MAX_QUEUE_NUM 1500
#define ROW 0
#define COLUMN 1
#define MAX_WALKCOUNT 65535
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
#define V_NUM_MAX 10
#define D_NUM_MAX 14

#define SLANT_NORTH 0
#define SLANT_NORTH_EAST 1
#define SLANT_EAST 2
#define SLANT_SOUTH_EAST 3
#define SLANT_SOUTH 4
#define SLANT_SOUTH_WEST 5
#define SLANT_WEST 6
#define SLANT_NORTH_WEST 7

#define BATT_MAX 4
#define MOTOR_BREAK 4

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
	uint32_t row[MAZE_SQUARE_NUM-1];
	uint32_t column[MAZE_SQUARE_NUM-1];
	uint32_t row_look[MAZE_SQUARE_NUM-1];
	uint32_t column_look[MAZE_SQUARE_NUM-1];

}WALL;

typedef struct{
	uint16_t row_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
	uint16_t column_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
    uint16_t row_direction[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
	uint16_t column_direction[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
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
WALL record;
uint32_t GOAL_X,GOAL_Y;
int kitikukan;
int g_timCount_sec=10;
int noGoalPillarMode;
uint8_t g_WallControl_mode;
parameter_speed speed300_exploration;
unsigned short pass_count;
char no_safty;
char record_mode;
char highspeed_mode;
float g_V_battery_mean=4.1;

//入出力変数
STACK_T g_Goal_x;
STACK_T g_Goal_y;
char Dijkstra_maker_flag;
uint32_t walk_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM]; //歩数いれる箱
char error_mode;
DIJKSTRA Dijkstra;
uint16_t g_sensor_front,g_sensor_right,g_sensor_left;
int x,y,direction;

int pass[PASS_NUM]; //1f 2r 3l
int pass_log[PASS_NUM]; //1f 2r 3l
int x_log[PASS_NUM]; //1f 2r 3l
int y_log[PASS_NUM]; //1f 2r 3l

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
void pl_R_DriveMotor_mode(int a){}
void pl_L_DriveMotor_mode(int a){}
void pl_FunMotor_start(void){}
void pl_FunMotor_stop(void){}
void pl_FunMotor_duty(float a){}
float get_center_velocity(parameter_speed Howspeed, int pass_number) {
	float End_velocity;
		End_velocity = 0;
	return End_velocity;
}

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



void turn90R(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn90L(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}


void turn180R(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn180L(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn45inR(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn45inL(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn135inR(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}

void turn135inL(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {

}


void turn45outR(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

}


void turn45outL(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

}



void turn135outR(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

}


void turn135outL(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

}


void V90R(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

}

void V90L(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {

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
		if (y <= MAZE_SQUARE_NUM-2) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(front_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(left_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		if (x <= MAZE_SQUARE_NUM-2) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(right_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(front_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		if (y <= MAZE_SQUARE_NUM-2) {
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

		if (x <= MAZE_SQUARE_NUM-2) {
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

		if (y <= MAZE_SQUARE_NUM-2) {
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
		if (y <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
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
		if (x <= MAZE_SQUARE_NUM-2) {
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
		if (y <= MAZE_SQUARE_NUM-2) {
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
		if (y <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
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
		if (x <= MAZE_SQUARE_NUM-2) {
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
		if (y <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		break;
	}

}








void search_AroundWalkCount(unsigned short *front_count,unsigned short *right_count,unsigned short *back_count,unsigned short *left_count,int x,int y,int direction){
//int direction,int x_coordinate,int y_coordinate
	unsigned short north_count,east_count,south_count,west_count;
//	unsigned short front_count, right_count, back_count, left_count;

	if (y >= MAZE_SQUARE_NUM-1) {north_count = MAX_WALKCOUNT;}
	else {north_count = walk_count[x][y + 1];}

	if (x >= MAZE_SQUARE_NUM-1) {east_count = MAX_WALKCOUNT;}
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

	if (y >= MAZE_SQUARE_NUM-1) {north_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {north_count = Dijkstra.row_count[x][y];}

	if (x >= MAZE_SQUARE_NUM-1) {east_count = MAX_WALKCOUNT_DIJKSTRA;}
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


void update_coordinate(int *x_up,int *y_up,int direction){
// int direction,int *x_coordinate,int *y_coordinate
//	*direction = *direction % 4;
//	if (*direction <= 0) {
//		*direction = *direction+4;
//	}
	switch (direction) {
	case 1://北
		*y_up += 1;
		break;
	case 2://東
		*x_up += 1;
		break;
	case 3://南
		*y_up -= 1;
		break;
	case 4://西
		*x_up -= 1;
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
		if ((direction==1 && y>=MAZE_SQUARE_NUM-2) ||
			(direction==2 && x>=MAZE_SQUARE_NUM-2) ||
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
		if (direction_now==1 && y_now>=MAZE_SQUARE_NUM-2) {break;}
		if (direction_now==2 && x_now>=MAZE_SQUARE_NUM-2) {break;}
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
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
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
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= MAZE_SQUARE_NUM*MAZE_SQUARE_NUM-2) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
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
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
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
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= MAZE_SQUARE_NUM*MAZE_SQUARE_NUM-2) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
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
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
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
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (count_number <= MAZE_SQUARE_NUM*MAZE_SQUARE_NUM-2) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
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

	int xd = 0;
	int yd = 0;
	int direction_d = 1;


	while (1) {
//		if (mode_safty == 1) {break;}
		update_coordinate(&xd,&yd,direction_d);

		if((xd == GOAL_X || xd == GOAL_X+1) && (yd == GOAL_Y || yd == GOAL_Y+1)){
					break;
		}


		search_AroundDijkstraCount(&front_count,&right_count,&back_count,&left_count,xd,yd,direction_d);
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
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 3:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 4:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			}

		}

		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction_d++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction_d--;
		}

		if (direction_d == 5) {
			direction_d = 1;
		}
		if (direction_d == 6) {
			direction_d = 2;
		}
		if (direction_d == 0) {
			direction_d = 4;
		}
		if (direction_d == -1) {
			direction_d = 3;
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


void create_DijkstraMap(void){
	STACK_T stack_x;
	STACK_T stack_y;
	STACK_T stack_matrix;//行列
	STACK_T stack_direction;//向き(0北　1北東　2東　3南東　4南　5南西　6西　7北西　8エラー)
	STACK_T stack_cost;//引かれるコスト
	int16_t VerticalCost=VERTICALCOST;
	int16_t DiagonalCost=DIAGONALCOST;
 	int16_t discount_v[V_NUM_MAX]={180,118,100,88,80,73,68,64,61,60};
 	int16_t discount_d[D_NUM_MAX]={127,91,79,71,65,60,56,53,50,48,46,44,42,42};

    int16_t dis_cost_in;
	//printf("%d,%d,%d,%d,%d\n",discount_v[0],discount_v[1],discount_v[2],discount_v[3],discount_v[4]);
	//printf("%d,%d,%d,%d,%d,%d\n",discount_d[0],discount_d[1],discount_d[2],discount_d[3],discount_d[4],discount_d[5]);
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);
	initStack_walk(&stack_matrix);
	initStack_walk(&stack_direction);
	initStack_walk(&stack_cost);
	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		for(int j=0;j<=MAZE_SQUARE_NUM-2;j++){
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
			if(Ycoordinate <= MAZE_SQUARE_NUM-3){
				if(Direction==SLANT_NORTH){
					dis_cost_in=dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
					VerticalCost=discount_v[dis_cost_in];
				}else{VerticalCost=discount_v[0];dis_cost_in=0;}
				if((wall.row[Ycoordinate+1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate+1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost){
					Dijkstra.row_count[Xcoordinate][Ycoordinate+1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost;
                    Dijkstra.row_direction[Xcoordinate][Ycoordinate+1]=Direction;
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
                    Dijkstra.row_direction[Xcoordinate][Ycoordinate-1]=Direction;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate - 1);
					pushStack_walk(&stack_matrix,ROW);
					pushStack_walk(&stack_direction,SLANT_SOUTH);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
				if(Direction==SLANT_SOUTH_EAST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate][Xcoordinate]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
                    Dijkstra.column_direction[Ycoordinate][Xcoordinate]=Direction;
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
                    Dijkstra.column_direction[Ycoordinate+1][Xcoordinate]=Direction;
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
                    Dijkstra.column_direction[Ycoordinate][Xcoordinate-1]=Direction;
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
                    Dijkstra.column_direction[Ycoordinate+1][Xcoordinate-1]=Direction;
					pushStack_walk(&stack_x,Xcoordinate-1);
					pushStack_walk(&stack_y,Ycoordinate+1);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_NORTH_WEST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}

		}
		if(Row_or_Column==COLUMN){
					if(Xcoordinate <= MAZE_SQUARE_NUM-3){
						if(Direction==SLANT_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
							VerticalCost=discount_v[dis_cost_in];
						}else{VerticalCost=discount_v[0];dis_cost_in=0;}
						if((wall.column[Xcoordinate+1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate+1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost){
							Dijkstra.column_count[Ycoordinate][Xcoordinate+1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost;
                            Dijkstra.column_direction[Ycoordinate][Xcoordinate+1]=Direction;
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
                            Dijkstra.column_direction[Ycoordinate][Xcoordinate-1]=Direction;
							pushStack_walk(&stack_x,Xcoordinate - 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,COLUMN);
							pushStack_walk(&stack_direction,SLANT_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
						if(Direction==SLANT_NORTH_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate][Ycoordinate]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
                            Dijkstra.row_direction[Xcoordinate][Ycoordinate]=Direction;
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
                            Dijkstra.row_direction[Xcoordinate+1][Ycoordinate]=Direction;
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
                            Dijkstra.row_direction[Xcoordinate][Ycoordinate-1]=Direction;
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
                            Dijkstra.row_direction[Xcoordinate+1][Ycoordinate-1]=Direction;
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
		if (x < 0 || y < 0 || x > MAZE_SQUARE_NUM-1 || y > MAZE_SQUARE_NUM-1) {
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


void pass_maker_Dijkstra(void){

	uint16_t front_count, right_count, back_count, left_count;

	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;

	int x = 0;
	int y = 0;
	int direction = 1;
	pass_count = 0;
	create_DijkstraMap();
	maze_display_Dijkstra();
    int count=0;
	pass[0] = 1;

    x_log[count]=x;
    y_log[count]=y;
    count++;
	while (1) {
//		if (mode_safty == 1) {break;}
		update_coordinate(&x,&y,direction);

        x_log[count]=x;
        y_log[count]=y;
        count++;

		if((x == GOAL_X || x == GOAL_X+1) && (y == GOAL_Y || y == GOAL_Y+1)){

			if (pass[pass_count] >= 0) {
					} else {
						pass_count++;
					}
					pass[pass_count] = pass[pass_count] + 1;
		//			wait(10);
		//			maze_display();
		//			x = 0;
		//			y = 0;
					direction = direction + 2;
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
					break;

		}

		get_wall(x,y,direction,&front_wall,&right_wall,&left_wall);
		search_AroundDijkstraCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT_DIJKSTRA;}
		if (right_wall) {right_count = MAX_WALKCOUNT_DIJKSTRA;}
		if (left_wall) {left_count = MAX_WALKCOUNT_DIJKSTRA;}

		if (front_count==MAX_WALKCOUNT_DIJKSTRA && right_count==MAX_WALKCOUNT_DIJKSTRA && left_count==MAX_WALKCOUNT_DIJKSTRA && back_count==MAX_WALKCOUNT_DIJKSTRA){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)

			break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
			// 直進
			if (pass[pass_count] >= 0) {} else {pass_count++;}
			pass[pass_count] = pass[pass_count] + 2;
		}
		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			pass_count++;
			pass[pass_count] = -2;
			direction++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			pass_count++;
			pass[pass_count] = -3;
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
}


void run_shortest(float inspeed, float inacc, int stmass, char pass_mode, char fun_mode,
		char slant_mode, parameter_speed howspeed,float fun_ratio,char mollifier_mode) {
	unsigned char slant_count;
	int slant_direction;
	float first_v,last_v;
	float end_velocity;

//	unsigned short front_count, right_count, back_count, left_count;
//
//	_Bool front_wall;
//	_Bool right_wall;
//	_Bool left_wall;
//
//	int x = 0;
//	int y = 0;
//	int direction = 1;
	slant_direction = -2;

	MOTOR_MODE mode;
	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;

	//highspeed_mode = 1;
	for(int i = 0; i < PASS_NUM; i++){pass[i] = 0;}
	for(int i = 0; i <= MAZE_SQUARE_NUM-2; i++){
		record.row[i] = wall.row[i];
		record.column[i] = wall.column[i];
		record.row_look[i] = wall.row_look[i];
		record.column_look[i] = wall.column_look[i];
		wall.row_look[i] = ~wall.row_look[i];
		wall.column_look[i] = ~wall.column_look[i];
		wall.row[i] = wall.row[i] | wall.row_look[i];
		wall.column[i] = wall.column[i] | wall.column_look[i];
	}
	//pass_maker();
   pass_maker_Dijkstra();

	pass_count = 1;
if(pass_mode==1){
	while (1) {		//パス圧縮
//		if (mode_safty == 1) {
//
//			break;
//		}
		if (pass[pass_count] == 0) {
			break;
		}

		if (pass[pass_count] == -2 && pass[pass_count - 1] >= 1	//右90度大回りの条件
		&& pass[pass_count + 1] >= 1) {
			pass[pass_count - 1] = pass[pass_count - 1] - 1;	//前90直進の削除
			pass[pass_count + 1] = pass[pass_count + 1] - 1;	//後90直進の削除
			pass[pass_count] = -4;		//右90度大回り

		}
		if (pass[pass_count] == -3 && pass[pass_count - 1] >= 1	//左90度大回りの条件
		&& pass[pass_count + 1] >= 1) {
			pass[pass_count - 1] = pass[pass_count - 1] - 1;	//前90直進の削除
			pass[pass_count + 1] = pass[pass_count + 1] - 1;	//後90直進の削除
			pass[pass_count] = -5;		//左90度大回り

		}
		if (pass[pass_count - 1] >= 1 && pass[pass_count] == -2
				&& pass[pass_count + 1] == -2 && pass[pass_count + 2] >= 1) {//右180度大回りの条件
			pass[pass_count - 1] = pass[pass_count - 1] - 1;
			pass[pass_count] = -6;
			pass[pass_count + 1] = -1;
			pass[pass_count + 2] = pass[pass_count + 2] - 1;

		}
		if (pass[pass_count - 1] >= 1 && pass[pass_count] == -3
				&& pass[pass_count + 1] == -3 && pass[pass_count + 2] >= 1) {//左180度大回りの条件
			pass[pass_count - 1] = pass[pass_count - 1] - 1;
			pass[pass_count] = -7;
			pass[pass_count + 1] = -1;
			pass[pass_count + 2] = pass[pass_count + 2] - 1;
		}
		if (pass[pass_count] == -2 && pass[pass_count - 1] == -3	//左90度大回りの条件

				) {
		}
//		if(){}
		if (pass[pass_count - 1] == 0) {
			pass[pass_count - 1] = -1;		//passが0になってしまったときの対策
		}

		pass_count++;
	}

	pass_count = 1;
	if (slant_mode == 1) {
		while (1) {		//斜め入出の圧縮
			if (pass[pass_count] == 0) {
				break;
			}

			if (pass[pass_count - 1] >= 1) {
				if (pass[pass_count] == -2 || pass[pass_count] == -3) {
//***************************************************************************************入りのモーションstart
					if (pass[pass_count] == -2 && pass[pass_count + 1] == -3) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -8;		//右45
					}
					if (pass[pass_count] == -3 && pass[pass_count + 1] == -2) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -9;		//左45
					}
					if (pass[pass_count] == -2 && pass[pass_count + 1] == -2) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -10;		//右135
						pass[pass_count + 1] = -1;
					}
					if (pass[pass_count] == -3 && pass[pass_count + 1] == -3) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -11;		//左135
						pass[pass_count + 1] = -1;
					}
//***************************************************************************************入りのモーションend

//***************************************************************************************途中のモーションstart
					while (pass[pass_count] <= -1) {
						pass_count++;
					}
//***************************************************************************************途中のモーションend

//***************************************************************************************出のモーションstart
					if (pass[pass_count - 1] == -2) {
						if (pass[pass_count - 2] == -2) {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -14;		//右135
							pass[pass_count - 2] = -1;
						} else {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -12;		//右45
						}

					}
					if (pass[pass_count - 1] == -3) {
						if (pass[pass_count - 2] == -3) {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -15;		//左135
							pass[pass_count - 2] = -1;
						} else {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -13;		//左45
						}

					}
//***************************************************************************************出のモーションend
				}
			}
			//		if(){}

			pass_count++;
		}

		pass_count = 1;
		while (1) {		//斜の圧縮
			if (pass[pass_count] == 0) {
				break;
			}

			if (pass[pass_count] == -8 || pass[pass_count] == -9
					|| pass[pass_count] == -10 || pass[pass_count] == -11) {
				if (pass[pass_count] == -8 || pass[pass_count] == -10) {
					slant_direction = -3;
				}
				if (pass[pass_count] == -9 || pass[pass_count] == -11) {
					slant_direction = -2;
				}
				pass_count++;
				if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
					pass_count++;
				}
				if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
					pass_count++;
				}
				if (pass[pass_count] >= -3) {
					slant_count = pass_count;
					pass[slant_count] = SLANT_PASS_COUNT+1;
					pass_count++;
				}

				//***************************************************************************************途中のモーションstart
				while (pass[pass_count] >= -3) {
					if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
						pass_count++;
					}
					if (pass[pass_count] == -12 || pass[pass_count] == -13
							|| pass[pass_count] == -14
							|| pass[pass_count] == -15) {
						break;
					}
					if (pass[pass_count] == slant_direction) {
						pass[slant_count] = pass[slant_count] - 1;
						slant_count = pass_count;
						if (slant_direction == -2) {
							pass[pass_count] = -16;
						}
						if (slant_direction == -3) {
							pass[pass_count] = -17;
						}

					} else {
						if (pass[slant_count] >= SLANT_PASS_COUNT) {
							pass[pass_count] = -1;
						} else {
							slant_count = pass_count;
							pass[slant_count] = SLANT_PASS_COUNT;
						}
						pass[slant_count] = pass[slant_count] + 1;
						if (slant_direction == -2) {
							slant_direction = -3;
						} else {
							slant_direction = -2;
						}

					}

					pass_count++;
				}
				//***************************************************************************************途中のモーションend

			}

			//		if(){}

			pass_count++;
		}
		pass_count=0;
		while (1) {		//パス圧縮

			if (pass[pass_count] == SLANT_PASS_COUNT) {
				pass[pass_count] =-1;

			}
			if (pass[pass_count] == 0) {
				break;
			}
			pass_count++;
		}

	}
}
	int j = 0;
	while (pass[j] != 0) {
		printf("pass_count %d pass %d\n", j, pass[j]);
		j++;
	}
	int pass_count2;
	pass_count2=0;
	while(pass[pass_count2] == -1){
		pass_count2++;
	}
	end_velocity=get_center_velocity(howspeed,pass[pass_count2]);
	printf("%d,%f\n",pass_count2, end_velocity);

	wait_ms_NoReset(500);
	pl_DriveMotor_standby(ON);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	no_safty = 1;
	no_safty = 0;
	clear_Ierror();
	////wall_control_mode = 1;
	if (fun_mode == 1) {
		pl_FunMotor_duty(fun_ratio*BATT_MAX/g_V_battery_mean);
		pl_FunMotor_start();
		wait_ms_NoReset(600);
		reset_gyro();
		reset_speed();
		clear_Ierror();
//		wait_ms_NoReset(1000);
//				reset_gyro();
//				enc.sigma_error = 0;
//					Gyro.sigma_error = 0;
//				pl_FunMotor_duty(160);
//		pl_FunMotor_start();
//						wait_ms_NoReset(2000);	//候補1

	}
	maze_mode = 1;
	highspeed_mode = 1;
//	record_mode=14;
	record_mode=18;
//	encoder_PID_error=2500;
//	gyro_PID_error=1800;
	pass_count = 0;


	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.calMazeMode=0;
	mode.WallCutMode=0;
	pass_count2=0;
	while(pass[pass_count2] == -1){
		pass_count2++;
	}
	end_velocity=get_center_velocity(howspeed,pass[pass_count2]);
	straight_table2(BACK_TO_CENTER_FRONT,0,end_velocity,end_velocity,end_velocity*end_velocity/ BACK_TO_CENTER_FRONT/2, mode);


	while (pass_count < PASS_NUM) {
		pass_count2=pass_count+1;
		while(pass[pass_count2] == -1){
			pass_count2++;
		}
		end_velocity=get_center_velocity(howspeed,pass[pass_count2]);

		if (pass[pass_count] == -1) {
			pass_count++;
		}
		else if (pass[pass_count] == -2) {

			slalomR(howspeed.slalom_R, OFF,SHORTEST,mollifier_mode,end_velocity);

			pass_count++;
		}
		else if (pass[pass_count] == -3) {

			slalomL(howspeed.slalom_L, OFF,SHORTEST,mollifier_mode,end_velocity);

			pass_count++;
		}
		else if (pass[pass_count] == -4) {
			turn90R(howspeed.turn90_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -5) {
			turn90L(howspeed.turn90_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -6) {
			turn180R(howspeed.turn180_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -7) {
			turn180L(howspeed.turn180_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -8) { //入り45R
			turn45inR(howspeed.turn45in_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -9) { //入り45L
			turn45inL(howspeed.turn45in_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -10) { //入り135R
			turn135inR(howspeed.turn135in_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -11) { //入り135L
			turn135inL(howspeed.turn135in_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -12) { //出り45R
			turn45outR(howspeed.turn45out_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -13) { //出り45L
			turn45outL(howspeed.turn45out_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -14) { //出り135R
			turn135outR(howspeed.turn135out_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -15) { //出り135L
			turn135outL(howspeed.turn135out_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -16) { //V90R
			V90R(howspeed.V90_R, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] == -17) { //V90L
			V90L(howspeed.V90_L, OFF,mollifier_mode,end_velocity);
			pass_count++;
		}
		else if (pass[pass_count] >= 1) {
			first_v = howspeed.TurnCentervelocity;
			last_v = howspeed.TurnCentervelocity;
			if (pass_count >= 1) {

				if (pass[pass_count - 1] == -2 || pass[pass_count - 1] == -3) {
					first_v = howspeed.SlalomCentervelocity;
				}
			}
			if (pass[pass_count + 1] == -2 || pass[pass_count + 1] == -3) {
				last_v = howspeed.SlalomCentervelocity;
			}
			if (pass[pass_count] >= SLANT_PASS_COUNT) {
				mode.WallControlMode=3;
				mode.WallControlStatus=0;
				straight_table2((45 * sqrt(2) * (pass[pass_count] - SLANT_PASS_COUNT)),first_v, end_velocity,inspeed, inacc, mode);
			} else {
				mode.WallControlMode=1;
				mode.WallControlStatus=0;
				straight_table2((45 * pass[pass_count]),first_v, end_velocity,inspeed, inacc, mode);
			}

			pass_count++;
		}

		if (pass[pass_count] == 0) {
			break;
		}


//		if (mode_safty == 1) {
//
//			break;
//		}
	}

		mode.WallControlMode=1;
		mode.WallControlStatus=0;
		straight_table2(MAZE_SECTION,end_velocity, 0,inspeed, inacc, mode);
		wait_ms_NoReset(700);
		pl_FunMotor_stop();
//		turning_table(180, 0, 0, 400, 5000);

	maze_mode = 0;
	highspeed_mode = 0;
	record_mode=0;
	pl_DriveMotor_standby(OFF);
	int t = 0;

	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row[t] = record.row[t];
		wall.column[t] = record.column[t];
		t++;
	}
	t = 0;
	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row_look[t] = ~wall.row_look[t];
		wall.column_look[t] = ~wall.column_look[t];
		t++;
	}

}






/* the gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  const mwSize dims0[]={PASS_NUM,1};
     const mwSize dims3[]={MAZE_SQUARE_NUM-1,MAZE_SQUARE_NUM};
  unsigned char *start_of_pr;
  unsigned short data[]={1,2,3,4};
  int coordinate[3];
  char all_mode[2];
  size_t bytes_to_copy;


   uint32_t log_row[MAZE_SQUARE_NUM-1];
   uint32_t log_column[MAZE_SQUARE_NUM-1];
   uint32_t log_row_look[MAZE_SQUARE_NUM-1];
   uint32_t log_column_look[MAZE_SQUARE_NUM-1];
   uint32_t log_goal_x;
   uint32_t log_goal_y;
   int log_X;
   int log_Y;
   int log_Direction;
   uint16_t log_Sensor_front;
   uint16_t log_Sensor_right;
   uint16_t log_Sensor_left;
   char log_Dijkstra_maker;
   char log_Error;
   uint16_t log_D_row_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
   uint16_t log_D_column_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];

  (void) nlhs; (void) nrhs;  /* unused parameters */

   uint32_t* row = mxGetPr(prhs[0]);
   uint32_t* column = mxGetPr(prhs[1]);
   uint32_t* row_look = mxGetPr(prhs[2]);
   uint32_t* column_look = mxGetPr(prhs[3]);
   uint32_t* goal_x = mxGetPr(prhs[4]);
   uint32_t* goal_y = mxGetPr(prhs[5]);
   int* X = mxGetPr(prhs[6]);
   int* Y = mxGetPr(prhs[7]);
   int* Direction = mxGetPr(prhs[8]);
   uint16_t* Sensor_front = mxGetPr(prhs[9]);
   uint16_t* Sensor_right = mxGetPr(prhs[10]);
   uint16_t* Sensor_left = mxGetPr(prhs[11]);
   char* Dijkstra_maker = mxGetPr(prhs[12]);
   char* Error = mxGetPr(prhs[13]);
   uint16_t* D_row_count = mxGetPr(prhs[14]);
   uint16_t* D_column_count = mxGetPr(prhs[15]);

    for(int t=0;t<MAZE_SQUARE_NUM-1;t++){
        wall.row[t] = row[t];
        wall.column[t] = column[t];
        wall.row_look[t] = row_look[t];
        wall.column_look[t] = column_look[t];
        for(int t2=0;t2<MAZE_SQUARE_NUM;t2++){
            Dijkstra.row_count[t2][t]=D_row_count[MAZE_SQUARE_NUM*t + t2];
            Dijkstra.column_count[t2][t]=D_column_count[MAZE_SQUARE_NUM*t + t2];
        }
    }
    GOAL_X = *goal_x;
    GOAL_Y = *goal_y;
    x = *X;
    y = *Y;
    direction = *Direction;
    g_sensor_front = *Sensor_front;
    g_sensor_right = *Sensor_right;
    g_sensor_left = *Sensor_left;
    Dijkstra_maker_flag = *Dijkstra_maker;
    error_mode = *Error;
  /* call the computational subroutine */
    for(int i=0;i<PASS_NUM;i++){
        pass[i]=0;
        x_log[i]=0;
        y_log[i]=0;
    }
    run_shortest(3000,10000,0,1,1,1,speed300_exploration,0.99,1);


  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[0] = mxCreateNumericArray(2,dims0,mxINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[0]);
  bytes_to_copy = PASS_NUM * mxGetElementSize(plhs[0]);
  memcpy(start_of_pr,pass,bytes_to_copy);

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[1] = mxCreateNumericArray(2,dims0,mxINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[1]);
  bytes_to_copy = PASS_NUM * mxGetElementSize(plhs[1]);
  memcpy(start_of_pr,x_log,bytes_to_copy);

      /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[2] = mxCreateNumericArray(2,dims0,mxINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[2]);
  bytes_to_copy = PASS_NUM * mxGetElementSize(plhs[2]);
  memcpy(start_of_pr,y_log,bytes_to_copy);

       /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[3] = mxCreateNumericArray(2,dims3,mxUINT16_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[3]);
  bytes_to_copy = TOTAL_ELEMENTS3 * mxGetElementSize(plhs[3]);
  memcpy(start_of_pr,Dijkstra.row_count,bytes_to_copy);  

   /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[4] = mxCreateNumericArray(2,dims3,mxUINT16_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[4]);
  bytes_to_copy = TOTAL_ELEMENTS3 * mxGetElementSize(plhs[4]);
  memcpy(start_of_pr,Dijkstra.column_count,bytes_to_copy); 


}
