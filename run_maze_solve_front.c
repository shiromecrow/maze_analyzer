#include <string.h> /* needed for memcpy() */
#include <stdint.h> /* needed for memcpy() */
#include "math.h"
#include "mex.h"

/*
 * run_maze_solve.c - example found in API guide
 *
 */

#define MAZE_SQUARE_NUM 32

#define NDIMS 2
#define TOTAL_ELEMENTS0 (MAZE_SQUARE_NUM-1)
#define TOTAL_ELEMENTS1 3
#define TOTAL_ELEMENTS2 (MAZE_SQUARE_NUM*MAZE_SQUARE_NUM)
#define TOTAL_ELEMENTS3 (MAZE_SQUARE_NUM*(MAZE_SQUARE_NUM-1))
#define TOTAL_ELEMENTS4 2

#define INTERRUPT_TIME 0.001
#define MOTOR_STOP 0
#define MOTOR_FRONT 1
#define MOTOR_BACK 2
#define MOTOR_BREAK 3
#define MAX_QUEUE_NUM 4000
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
#define MOLLIFIER_INTEGRAL 0.444




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

#define OFFSET_CONTROL_IN_SLALOM 0//大回りのオフセットでの壁制御(入り)
#define OFFSET_CONTROL_OUT_SLALOM 0//大回りのオフセットでの壁制御(出る)
#define OFFSET_CONTROL_IN 0//大回りのオフセットでの壁制御(入り)
#define OFFSET_CONTROL_OUT 1//大回りのオフセットでの壁制御(出る)
#define OFFSET_CONTROL_IN_SLANT 0//3で制御あり
#define OFFSET_CONTROL_OUT_SLANT 3//3で制御あり

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
}DIJKSTRA;

typedef struct {
	float g_speed;
		float f_ofset;
		float e_ofset;
		float t_speed;
		float t_acc;
} parameter;
typedef struct {
	float velocity;
	float acceleration;
	float displacement;

}TARGET;

typedef struct{
	float displacement;
	float start_velocity;
	float end_velocity;
	float count_velocity;
	float acceleration;
	float deceleration;

}TRAPEZOID;

typedef struct{
	float displacement;
	float center_velocity;
	float max_turning_velocity;
}MOLLIFIER;

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
parameter_speed speed300_exploration;
TARGET straight;
TARGET turning;
TRAPEZOID Trapezoid_straight;
TRAPEZOID Trapezoid_turning;
MOLLIFIER Mollifier_turning;
char modeacc;
volatile char g_acc_flag;
volatile char g_MotorEnd_flag;
uint8_t g_FrontWallControl_mode;
float mollifier_timer;
char no_safty;
char highspeed_mode;

//入出力変数
STACK_T g_Goal_x;
STACK_T g_Goal_y;
char Dijkstra_maker_flag;
uint32_t walk_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM]; //歩数いれる箱
char error_mode;
DIJKSTRA Dijkstra;
uint16_t g_sensor_front,g_sensor_right,g_sensor_left;
int x,y,direction;
float straight_velocity_log[6000];
float turning_velocity_log[6000];
int log_num=0;

void initStack_walk(STACK_T *stack){
//	for(int i=0;i<=MAX_QUEUE_NUM-1;i++){
//		stack->data[i] = 0;
//	}
    /* スタックを空に設定 */
	stack->head = 0;
    stack->tail = 0;
}



void input_parameter(void) {

	speed300_exploration.SlalomCentervelocity = 300;
	speed300_exploration.TurnCentervelocity = 300;

	speed300_exploration.slalom_R.g_speed =
			speed300_exploration.SlalomCentervelocity;
	speed300_exploration.slalom_R.t_speed = 980; //550
	speed300_exploration.slalom_R.t_acc = 13000; //10000
	speed300_exploration.slalom_R.f_ofset = 3; //55;
	speed300_exploration.slalom_R.e_ofset = 24;

	speed300_exploration.slalom_L.g_speed =
			speed300_exploration.SlalomCentervelocity;
	speed300_exploration.slalom_L.t_speed = 980;
	speed300_exploration.slalom_L.t_acc = 13000;
	speed300_exploration.slalom_L.f_ofset = 2; //50;
	speed300_exploration.slalom_L.e_ofset = 24;
}


//  fake関数

void pl_yellow_LED_count(unsigned char yy){}
void pl_DriveMotor_standby(int pin){}
void pl_DriveMotor_start(void){}
void pl_DriveMotor_stop(void){}
void pl_L_DriveMotor_mode(int l_motor_mode){}
void pl_R_DriveMotor_mode(int r_motor_mode){}
void record_in(void) {}
void record_out(void) {}
void maze_display(void) {}
void maze_display_Dijkstra(void) {}
void clear_Ierror(void) {}
void reset_speed(void) {}
void reset_gyro(void) {}
void reset_distance(void) {}
void wait_ms_NoReset(uint32_t waitTime) {
    for(int i = 0;i < waitTime;i++){
    straight_velocity_log[log_num+i]=0; 
    turning_velocity_log[log_num+i]=0;
    }
    log_num=log_num + waitTime;
}

void cal_table(TRAPEZOID input,TARGET *target){
float time_over;
if (input.displacement>=0){
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
			if (target->velocity >= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}
			else if((input.displacement <= (2*target->velocity*target->velocity
					-input.start_velocity*input.start_velocity
					-input.end_velocity*input.end_velocity)
					/2/input.acceleration)){
				time_over=((2*target->velocity*target->velocity
						-input.start_velocity*input.start_velocity
						-input.end_velocity*input.end_velocity)
						/2/input.acceleration-input.displacement)/target->velocity;
				target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity -= input.acceleration*(2*time_over);

				target->acceleration = -input.acceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		if (input.displacement-target->displacement <=
				(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2/input.acceleration) {
			time_over=(target->displacement+(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2
						/input.acceleration-input.displacement)/target->velocity;
			target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity -= input.acceleration*(time_over);

			target->acceleration = -input.acceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity <= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		break;
	case 5:
		//加速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}
}else{
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
			if (target->velocity <= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}

			else if((-input.displacement <= (2*target->velocity*target->velocity
					-input.start_velocity*input.start_velocity
					-input.end_velocity*input.end_velocity)
					/2/input.acceleration)){
				time_over=(-(2*target->velocity*target->velocity
						-input.start_velocity*input.start_velocity
						-input.end_velocity*input.end_velocity)
						/2/input.acceleration-input.displacement)/target->velocity;
				target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity += input.acceleration*(2*time_over);

				target->acceleration = input.acceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		if (-input.displacement+target->displacement <=
				(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2/input.acceleration) {
			time_over=(target->displacement-(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2
						/input.acceleration-input.displacement)/target->velocity;
			target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity += input.acceleration*(time_over);

			target->acceleration = input.acceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity >= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		g_MotorEnd_flag=1;
		break;
	case 5:
		//加速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}

}

}

float cal_mollifier_velocity(float t_now,float mollifier_T,float integral){
	float velocity;
	velocity=(2/mollifier_T)*integral/MOLLIFIER_INTEGRAL*exp(-mollifier_T*mollifier_T/4/(mollifier_T*mollifier_T/4-t_now*t_now));
	return velocity;
}
float cal_mollifier_acceleration(float t_now,float mollifier_T,float integral){
	float acceleration;
	acceleration= integral/MOLLIFIER_INTEGRAL*(-mollifier_T*t_now/(mollifier_T*mollifier_T/4-t_now*t_now)/(mollifier_T*mollifier_T/4-t_now*t_now))*exp(-mollifier_T*mollifier_T/4/(mollifier_T*mollifier_T/4-t_now*t_now));
	return acceleration;
}


void cal_mollifier_table(MOLLIFIER input,TARGET *target){

float mollifier_T;
float old_velocity;
float time_delay=12;
float time_delay2=-10;
	mollifier_timer+=INTERRUPT_TIME;
		mollifier_T=2*fabs(input.displacement)/MOLLIFIER_INTEGRAL*exp(-1)/input.max_turning_velocity;
		if (mollifier_timer>-mollifier_T/2 && mollifier_timer<mollifier_T/2){
			old_velocity=target->velocity;
			target->velocity = cal_mollifier_velocity(mollifier_timer,mollifier_T,input.displacement);
			if(target->velocity >=1950){
				target->velocity=1950;
			}
			if(target->velocity <=-1950){
				target->velocity=-1950;
			}

			if(mollifier_timer<-mollifier_T/2/1.316+time_delay*INTERRUPT_TIME){
				target->acceleration = cal_mollifier_acceleration(-mollifier_T/2/1.316,mollifier_T,input.displacement);
			}else if(mollifier_timer<0){
				target->acceleration = cal_mollifier_acceleration(mollifier_timer-INTERRUPT_TIME*time_delay,mollifier_T,input.displacement);
			}else if(mollifier_timer<mollifier_T/2/1.316+time_delay2*INTERRUPT_TIME){
				target->acceleration = cal_mollifier_acceleration(mollifier_timer-INTERRUPT_TIME*time_delay,mollifier_T,input.displacement);
			}else if(mollifier_timer<mollifier_T/2+time_delay2*INTERRUPT_TIME){
				time_delay=0;
				target->acceleration = cal_mollifier_acceleration(mollifier_T/2/1.316,mollifier_T,input.displacement);
			}else{
				target->acceleration = cal_mollifier_acceleration(mollifier_T/2-INTERRUPT_TIME,mollifier_T,input.displacement);
			}
//			if(mollifier_timer>-mollifier_T/2*0.35 && mollifier_timer<mollifier_T/2*0.45){
//							target->acceleration = 0.7*target->acceleration;
//			}
//			if(mollifier_timer>mollifier_T/2*0.6){
//							target->acceleration = 0.4*target->acceleration;
//			}
//			if(mollifier_timer>mollifier_T/2*0.9){
//							target->acceleration = -0.6*target->acceleration;
//			}
		}else{
			old_velocity=target->velocity;
			target->velocity=0;
			target->acceleration = -target->velocity+old_velocity;
			g_acc_flag=4;

		}

}





void straight_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration,MOTOR_MODE motor_mode) {
	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	Trapezoid_straight.displacement = input_displacement;
	Trapezoid_straight.start_velocity = input_start_velocity;
	Trapezoid_straight.end_velocity = input_end_velocity;
	Trapezoid_straight.count_velocity = input_count_velocity;
	Trapezoid_straight.acceleration = input_acceleration;
    	if (input_count_velocity>=0){straight.acceleration = input_acceleration;
	}else{straight.acceleration = -input_acceleration;}

    straight.velocity = input_start_velocity;
	straight.displacement = 0;
	turning.velocity = 0;
	turning.acceleration = 0;
	turning.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	modeacc = 1;
	g_WallControl_mode=motor_mode.WallControlMode;
	pl_DriveMotor_start();

	while (g_acc_flag!=4){
        straight_velocity_log[log_num]=straight.velocity;
        turning_velocity_log[log_num]=turning.velocity;
        log_num++;
        straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table(Trapezoid_straight,&straight);

	}
    log_num--;

	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(100);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		pl_DriveMotor_stop();//これは必要か？
		wait_ms_NoReset(100);
	}



}

void End_straight(float input_displacement,MOTOR_MODE motor_mode,_Bool right_wall,_Bool left_wall){
	
}

float turning_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {
	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	Trapezoid_turning.displacement = input_displacement;
	Trapezoid_turning.start_velocity = input_start_velocity;
	Trapezoid_turning.end_velocity = input_end_velocity;
	Trapezoid_turning.count_velocity = input_count_velocity;
	Trapezoid_turning.acceleration = input_acceleration;

	if (input_count_velocity>=0){turning.acceleration = input_acceleration;
	}else{turning.acceleration = -input_acceleration;}
	turning.velocity = input_start_velocity;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;turning.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;turning.acceleration = input_acceleration;}
	modeacc = 2;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){
        straight_velocity_log[log_num]=straight.velocity;
        turning_velocity_log[log_num]=turning.velocity;
        log_num++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table(Trapezoid_turning,&turning);
	}
    log_num--;
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(300);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(100);
	}
//	modeacc = 0;

	pl_DriveMotor_stop();
}



float slalom_table2(float input_center_velocity,float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	Trapezoid_turning.displacement = input_displacement;
	Trapezoid_turning.start_velocity = input_start_velocity;
	Trapezoid_turning.end_velocity = input_end_velocity;
	Trapezoid_turning.count_velocity = input_count_velocity;
	Trapezoid_turning.acceleration = input_acceleration;

	if (input_count_velocity>=0){turning.acceleration = input_acceleration;
	}else{turning.acceleration = -input_acceleration;}
	turning.velocity = input_start_velocity;
	turning.displacement = 0;
	straight.velocity = input_center_velocity;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;turning.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;turning.acceleration = input_acceleration;}
	modeacc = 4;
//	enc.sigma_error=0;
	pl_DriveMotor_start();
	while (g_acc_flag!=4){
        straight_velocity_log[log_num]=straight.velocity;
        turning_velocity_log[log_num]=turning.velocity;
        log_num++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table(Trapezoid_turning,&turning);
	}
    log_num--;
	pl_DriveMotor_stop();

}

void mollifier_slalom_table(float input_center_velocity,float input_displacement, float input_max_turning_velocity) {

	// 例外処理

	Mollifier_turning.center_velocity = input_center_velocity;
	Mollifier_turning.displacement = input_displacement;
	Mollifier_turning.max_turning_velocity = input_max_turning_velocity;


	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = input_center_velocity;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
	mollifier_timer=-fabs(input_displacement)/MOLLIFIER_INTEGRAL*exp(-1)/input_max_turning_velocity;
	modeacc = 6;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){
                straight_velocity_log[log_num]=straight.velocity;
        turning_velocity_log[log_num]=turning.velocity;
         log_num++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME;// + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		cal_mollifier_table(Mollifier_turning,&turning);//角速度と角加速度はここで決定
	}
    log_num--;
//	modeacc = 0;



	pl_DriveMotor_stop();

}


void no_frontwall_straight(void){
	turning.acceleration = 0;
	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;

	g_FrontWallControl_mode=1;
	modeacc = 5;

	pl_DriveMotor_start();
	wait_ms_NoReset(150);

	g_FrontWallControl_mode=0;
	modeacc = 0;

	pl_DriveMotor_stop();

}




void backTurn_controlWall(float input_TurningVelocity,float input_TurningAcceleration,_Bool front_wall,_Bool left_wall,_Bool right_wall){
no_safty = 1;
	if(front_wall){
		no_frontwall_straight();
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		//clear_Ierror();
		wait_ms_NoReset(50);
	}
	if(left_wall){
		turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		no_frontwall_straight();
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		//clear_Ierror();
		wait_ms_NoReset(50);
		turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
	}else if(left_wall==0 && right_wall){
		turning_table2(-90,0,0,-input_TurningVelocity,input_TurningAcceleration);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		no_frontwall_straight();
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		//clear_Ierror();
		wait_ms_NoReset(50);
		turning_table2(-90,0,0,-input_TurningVelocity,input_TurningAcceleration);
	}else if(left_wall==0 && right_wall==0){
		turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
	}
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(150);
	no_safty = 0;
}


void slalomR(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {
MOTOR_MODE wallmode;
	if (test_mode == ON) {
		highspeed_mode = 0;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(BACK_TO_CENTER + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		wallmode.WallCutMode=1;
		wallmode.WallControlMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLALOM;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=1;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLALOM;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);
//		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
//										fabs(end_velocity*end_velocity-turnpara.g_speed * turnpara.g_speed)  / 2 / turnpara.e_ofset,wallmode);
	}
}

void slalomL(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == ON) {
		highspeed_mode = 0;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(BACK_TO_CENTER + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLALOM;
		wallmode.WallCutMode=1;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLALOM;
		wallmode.WallCutMode=0;
		straight_table2(45 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=1;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
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
	uint16_t count_number = 1;
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
	uint16_t count_number = 1;
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
	uint16_t count_number = 1;
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
			if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
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
					if(Xcoordinate <= MAZE_SQUARE_NUM-3){
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
					if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
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





void AdatiWayReturn(float input_StraightVelocity, float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,int know_mode,uint8_t Dijkstra_mode) {



	//初期化
	maze_mode = 1; //迷路探索開始フラグ
	unsigned short front_count, right_count, back_count, left_count;
	//int x=0;//構造体にしたい
	//int y=0;
	//int direction=1;
	_Bool front_wall,right_wall,left_wall;
	char timer_end_mode=0;
	int kitiku_distance;
	MOTOR_MODE mode;
	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
    
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
			run_movement_suspension(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall, x, y, 1, 1);
			if (direction >= 5) {direction = direction-4;}
			if (direction <= 0) {direction = direction+4;}
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
        if(1){break;}

	}




}







/* the gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  const mwSize dims0[]={MAZE_SQUARE_NUM-1,1};
  const mwSize dims1[]={3,1};
  const mwSize dims2[]={MAZE_SQUARE_NUM,MAZE_SQUARE_NUM};
  const mwSize dims3[]={MAZE_SQUARE_NUM-1,MAZE_SQUARE_NUM};
  const mwSize dims4[]={2,1};
  unsigned char *start_of_pr;
  unsigned short data[]={1,2,3,4};
  int coordinate[3];
  char all_mode[2];
  size_t bytes_to_copy;
    double *outMatrix1;              /* output matrix */
 double *outMatrix2;  


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
    log_num=0;
    input_parameter();
    AdatiWayReturn(300,400,2000,3000,speed300_exploration,1,1);
 


  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[0] = mxCreateNumericArray(2,dims0,mxUINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[0]);
  bytes_to_copy = TOTAL_ELEMENTS0 * mxGetElementSize(plhs[0]);
  memcpy(start_of_pr,wall.row,bytes_to_copy);

      /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[1] = mxCreateNumericArray(2,dims0,mxUINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[1]);
  bytes_to_copy = TOTAL_ELEMENTS0 * mxGetElementSize(plhs[1]);
  memcpy(start_of_pr,wall.column,bytes_to_copy);

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[2] = mxCreateNumericArray(2,dims0,mxUINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[2]);
  bytes_to_copy = TOTAL_ELEMENTS0 * mxGetElementSize(plhs[2]);
  memcpy(start_of_pr,wall.row_look,bytes_to_copy);

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[3] = mxCreateNumericArray(2,dims0,mxUINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[3]);
  bytes_to_copy = TOTAL_ELEMENTS0 * mxGetElementSize(plhs[3]);
  memcpy(start_of_pr,wall.column_look,bytes_to_copy);

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[4] = mxCreateNumericArray(2,dims1,mxINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[4]);
  bytes_to_copy = TOTAL_ELEMENTS1 * mxGetElementSize(plhs[4]);
  coordinate[0]=x;
  coordinate[1]=y;
  coordinate[2]=direction;
  memcpy(start_of_pr,coordinate,bytes_to_copy);

   /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[5] = mxCreateNumericArray(2,dims2,mxUINT32_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[5]);
  bytes_to_copy = TOTAL_ELEMENTS2 * mxGetElementSize(plhs[5]);
  memcpy(start_of_pr,walk_count,bytes_to_copy);  

   /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[6] = mxCreateNumericArray(2,dims3,mxUINT16_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[6]);
  bytes_to_copy = TOTAL_ELEMENTS3 * mxGetElementSize(plhs[6]);
  memcpy(start_of_pr,Dijkstra.row_count,bytes_to_copy);  

   /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[7] = mxCreateNumericArray(2,dims3,mxUINT16_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[7]);
  bytes_to_copy = TOTAL_ELEMENTS3 * mxGetElementSize(plhs[7]);
  memcpy(start_of_pr,Dijkstra.column_count,bytes_to_copy);  

   /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[8] = mxCreateNumericArray(2,dims4,mxINT8_CLASS,mxREAL);
  start_of_pr = (unsigned char *)mxGetData(plhs[8]);
  bytes_to_copy = TOTAL_ELEMENTS4 * mxGetElementSize(plhs[8]);
    all_mode[0]=Dijkstra_maker_flag;
    all_mode[1]=error_mode;
  memcpy(start_of_pr,all_mode,bytes_to_copy);  

       /* create a 2-by-2 array of unsigned 16-bit integers */
    plhs[9] = mxCreateDoubleMatrix(1,(mwSize)(log_num+1),mxREAL);
    outMatrix1 = mxGetPr(plhs[9]);
    straight_velocity_log[log_num]=(float)(log_num)*INTERRUPT_TIME;
    for (int index = 0; index < log_num+1; index++) {
        outMatrix1[index] = straight_velocity_log[index];
    }

           /* create a 2-by-2 array of unsigned 16-bit integers */
    plhs[10] = mxCreateDoubleMatrix(1,(mwSize)(log_num+1),mxREAL);
    outMatrix2 = mxGetPr(plhs[10]);
    turning_velocity_log[log_num]=(float)(log_num)*INTERRUPT_TIME;
    for (int index = 0; index < log_num+1; index++) {
        outMatrix2[index] = turning_velocity_log[index];
    }

//   plhs[10] = mxCreateNumericArray(1,(mwSize)1,mxINT32_CLASS,mxREAL);
//   start_of_pr = (unsigned char *)mxGetData(plhs[10]);
//   bytes_to_copy = mxGetElementSize(plhs[10]);
//   memcpy(start_of_pr,log_num,bytes_to_copy); 
//   plhs[9] = mxCreateNumericArray(2,dims4,mxINT8_CLASS,mxREAL);
//   start_of_pr = (unsigned char *)mxGetData(plhs[9]);
//   bytes_to_copy = TOTAL_ELEMENTS4 * mxGetElementSize(plhs[9]);
//     all_mode[0]=Dijkstra_maker_flag;
//     all_mode[1]=error_mode;
//   memcpy(start_of_pr,all_mode,bytes_to_copy);  


}
