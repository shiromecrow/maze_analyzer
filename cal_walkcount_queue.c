#include <string.h> /* needed for memcpy() */
#include <stdint.h> /* needed for memcpy() */
#include "mex.h"

/*
 * cal_walkcount_queue.c - example found in API guide
 *
 * constructs a 2-by-2 matrix with unsigned 16-bit integers, doubles
 * each element, and returns the matrix
 *
 * This is a MEX-file for MATLAB.
 * Copyright 1984-2007 The MathWorks, Inc.
 */


#define NDIMS 2
#define TOTAL_ELEMENTS 256

#define MAX_QUEUE_NUM 1000
#define ROW 0
#define COLUMN 1
#define MAX_WALKCOUNT 255
#define MAX_WALKCOUNT_DIJKSTRA 65535

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


uint32_t walk_count[16][16]; //歩数いれる箱
WALL wall;
uint32_t GOAL_X,GOAL_Y;


void initStack_walk(STACK_T *stack){
//	for(int i=0;i<=MAX_QUEUE_NUM-1;i++){
//		stack->data[i] = 0;
//	}
    /* スタックを空に設定 */
	stack->head = 0;
    stack->tail = 0;
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
  /* call the computational subroutine */
    create_StepCountMap_queue();

  /* create a 2-by-2 array of unsigned 16-bit integers */
  plhs[0] = mxCreateNumericArray(NDIMS,dims,mxUINT32_CLASS,mxREAL);

  /* populate the real part of the created array */
  start_of_pr = (unsigned char *)mxGetData(plhs[0]);
  bytes_to_copy = TOTAL_ELEMENTS * mxGetElementSize(plhs[0]);
  memcpy(start_of_pr,walk_count,bytes_to_copy);
}
