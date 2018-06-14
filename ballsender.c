/*
-----------------------------------------------------------------------------
-- Copyright (C) 2005 IMEC                                                  -
--                                                                          -
-- Redistribution and use in source and binary forms, with or without       -
-- modification, are permitted provided that the following conditions       -
-- are met:                                                                 -
--                                                                          -
-- 1. Redistributions of source code must retain the above copyright        -
--    notice, this list of conditions and the following disclaimer.         -
--                                                                          -
-- 2. Redistributions in binary form must reproduce the above               -
--    copyright notice, this list of conditions and the following           -
--    disclaimer in the documentation and/or other materials provided       -
--    with the distribution.                                                -
--                                                                          -
-- 3. Neither the name of the author nor the names of contributors          -
--    may be used to endorse or promote products derived from this          -
--    software without specific prior written permission.                   -
--                                                                          -
-- THIS CODE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS''           -
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED        -
-- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A          -
-- PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR       -
-- CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,             -
-- SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         -
-- LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF         -
-- USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      -
-- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,       -
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT       -
-- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF       -
-- SUCH DAMAGE.                                                             -
--                                                                          -
-----------------------------------------------------------------------------
-----------------------------------------------------------------------------
-- File           :
-----------------------------------------------------------------------------
-- Description    : C code
-- --------------------------------------------------------------------------
-- Author         :
-- Date           :
-- Change history :
// to do :
// 			- implement starting coordination +
*			- more timing to increase game performance
* 			- inter-processors mailbox communication
* 			- starting sync
* 			- fix "chipoff" problem at top/btm edges
* 			- check if bar speed/ball speed are as intended
* 			- if brickthreads need to be 10 separate threads
* 			- reset game functionality
* 			- pre-compute ball tracing
* 			- ball stuck in horizontal motion
* 			- possible enhancements (high score/ save+load game)
*
//
//
// to send to microblaze1 together
// 1) bar position
// 2) ball speedup (as triggered every 10 points)
//
// to receive from microblaze1
// 1) ball x,y coordinates
// 2) ball speed
// 3) brick column - send col = 0 if no collision, 1-8 if collision, all other values will crash program
// 4) brick status - the brickcol's status/ or brick row
//
// completed :
*
* - pause/resume
* - win and lose conditions
* - prevent "chipoff" on left/right edges of gamearea
* - coloured sections for bar
* - make red colums change randomly
* - ball logic/ reduce ball passing through bricks
-----------------------------------------------------------------------------
*/
#include "xmk.h"
#include "xmutex.h"
#include "xmbox.h"
#include "xstatus.h"
#include "semaphore.h"
#include "xparameters.h"
#include <pthread.h>
#include <errno.h>
#include <sys/msg.h>
#include <sys/init.h>
#include <sys/ipc.h>
#include <sys/timer.h>
#include <sys/intr.h> //xilkernel api for interrupts
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include "xuartps.h" //replace xuart_lite.h


/************************** Constant Definitions ****************************/
/**
* The following constants map to the XPAR parameters created in the
* xparameters.h file. They are defined here such that a user can easily
* change all the needed parameters in one place.
*/
#define XST_SUCCESS 	0L
#define XST_FAILURE 	1L

//gamearea
#define GAMEAREA_LEFT 60
#define GAMEAREA_TOP 60
#define GAMEAREA_RIGHT 514
#define GAMEAREA_BTM 419

//gamestate
#define GAME_NORMAL 0
#define GAME_WIN 1
#define GAME_LOSE 2
#define GAME_PAUSE 3
#define GAME_RESET 4

//message for
#define MESSAGE_FOR_BALL 0
#define MESSAGE_FOR_GAME 1

//bar
#define BAR_TOP 405
#define INITIAL_BAR 288

//ball macros
#define CIRCLE_RADIUS 7
#define PI 3.14159265
#define INITIAL_X INITIAL_BAR
#define INITIAL_Y BAR_TOP - CIRCLE_RADIUS
#define INCREMENT_ONE_VALUE 1
#define MAX_BALLSPEED 20
#define MIN_BALLSPEED 2
#define	INITIAL_BALLANGLE 90
#define	INITIAL_BALLSPEED 10

//brick
#define TOTAL_COLUMNS 10
#define TOTAL_ROWS 8
#define INTERBRICK_Y 20
#define INTERBRICK_X 45
#define BRICK_LENGTH 40 //not used by ball
#define BRICK_HEIGHT 15 //not used by ball

//collision
#define BOTTOM_HIT 20
#define TOP_HIT 21
#define LEFT_HIT 22
#define RIGHT_HIT 23
#define BAR_HIT 24


/**
* User has to specify a 2MB memory space for filling the frame data.
* This constant has to be updated based on the memory map of the
* system.
*/
#define TFT_FRAME_ADDR        0x10000000

/**************************** Type Definitions ******************************/


typedef struct {
	int message_for;
	int ballx;
	int bally;
	int brickrow;
	int brickcol;
	int speed;
} msg_ball;

typedef struct {
	int message_for;
	int bar_position;
	int score;
	int game_status;
	int poweruphold;
	int poweruplengthen;
	int ballheldx;
} msg_game;	//debug - try removing the dummy data if mailbox is always in order

/************************** Function Prototypes *****************************/
//threads

//ball
void* thread_func_1 ();
void* thread_reset();


//generic function declarations
int main_prog(void);
void init_variables();
void init_threads();
void resetHandler();

//ball functions

void determine_new_circle_coordinates();
void increment_by_one(int * , int * );
int check_collision( int * xcoordinate, int * ycoordinate);
void mini_ray_trace();
void recalculate_angle();
void set_brick_collision_protocol(int ROW_ID, int COL_ID);
int check_collision_bar(int cursor_bar_local, int * xcoordinate, int * ycoordinate);

/************************** Variable Definitions ****************************/

// mailbox declaration
//#define MY_CPU_ID 1
#define MY_CPU_ID XPAR_CPU_ID
#define MBOX_DEVICE_ID		XPAR_MBOX_0_DEVICE_ID
static XMbox Mbox;	/* Instance of the Mailbox driver */

//HW Mutex
#define MUTEX_DEVICE_ID XPAR_MUTEX_0_IF_0_DEVICE_ID
#define MUTEX_NUM 0
XMutex mutex;


// software mutex declaration
pthread_mutex_t uart_mutex;
pthread_mutex_t flagset_mutex; //not sure if used
pthread_mutex_t brickcollisionprotocol_mutex;

// threads declaration
//ball threads
pthread_t tid1, treset;


//global variables
// to be received from display
signed int cursor_curr;			// absolute position
signed int game_score;
int gamestateflag;

//ball
//global variables
int global_x;
int global_y; // ball initialized to be in the center
float angle; //angle range is from 0 to 360 degrees, initialized as 60
int SPEED ; //desired pixel speed
int INCREMENT;
int ID;
int count;
int angle_origin_x;
int angle_origin_y;
int set_collision_x;
int set_collision_y;
//previous static variables declared in determine_new_coordinates
int xdir;
int ydir;
int xdirection;
int ydirection;
int collision_detect; //initialized to 0
int post_collision; //initialized to 1

int x_offset;
int y_offset;
int brickflags[TOTAL_ROWS][TOTAL_COLUMNS] ;
int collision_brick_pending_status;
int collided_brick_row;
int collided_brick_col;
int communicate_collidedbrick_row;
int communicate_collidedbrick_col;

int send_packet_no;

int poweruplengthen;
//
//	Thread functions
//


// ball threads


void* thread_func_1 () {
	msg_ball msg_ball_tosend;
	msg_game msg_game_rcd;
	int score_nextlevel = 10;
	int poweruphold = 0;
	//int poweruplengthen = 0;

	int ballthread_timestamp;
	int ballthread_timestamp2;
	int ballthread_timeinterval;
	int ballthread_finishtimestamp;
	int ballthread_finishinterval;

	while (1) {

		ballthread_timestamp= xget_clock_ticks();
		//gamestateflag can only be GAME_NORMAL here
		// if not will not reach here

		//ensure that message from GAME is received before proceeding
		//while(gamemessage_received!=1);
		//gamemessage_received = 0;

		//Increment SPEED according to game score as per needed
		if (game_score >= score_nextlevel)
		{
			// set next level benchmark
			score_nextlevel += 10;

			// increase ball speed by telling ball thread [debug]
			// increase global variable of ballspeed
			if (SPEED + 1 <= MAX_BALLSPEED)
			SPEED += 1;
			else
			SPEED = MAX_BALLSPEED;

		}

		determine_new_circle_coordinates();


		//don't need mutex for following sections strictly speaking
		msg_ball_tosend.message_for = MESSAGE_FOR_GAME;
		msg_ball_tosend.ballx = global_x;
		msg_ball_tosend.bally = global_y;
		msg_ball_tosend.brickrow = communicate_collidedbrick_row;
		msg_ball_tosend.brickcol = communicate_collidedbrick_col;
		msg_ball_tosend.speed = SPEED;


		XMbox_WriteBlocking(&Mbox, &msg_ball_tosend, sizeof(msg_ball));
		//XMutex_Lock(&mutex, MUTEX_NUM);
		//xil_printf("-- Sucessfully send from BALL to GAME --\r\n");
		//XMutex_Unlock(&mutex, MUTEX_NUM);

		/*
		ballthread_timestamp2 = xget_clock_ticks();
		ballthread_timeinterval = ballthread_timestamp2 - ballthread_timestamp;
		pthread_mutex_lock(&uart_mutex);
		//xil_printf("new ball packet no. %d sent %d * 10 milliseconds after previous send \n\n", send_packet_no, ballthread_timeinterval);
		pthread_mutex_unlock(&uart_mutex);

		ballthread_timestamp = xget_clock_ticks();
		send_packet_no += 1;
		*/

		if (communicate_collidedbrick_row == BOTTOM_HIT)
		{
			//game already knows game lose from the mail
			gamestateflag = GAME_LOSE;
		}


	//sleep(10);	//is this sleep needed [debug]

	do{
		// GAME_LOSE + GAME_WIN will block at read
		// GAME_PAUSE wiil take in msg and blocked at read
		XMbox_ReadBlocking(&Mbox, &msg_game_rcd, sizeof(msg_game));

		  if (poweruphold==1 && communicate_collidedbrick_row== BAR_HIT)
		  {
			  global_x = msg_game_rcd.ballheldx;
			  angle_origin_x = global_x;
		  }
		//XMutex_Lock(&mutex, MUTEX_NUM);
		//xil_printf("-- Sucessfully received in BALL from GAME --\r\n");
		//XMutex_Unlock(&mutex, MUTEX_NUM);

		//debug
		if (msg_game_rcd.message_for == MESSAGE_FOR_GAME)
		{
			XMutex_Lock(&mutex, MUTEX_NUM);
			xil_printf("-- Error - BALL received message meant for GAME!! --\r\n");
			XMutex_Unlock(&mutex, MUTEX_NUM);
			while(1); //stall here
		}
		//might need mutex protection
		cursor_curr = msg_game_rcd.bar_position;
		game_score = msg_game_rcd.score;
		gamestateflag = msg_game_rcd.game_status;
		poweruphold = msg_game_rcd.poweruphold;
		poweruplengthen = msg_game_rcd.poweruplengthen;

	}while (gamestateflag == GAME_PAUSE);

	if(gamestateflag == GAME_RESET)
	{
		resetHandler();
		pthread_mutex_lock(&uart_mutex);
		xil_printf("func 1 exiting..\n\n");
		pthread_mutex_unlock(&uart_mutex);
		pthread_exit(0);

	}
	// only will go into next iteration if gamestateflag == GAME_NORMAL

	//reset communicate_collidedbrick_row and communicate_collidedbrick_col if indicate brick collision
	// debug maybe dont need the if condition and always set the value to 0 here
	if (communicate_collidedbrick_row !=0 || communicate_collidedbrick_col != 0)
	{
		communicate_collidedbrick_row = 0;
		communicate_collidedbrick_col = 0;
	}

	ballthread_timestamp2 = xget_clock_ticks();
	ballthread_timeinterval = ballthread_timestamp2 - ballthread_timestamp;
	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("time taken for iteration of ballthread to complete is %d*10 milliseconds \n\n", ballthread_timeinterval);
	//pthread_mutex_unlock(&uart_mutex);
    if (ballthread_timeinterval == 0)
    	sleep(40);
    else if (ballthread_timeinterval==1)
    	sleep(30);
    else if (ballthread_timeinterval == 2)
		sleep(20);
	else if (ballthread_timeinterval==3)
		sleep(10);
	else if (ballthread_timeinterval >=4)
    //do nothing
	sleep(1);
	//sleep(30);//is this sleep needed [debug]
}
}



void* thread_reset()
{
	//attempts to lock, then unlock all mutex
	pthread_mutex_trylock(&uart_mutex);
	pthread_mutex_unlock(&uart_mutex);

	pthread_mutex_trylock(&flagset_mutex);
	pthread_mutex_unlock(&flagset_mutex);

	pthread_mutex_trylock(&brickcollisionprotocol_mutex);
	pthread_mutex_unlock(&brickcollisionprotocol_mutex);

	//reset hwmutex deubg

	pthread_mutex_lock(&uart_mutex);
	xil_printf("mutex reset, reset thread1 sleeping...\n\n");
	pthread_mutex_unlock(&uart_mutex);

	sleep(40); //buffer time to let other threads die

	init_variables();
	init_threads();

	pthread_mutex_lock(&uart_mutex);
	xil_printf("thread reset1 about exiting..\n\n");
	pthread_mutex_unlock(&uart_mutex);

	pthread_exit(0);
}

int main(void) {
	print("-- Entering main() uB0 RECEIVER--\r\n");
	xilkernel_init();
	xmk_add_static_thread(main_prog, 0);
	xilkernel_start();
	//Start Xilkernel
	xilkernel_main();

	//Control does not reach here
	return 0;
}

int main_prog(void) { // This thread is statically created (as configured in the kernel configuration) and has priority 0 (This is the highest possible)

	int ret;
	XMbox_Config *ConfigPtr;
	int Status;
	XMutex_Config *mutex_cfg;

	print("-- Entering main_prog() uB0 SENDER--\r\n");

	// initialize the SW Mutex here
	//display
	ret = pthread_mutex_init(&uart_mutex, NULL);
	//if (ret != 0)
	//xil_printf("-- ERROR (%d) init uart_mutex...\r\n", ret);
	// initialize Mutex


	//Initialize HW Mutex

	mutex_cfg = XMutex_LookupConfig(MUTEX_DEVICE_ID);
	XMutex_CfgInitialize(&mutex, mutex_cfg, mutex_cfg->BaseAddress);

	// CONFIGURE THE MAILBOX HERE

	ConfigPtr = XMbox_LookupConfig(MBOX_DEVICE_ID );
	if (ConfigPtr == (XMbox_Config *)NULL) {
		print("-- Error configuring Mbox uB1 Sender--\r\n");
		return NULL;
	}

	// Perform the rest of the initialization.
	Status = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		print("-- Error initializing Mbox uB1 Sender--\r\n");
		return NULL;
	}


	//ball
	ret = pthread_mutex_init (&flagset_mutex, NULL);
	if (ret != 0)
	{
		//xil_printf ("-- ERROR (%d) init flagset_mutex...\r\n", ret);
	}
	ret = pthread_mutex_init (&brickcollisionprotocol_mutex, NULL);
	if (ret != 0)
	{
		//xil_printf ("-- ERROR (%d) init brickcollisionprotocol_mutex...\r\n", ret);
	}


	init_variables();
	init_threads();
	return NULL;

}


void init_variables()
{
	int i,j;

	//display
	cursor_curr = INITIAL_BAR;
	game_score = 0;
	gamestateflag = GAME_NORMAL;


	//ball
	global_x = INITIAL_X;
	global_y = INITIAL_Y; // ball initialized to be in the center
	angle = INITIAL_BALLANGLE; //angle range is from 0 to 360 degrees, initialized as 90
	SPEED = INITIAL_BALLSPEED; //desired pixel speed

	INCREMENT = SPEED; //check
	ID = 0;
	count = 0;
	angle_origin_x = INITIAL_X;
	angle_origin_y = INITIAL_Y;
	set_collision_x = 0;
	set_collision_y = 0;
	//previous static variables declared in determine_new_coordinates
	xdir = 0;
	ydir = 0;
	xdirection=0;
	ydirection=0;
	collision_detect = 0; //initialized to 0
	post_collision= 1; //initialized to 1


	x_offset = 65;
	y_offset = 65;
	collision_brick_pending_status = 0;
	collided_brick_row = 0;
	collided_brick_col = 0;
	communicate_collidedbrick_row = 0;
	communicate_collidedbrick_col = 0;

	send_packet_no = 1;
	poweruplengthen = 0;

	//set bricks for ball code
	//Setting brickflags
	//ball
	for (i=0; i<TOTAL_ROWS; i++)
	{
		for (j=0; j<TOTAL_COLUMNS; j++)
		brickflags[i][j] = 1;
	}

	pthread_mutex_lock(&uart_mutex);
	xil_printf(" Finished init_variables.\r\n");
	pthread_mutex_unlock(&uart_mutex);
}

void init_threads()
{
	int ret;
	// thread launches


	//ball
	//start thread 1
	ret = pthread_create (&tid1, NULL, (void*)thread_func_1, NULL);
	if (ret != 0)
	{
		xil_printf ("-- ERROR (%d) launching thread_func_1...\r\n", ret);
	}
	else
	{
		xil_printf ("thread_func_1 launched with ID %d \r\n",tid1);
	}


pthread_mutex_lock(&uart_mutex);
xil_printf(" Finished init_threads.\r\n");
pthread_mutex_unlock(&uart_mutex);
}

void resetHandler()
{
	int ret;

	//launches thread_reset which will carry out the reset operation
	ret = pthread_create(&treset, NULL, (void*) thread_reset, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_reset...\r\n", ret);
	}
	else
	{
		xil_printf("thread_reset launched with ID %d \r\n", treset);
	}
}



//generic functions: ball


void determine_new_circle_coordinates()
{
	int i;
	/* Step 1: Assuming the ball has just collided, determine the INCREMENT value to be used to maintain ball speed*/

	//special case when there is no horizontal movement
	if (post_collision ==1)
	{
		post_collision = 0;
		if (angle==90 || angle==270)
		{
			INCREMENT = (int) SPEED;
			//xil_printf("adjusted increment value is %d\n", INCREMENT);
		}

		else
		{

			INCREMENT = (int) (SPEED * fabs(cos(angle*(PI/180))));
			//xil_printf("adjusted increment value is %d\n", INCREMENT);
		}
	}
	/* Step 2: Check by incrementing by one pixel at a time if there is a collision*/
    if (INCREMENT==0)
    	INCREMENT =1;

	for (i=1; i<= INCREMENT; i++)
	{
		//separate mini-ray trace function call
		mini_ray_trace();
		if (collision_detect)
			break;

	}

	/* if no collision_detect update the position of the ball by an INCREMENT value,
	if collision detect- set the co-ordinates of the circle to the collision point, recalculate angle based on collision ID, reset all required variables*/

	if (collision_detect!=1)
	{
		/*
		value of angle affects exactly what sort of increment is done
		*/

		//xil_printf("Entering function section to change ball position\n");
		float output;
		//if angle is known, determine new co-ordinate set
		if (angle < 90 || angle > 270)
		{
			xdir = xdir + INCREMENT;
			output = tan(angle*(PI/180));
			ydir = (int) (output* xdir) ;
		}

		if (angle > 90 && angle < 270)
		{
			xdir = xdir- INCREMENT;
			output = tan(angle*(PI/180));
			ydir = (int) (output* xdir) ;
		}

		if (angle==90)
		{
			xdir = xdir;
			ydir = ydir+ INCREMENT;
		}

		if (angle==270)
		{
			xdir = xdir;
			ydir = ydir-INCREMENT;
		}
		//xil_printf("values of xdir and ydir after increment is %d and %d\n", xdir,ydir);

		// updating co-ordinates of circle
		global_x = angle_origin_x + xdir;
		global_y = angle_origin_y - ydir;
		//xil_printf ("co-ordinates of ball after moving is x = %d and y = %d\n", global_x, global_y);

	}

	else if (collision_detect == 1)
	{
		//reset collision_detect
		collision_detect = 0;

		//xil_printf("Collision\n\n");
		global_x = set_collision_x;
		global_y = set_collision_y;

		//xil_printf ("co-ordinates of ball at collision is x = %d and y = %d\n", global_x, global_y);

		// any remaining need for mutexes?

		//pthread_mutex_lock(&brickcollisionprotocol_mutex);
		if (collision_brick_pending_status==1)
		{
			if (collided_brick_row-1 >=0 && collided_brick_row-1 <8)
				brickflags[collided_brick_row-1][collided_brick_col-1] = 0;

			collision_brick_pending_status=0;
			communicate_collidedbrick_row = collided_brick_row;
			communicate_collidedbrick_col = collided_brick_col;

		}
		else
		{
			communicate_collidedbrick_row = 0;
			communicate_collidedbrick_col = 0;
		}
		//pthread_mutex_unlock(&brickcollisionprotocol_mutex); //might be unnecessary


		//xil_printf("values of angle_origin variables x and y before reset are %d and %d\n", angle_origin_x, angle_origin_y);
		//Reset angle_origin variables
		angle_origin_x = global_x;
		angle_origin_y = global_y;
		//xil_printf("values of angle_origin variables x and y after reset are %d and %d\n", angle_origin_x, angle_origin_y);
		//Reset xdir, ydir variables
		xdir = 0;
		ydir = 0;
		//xil_printf("values of xdir and ydir after collision is %d and %d\n", xdir,ydir);
		//Reset collision variables - perhaps not required

		//xil_printf("Value of collision ID is %d\n", ID);

		//Calculate new angle using ID
		recalculate_angle();

		//int angle_int = (int) angle;
		//xil_printf("Value of recalculated angle after collision is %d\n", angle_int);
		//Reset ID to 0
		ID = 0;

		//xil_printf("value of ray_trace_flag before reset is %d\n", ray_trace_done_flag);
		//need to reset ray_trace_done flag

		//need to set post_collision flag to recalculate INCREMENT post collision
		post_collision= 1;
	}

}

void mini_ray_trace()
{
	//static int xdirection=0;
	//static int ydirection=0;
	static int collision_check = 0;



	int xcoordinate, ycoordinate;
	int * xdir_ptr;
	int * ydir_ptr;
	xdir_ptr = &xdirection;
	ydir_ptr = &ydirection;

	increment_by_one(xdir_ptr, ydir_ptr);
	//xil_printf("Value of xdirection, ydirection after incrementing function is %d , %d\n", xdirection, ydirection);
	xcoordinate = angle_origin_x + xdirection;
	ycoordinate = angle_origin_y - ydirection; // if y incremented is positive, y co-ordinate on screen must be decremented

    //Check bar collision first
	if (ycoordinate + CIRCLE_RADIUS >= BAR_TOP)
	{
		collision_check = check_collision_bar(cursor_curr, &xcoordinate, &ycoordinate);
	}

	if (collision_check != 1)
	collision_check = check_collision(&xcoordinate, &ycoordinate);

	if(collision_check==1)
	{
		//reset collision_check flag
		collision_check = 0;
		//set collision coordinate information
		set_collision_x = xcoordinate;
		set_collision_y = ycoordinate;

		//Reset xdirection and ydirection
		xdirection=0;
		ydirection=0;

		// set collision detect flag
		collision_detect = 1; //initialized to 0
	}
}


void recalculate_angle ()
{
	float interim_angle;
	float incidence_angle;

	switch(ID)
	{

		case 1: //Condition 1: check for collision against left side of screen i.e. x<=0/ right side of a brick

		//check for ball traveling north-west
		if (angle > 90 && angle < 180)
		{
			interim_angle = angle + 180;
			incidence_angle = 360-interim_angle;
			interim_angle = (float) fmod((interim_angle + 2*incidence_angle), 360);
			angle = interim_angle;
		}

		//check for ball traveling south west
		else if (angle > 180 && angle < 270)
		{
			incidence_angle = (float) fmod((angle + 180),360) ;
			interim_angle = 360 - incidence_angle;
			angle = interim_angle;
		}
		//check for ball traveling west
		else if (angle == 180)
		{
			angle = 360;
		}
		break;

		case 2:    //Condition 2: check for collision against right side of screen i.e. x>=639/ left side of a brick

		//check for ball traveling north-east
		if (angle < 90 && angle > 0)
		{
			interim_angle = angle + 180;
			incidence_angle = interim_angle - 180;
			interim_angle = interim_angle - 2*(incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling south east
		else if (angle < 360 && angle > 270)
		{
			interim_angle = (float) fmod((angle + 180),360) ;
			incidence_angle = 180 - interim_angle;
			interim_angle = interim_angle + (2*incidence_angle);
			angle = interim_angle;
		}

		//check for ball traveling east
		else if (angle == 360 || angle ==0 )
		{
			angle = 180;
		}

		break;

		case 3: 	//Condition 3: check for collision against top side of screen i.e. y<=0/ bottom side of a brick

		//check for ball traveling north east
		if (angle < 90 && angle > 0)
		{
			interim_angle =  (angle + 180) ;
			incidence_angle = 270 - interim_angle;
			interim_angle = interim_angle + (2*incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling north-west
		else if (angle > 90 && angle < 180)
		{
			interim_angle = angle + 180;
			incidence_angle = interim_angle - 270;
			interim_angle = interim_angle - 2*(incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling north
		else if (angle == 90)
		{
			angle = 270;
		}

		break;

		case 4:    //Condition 4: check for collision against bottom side of screen i.e. y>=479/ top of a particular brick

		//check for ball traveling south-east
		if (angle > 270 && angle <360)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = interim_angle - 90;
			interim_angle = interim_angle - 2*(incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling south west
		else if (angle > 180 && angle < 270)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = 90 - interim_angle;
			interim_angle = interim_angle + 2*(incidence_angle);
			angle = interim_angle;
		}

		//check for ball traveling south
		else if (angle == 270)
		{
			angle = 90;
		}
		break;

		case 5:    //Condition 5: collision against south east corner of a brick
		// ball can be traveling north east, south west, west and north, northwest

		//check for ball traveling north-east and north
		if (angle <= 90 && angle > 0)
		{
			interim_angle = (angle + 180);
			incidence_angle = 315 - interim_angle;
			interim_angle = (float) fmod(interim_angle + 2*(incidence_angle), 360);
			angle = interim_angle;
		}
		//check for ball traveling south west and west
		else if (angle >= 180 && angle < 270)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = (360 + interim_angle) - 315;
			interim_angle = 315 - incidence_angle;
			angle = interim_angle;
		}

		//check for ball traveling north west
		if (angle < 180 && angle > 90)
		{
			interim_angle = (angle + 180);

			if(interim_angle <= 315)
			{
				incidence_angle = 315 - interim_angle;
				interim_angle = 315 + incidence_angle;
				angle = interim_angle;
			}

			else if(interim_angle > 315)
			{
				incidence_angle = interim_angle - 315;
				interim_angle = 315 - incidence_angle;
				angle = interim_angle;
			}
		}
		break;

		case 6:   //Condition 6: collision against north east corner of a brick
		//ball movement can be north west, south east, west and south, south west

		//check for ball traveling north-west and west
		if (angle > 90 && angle <= 180)
		{
			interim_angle = angle + 180;
			incidence_angle = (360+45) - interim_angle;
			interim_angle = 45 + incidence_angle;
			angle = interim_angle;
		}
		//check for ball traveling south east and south
		else if (angle >= 270 && angle < 360)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = interim_angle - 45;
			interim_angle = (360+45) - incidence_angle;
			angle = interim_angle;
		}

		//check for ball traveling in the south west direction
		if (angle > 180 && angle < 270)
		{
			interim_angle = (float) fmod((angle + 180),360);

			if(interim_angle <= 45)
			{
				incidence_angle = 45 - interim_angle;
				interim_angle = 45 + incidence_angle;
				angle = interim_angle;
			}

			else if(interim_angle > 45)
			{
				incidence_angle = interim_angle - 45;
				interim_angle = 45 - incidence_angle;
				angle = interim_angle;
			}
		}
		break;

		case 7: //Condition 7: collision with north west corner of all bricks
		//ball movement can be north east, south west, east and south, south east

		//check for ball traveling north-east and east
		if ((angle < 90 && angle >= 0)|| angle ==360)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = interim_angle - 135;
			interim_angle = interim_angle - 2*(incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling south west and south
		else if (angle > 180 && angle <= 270)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = 135 - interim_angle;
			interim_angle = (float) fmod(interim_angle + 2*(incidence_angle), 360);
			angle = interim_angle;
		}

		//check for ball traveling in the south east direction
		if (angle > 270 && angle < 360)
		{
			interim_angle = (float) fmod((angle + 180),360);

			if(interim_angle <= 135)
			{
				incidence_angle = 135 - interim_angle;
				interim_angle = 135 + incidence_angle;
				angle = interim_angle;
			}

			else if(interim_angle > 135)
			{
				incidence_angle = interim_angle - 135;
				interim_angle = 135 - incidence_angle;
				angle = interim_angle;
			}
		}

		break;

		case 8: //condition 8: collision with south-west corner of all bricks
		//ball movement can be north west, south east, east and north, north east

		//check for ball traveling north-west and north

		if (angle >= 90 && angle < 180)

		{
			interim_angle = angle + 180;
			incidence_angle = interim_angle - 225;
			interim_angle = interim_angle - 2*(incidence_angle);
			angle = interim_angle;
		}
		//check for ball traveling south east and east
		else if ((angle > 270 && angle <= 360) || angle==0)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = 225 - interim_angle;
			interim_angle = interim_angle + 2*(incidence_angle);
			angle = interim_angle;
		}

		//check for ball traveling in the north east direction
		if (angle > 0 && angle < 90)
		{
			interim_angle = (angle + 180);

			if(interim_angle <= 225)
			{
				incidence_angle = 225 - interim_angle;
				interim_angle = 225 + incidence_angle;
				angle = interim_angle;
			}

			else if(interim_angle > 225)
			{
				incidence_angle = interim_angle - 225;
				interim_angle = 225 - incidence_angle;
				angle = interim_angle;
			}
		}

		case 9:    // Condition: hitting the bar in the A+ region
		//check for ball traveling south-east
		if (angle > 270 && angle <360)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = interim_angle - 90;
			interim_angle = interim_angle - 2*(incidence_angle);

			if(interim_angle + 15 <= 165)
			interim_angle += 15;
			else
			interim_angle = 165;
			angle = interim_angle;
		}
		//check for ball traveling south west
		else if (angle > 180 && angle < 270)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = 90 - interim_angle;
			interim_angle = interim_angle + 2*(incidence_angle);

			if(interim_angle + 15 <= 165)
			interim_angle += 15;
			else
			interim_angle = 165;
			angle = interim_angle;
		}

		//check for ball traveling south
		else if (angle == 270)
		{
			angle = 90 + 15;
		}
		break;

		case 10:    // Condition: hitting the bar in the A- region
		//check for ball traveling south-east
		if (angle > 270 && angle <360)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = interim_angle - 90;
			interim_angle = interim_angle - 2*(incidence_angle);

			if(interim_angle - 15 >= 15)
			interim_angle -= 15;
			else
			interim_angle = 15;
			angle = interim_angle;
		}
		//check for ball traveling south west
		else if (angle > 180 && angle < 270)
		{
			interim_angle = (float) fmod((angle + 180),360);
			incidence_angle = 90 - interim_angle;
			interim_angle = interim_angle + 2*(incidence_angle);

			if(interim_angle - 15 >= 15)
			interim_angle -= 15;
			else
			interim_angle = 15;
			angle = interim_angle;
		}

		//check for ball traveling south
		else if (angle == 270)
		{
			angle = 90 - 15;
		}

	} //switch end
}





void increment_by_one(int * xdir_ptr, int * ydir_ptr)

{
	//xil_printf("Entering increment by one function \n");
	float output;

	/*
	value of angle affects exactly what sort of increment is done
	*/

	if (angle < 90 || angle > 270)
	{
		////xil_printf (" value of xdir is %d\n", *xdir_ptr);
		*xdir_ptr = *xdir_ptr + INCREMENT_ONE_VALUE;
		////xil_printf (" value of xdir after incrementing is %d\n", *xdir_ptr);
		output = tan(angle*(PI/180));
		////xil_printf ("output value is %f\n", output);
		*ydir_ptr = (int) (output * (*xdir_ptr));
		//xil_printf (" value of ydir  is %d\n", *ydir_ptr);
		//xil_printf (" value of xdir after calculating ydir is %d\n", *xdir_ptr);
	}
	if (angle > 90 && angle < 270)
	{
		*xdir_ptr = *xdir_ptr - INCREMENT_ONE_VALUE;
		output = tan(angle*(PI/180));
		*ydir_ptr = (int) (output* (*xdir_ptr));
		//xil_printf (" value of ydir  is %d\n", *ydir_ptr);
		//xil_printf (" value of xdir after calculating ydir is %d\n", *xdir_ptr);
	}

	if (angle==90)
	{
		//xil_printf (" value of ydir before increment  is %d\n", *ydir_ptr);
		*ydir_ptr = *ydir_ptr + INCREMENT_ONE_VALUE;
		//xil_printf (" value of ydir  is %d\n", *ydir_ptr);
		//xil_printf (" value of xdir after calculating ydir is %d\n", *xdir_ptr);
	}

	if (angle==270)
	{
		//xil_printf (" value of ydir before increment  is %d\n", *ydir_ptr);
		*ydir_ptr = *ydir_ptr - INCREMENT_ONE_VALUE;
		//xil_printf (" value of ydir  is %d\n", *ydir_ptr);
		//xil_printf (" value of xdir after calculating ydir is %d\n", *xdir_ptr);
	}

}

int check_collision( int * xcoordinate, int * ycoordinate)
{
	int COLUMN_ID;
	int ROW_ID;
	int x, height;
	int i;

	//Condition 1: check for collision against left side of screen i.e. x<=0
	if((*xcoordinate)- CIRCLE_RADIUS <= GAMEAREA_LEFT)
	{
		ID = 1;
		*xcoordinate = GAMEAREA_LEFT + CIRCLE_RADIUS; //no particular need except for edge case
		//set_brick_collision_protocol(LEFT_HIT, LEFT_HIT);
		return 1;
	}

	//Condition 2: check for collision against right side of screen i.e. x>=639
	if ((*xcoordinate)+ CIRCLE_RADIUS >= GAMEAREA_RIGHT)
	{
		ID= 2;
		*xcoordinate = GAMEAREA_RIGHT - CIRCLE_RADIUS; //no particular need except for edge case
		//set_brick_collision_protocol(RIGHT_HIT, RIGHT_HIT);
		return 1;

	}
	//Condition 3: check for collision against top side of screen i.e. y<=0
	if((*ycoordinate)- CIRCLE_RADIUS <= GAMEAREA_TOP)
	{
		//do an angle check to prevent false collision detection in cases when movement away from collision point is pending
		if (angle> 0 && angle < 180)
		{
			ID= 3;
			*ycoordinate = GAMEAREA_TOP + CIRCLE_RADIUS; // enforcing condition in case value is less than 0
			//set_brick_collision_protocol(TOP_HIT, TOP_HIT);
			return 1;
		}
	}

	//Condition 4: check for collision against down side of screen i.e. y>=479
	if ((*ycoordinate)+ CIRCLE_RADIUS >= GAMEAREA_BTM)
	{
		//do an angle check to prevent false collision detection in cases when movement away from collision point is pending
		if(angle> 180 && angle < 360)
		{
			ID= 4;
			*ycoordinate = GAMEAREA_BTM - CIRCLE_RADIUS; // enforcing condition in case value is more than 479
			set_brick_collision_protocol(BOTTOM_HIT, BOTTOM_HIT);
			return 1;
		}
	}

	// brick dimensions: 15 pixels width, 40 pixels length



	//Condition 5: Collision check for brick edges - specific cases only


	// For checking for each brick run a for loop representing COLUMN_ID with a nested for loop representing ROW_ID
	//assumption made: only one brick at any given time can be hit


	for (COLUMN_ID = 1; COLUMN_ID <= TOTAL_COLUMNS; COLUMN_ID++)
	{
		for (ROW_ID = 1; ROW_ID <= TOTAL_ROWS; ROW_ID++)
		{
			if(brickflags[ROW_ID-1][COLUMN_ID -1]==1)
			{
				if((*xcoordinate + CIRCLE_RADIUS >= x_offset + INTERBRICK_X*(COLUMN_ID-1) && *xcoordinate + CIRCLE_RADIUS < x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40)|| (*xcoordinate - CIRCLE_RADIUS <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 && *xcoordinate - CIRCLE_RADIUS > x_offset + INTERBRICK_X*(COLUMN_ID-1)))
				{
					if((*ycoordinate + CIRCLE_RADIUS >= y_offset + INTERBRICK_Y*(ROW_ID-1) && *ycoordinate + CIRCLE_RADIUS < y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 )|| (*ycoordinate - CIRCLE_RADIUS <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 && *ycoordinate - CIRCLE_RADIUS > y_offset + INTERBRICK_Y*(ROW_ID-1)))
					{
						for(x=-CIRCLE_RADIUS; x<=CIRCLE_RADIUS; x+= CIRCLE_RADIUS)
						{
							height = (int) (sqrt(CIRCLE_RADIUS * CIRCLE_RADIUS - x * x)) ;

							if (height==0) //Special case
							{
								//south east corner
								if ((*ycoordinate - height == y_offset + INTERBRICK_Y*(ROW_ID-1) + 15)  && (*xcoordinate + x <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40) && (*xcoordinate + x >= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 - INCREMENT_ONE_VALUE))
								{
									ID = 5;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//north east corner
								else if ((*ycoordinate + height == y_offset + INTERBRICK_Y*(ROW_ID-1))&& (*xcoordinate + x <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40) && (*xcoordinate + x >= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 - INCREMENT_ONE_VALUE))
								{
									ID = 6;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//north west
								else if ((*ycoordinate + height == y_offset + INTERBRICK_Y*(ROW_ID-1)) && (*xcoordinate + x >= x_offset + INTERBRICK_X*(COLUMN_ID-1)) && (*xcoordinate + x <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + INCREMENT_ONE_VALUE))
								{
									ID = 7;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//south west
								else if ((*ycoordinate - height == y_offset + INTERBRICK_Y*(ROW_ID-1) + 15) && (*xcoordinate + x >= x_offset + INTERBRICK_X*(COLUMN_ID-1)) && (*xcoordinate + x <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + INCREMENT_ONE_VALUE))
								{
									ID = 8;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}


							}

							else if (height== CIRCLE_RADIUS) //special case
							{

								//south east corner
								if ((*ycoordinate - height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15) && (*ycoordinate - height >= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 - INCREMENT_ONE_VALUE)  && (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40))
								{
									ID = 5;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//north east corner
								else if ((*ycoordinate + height >= y_offset + INTERBRICK_Y*(ROW_ID-1))&& (*ycoordinate + height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + INCREMENT_ONE_VALUE)&& (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40))
								{
									ID = 6;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//north west
								else if ((*ycoordinate + height >= y_offset + INTERBRICK_Y*(ROW_ID-1)) && ((*ycoordinate + height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + INCREMENT_ONE_VALUE)) && (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1)))
								{
									ID = 7;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//south west
								else if ((*ycoordinate - height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15) && (*ycoordinate - height >= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 - INCREMENT_ONE_VALUE)&& (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1)))
								{
									ID = 8;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}


							}
						} // CIRCLE for loop end
					} // y coordinate if end
				} // x coordinate if end
			} //bricks flag check if end
		}// nested for loop end
	} //outer for loop end

	// Condition 6 - Bricks - non corners
	for (COLUMN_ID = 1; COLUMN_ID <= TOTAL_COLUMNS; COLUMN_ID++)
	{
		for (ROW_ID = 1; ROW_ID <= TOTAL_ROWS; ROW_ID++)
		{
			//Optimize search strategy
			// 1. Check if the brick still exists in the place using COLUMN_ID and ROW_ID
			// 2. Check whether the ball is within range of the boundaries of the brick
			if(brickflags[ROW_ID-1][COLUMN_ID -1]==1)
			{
				if((*xcoordinate + CIRCLE_RADIUS >= x_offset + INTERBRICK_X*(COLUMN_ID-1) && *xcoordinate + CIRCLE_RADIUS < x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40)|| (*xcoordinate - CIRCLE_RADIUS <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 && *xcoordinate - CIRCLE_RADIUS > x_offset + INTERBRICK_X*(COLUMN_ID-1)))
				{
					if((*ycoordinate + CIRCLE_RADIUS >= y_offset + INTERBRICK_Y*(ROW_ID-1) && *ycoordinate + CIRCLE_RADIUS < y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 )|| (*ycoordinate - CIRCLE_RADIUS <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 && *ycoordinate - CIRCLE_RADIUS > y_offset + INTERBRICK_Y*(ROW_ID-1)))
					{
						//Check if left side of the brick is hit
						if (((*xcoordinate)+ CIRCLE_RADIUS >= x_offset + INTERBRICK_X*(COLUMN_ID-1)) && ((*xcoordinate)+ CIRCLE_RADIUS <= x_offset + INTERBRICK_X*(COLUMN_ID-1)+ 40))
						{
							if (angle <90 || angle > 270) //angle check for guarding against false collision
							{
								for (i=0; i<=15; i++)
								//for (i=1; i<15; i++)   //leaving edge cases for corner detection
								{
									if ((y_offset + INTERBRICK_Y*(ROW_ID-1) + i)== *ycoordinate)
									{

										//*xcoordinate = x_offset + INTERBRICK_X*(COLUMN_ID-1) - CIRCLE_RADIUS; //Not necessary except for edge case
										ID = 2;  //hitting the bottom of the brick is same as hitting top of screen
										set_brick_collision_protocol(ROW_ID,COLUMN_ID);
										return 1;
									}
								}
							}
						}

						//Check if right side of the brick is hit
						if (((*xcoordinate) - CIRCLE_RADIUS <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40) && ((*xcoordinate) - CIRCLE_RADIUS >= x_offset + INTERBRICK_X*(COLUMN_ID-1)) )
						{
							if (angle>90 && angle<270) //angle check for guadring against false collision
							{
								for (i=0; i<=15; i++)

								{
									if ((y_offset + INTERBRICK_Y*(ROW_ID-1) + i)== *ycoordinate)
									{
										//*xcoordinate = x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 + CIRCLE_RADIUS; //Not necessary except for edge case
										ID = 1;  //hitting the bottom of the brick is same as hitting top of screen
										set_brick_collision_protocol(ROW_ID, COLUMN_ID);
										return 1;
									}
								}
							}
						}

						//Check if the top side of brick is hit
						if(((*ycoordinate) + CIRCLE_RADIUS >= (y_offset + INTERBRICK_Y*(ROW_ID-1))) && ((*ycoordinate) + CIRCLE_RADIUS <= (y_offset + INTERBRICK_Y*(ROW_ID-1)+ 15)))
						{
							if (angle> 180 && angle <360) //do an angle check in case of pending away movement to avoid false collision detection/ false collision in general
							{
								//check if xcoordinate is within range of brick's x position
								for (i=0; i<=40; i++)
								{
									if ((x_offset + INTERBRICK_X*(COLUMN_ID-1) + i)== *xcoordinate)
									{
										ID = 4;  //hitting the top of the brick is same as hitting bottom of screen
										set_brick_collision_protocol(ROW_ID, COLUMN_ID);
										//*ycoordinate =  (y_offset + INTERBRICK_Y*(ROW_ID-1)) - CIRCLE_RADIUS; // enforcing y co-ordinate in case of collision
										return 1;
									}
								}
							}
						}

						//Check if bottom side of brick is hit
						if(((*ycoordinate)- CIRCLE_RADIUS <= (y_offset + INTERBRICK_Y*(ROW_ID-1) + 15)) && ((*ycoordinate)- CIRCLE_RADIUS >= (y_offset + INTERBRICK_Y*(ROW_ID-1))))
						{
							//do an angle check in case of pending away movement to avoid false collision detection/ false collision in general
							if(angle> 0 && angle < 180)
							{

								//check if xcoordinate is within range of brick's x position
								for (i=0; i<=40; i++)
								{
									if ((x_offset + INTERBRICK_X*(COLUMN_ID-1) + i)== *xcoordinate)
									{
										ID = 3;  //hitting the bottom of the brick is same as hitting top of screen
										set_brick_collision_protocol(ROW_ID, COLUMN_ID);
										//*ycoordinate =  CIRCLE_RADIUS + (y_offset + INTERBRICK_Y*(ROW_ID-1) + 15); // enforcing y co-ordinate in case of collision
										return 1 ;
									}
								}
							}
						}
					}//nested if y co-ordinate end
				} //nested if x co-ordinate end
			}//brick_check if end

		}//outer loop end
	}// inner loop end


	//Condition 7: Collision check for brick edges - all other cases

	for (COLUMN_ID = 1; COLUMN_ID <= TOTAL_COLUMNS; COLUMN_ID++)
	{
		for (ROW_ID = 1; ROW_ID <= TOTAL_ROWS; ROW_ID++)
		{
			if(brickflags[ROW_ID-1][COLUMN_ID -1]==1)
			{
				if((*xcoordinate + CIRCLE_RADIUS >= x_offset + INTERBRICK_X*(COLUMN_ID-1) && *xcoordinate + CIRCLE_RADIUS < x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40)|| (*xcoordinate - CIRCLE_RADIUS <= x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40 && *xcoordinate - CIRCLE_RADIUS > x_offset + INTERBRICK_X*(COLUMN_ID-1)))
				{
					if((*ycoordinate + CIRCLE_RADIUS >= y_offset + INTERBRICK_Y*(ROW_ID-1) && *ycoordinate + CIRCLE_RADIUS < y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 )|| (*ycoordinate - CIRCLE_RADIUS <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 && *ycoordinate - CIRCLE_RADIUS > y_offset + INTERBRICK_Y*(ROW_ID-1)))
					{
						for(x=-CIRCLE_RADIUS+1; x<CIRCLE_RADIUS; x++)
						{
							height = (int) (sqrt(CIRCLE_RADIUS * CIRCLE_RADIUS - x * x)) ;

							if (height== CIRCLE_RADIUS)
							{
								//do nothing as special case takes care of this
							}
							else
							{
								//Collision check for south east corner
								if ((*ycoordinate - height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15) && (*ycoordinate - height >= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 - 8) && (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40))
								{
									ID = 5;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

								//Collision check for north east corner

								else if ((*ycoordinate + height >= y_offset + INTERBRICK_Y*(ROW_ID-1)) && (*ycoordinate + height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 7)&& (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1) + 40))
								{
									ID = 6;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}
								//Collision check for north west corner


								else if ((*ycoordinate + height >= y_offset + INTERBRICK_Y*(ROW_ID-1)) && (*ycoordinate + height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 7)&& (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1)))
								{
									ID = 7;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}
								//Collision check for south west corner

								else if ((*ycoordinate - height <= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15) && (*ycoordinate - height >= y_offset + INTERBRICK_Y*(ROW_ID-1) + 15 - 8) && (*xcoordinate + x == x_offset + INTERBRICK_X*(COLUMN_ID-1)))
								{
									ID = 8;
									set_brick_collision_protocol(ROW_ID, COLUMN_ID);
									return 1;
								}

							}//else end

						} // CIRCLE for loop end
					} // y coordinate if end
				} // x coordinate if end
			} //bricks flag check if end
		}// nested for loop end
	} //outer for loop end

	return 0;
}

void set_brick_collision_protocol(int ROW_ID, int COL_ID)
{
	pthread_mutex_lock(&brickcollisionprotocol_mutex);
	collision_brick_pending_status = 1;
	collided_brick_row = ROW_ID;
	collided_brick_col = COL_ID;
	pthread_mutex_unlock(&brickcollisionprotocol_mutex);

};


int check_collision_bar(int cursor_bar_local, int * xcoordinate, int * ycoordinate)
{
	int i;

  if (poweruplengthen == 0)
{
	if (angle> 180 && angle <360) //do an angle check in case of pending away movement to avoid false collision detection/ false collision in general
	{

		//zone N
		for (i=-20; i<20; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}

		//zone S+
		for (i=20; i<30; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision

				//Speed increases by 100 px/sec = 4 pixels/frame - maximum speed is 1000 px/sec or 40 px/frame
				//use limit as 20
				if (SPEED + 4 <= MAX_BALLSPEED)
				SPEED +=4;
				else
				SPEED = MAX_BALLSPEED;

				set_brick_collision_protocol(BAR_HIT, BAR_HIT);

				return 1;
			}
		}
		//zone S-
		for (i=-21; i>=-30; i--)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision

				//Speed slows by 100 px/sec = 4 pixels/frame - minimum speed is 50 px/sec or 2 px/frame

				if (SPEED - 4 >= MIN_BALLSPEED)
				SPEED -=4;
				else
				SPEED = MIN_BALLSPEED;

				set_brick_collision_protocol(BAR_HIT, BAR_HIT);

				return 1;
			}
		}

		//zone A+
		for (i=30; i<40; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				//ID = 9;  //hitting the top of the bar is same as hitting bottom of screen
				ID = 9;
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}
		//zone A-
		for (i=-31; i>=-40; i--)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				//ID = 10;  //hitting the top of the bar is same as hitting bottom of screen
				ID= 10;
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}
	}// anglecheck if condition end
	return 0;
   }// barlengthen if condition end

  if (poweruplengthen == 1)
{
	if (angle> 180 && angle <360) //do an angle check in case of pending away movement to avoid false collision detection/ false collision in general
	{

		//zone N
		for (i=-40; i<40; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}

		//zone S+
		for (i=40; i<60; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision

				//Speed increases by 100 px/sec = 4 pixels/frame - maximum speed is 1000 px/sec or 40 px/frame
				//use limit as 20
				if (SPEED + 4 <= MAX_BALLSPEED)
				SPEED +=4;
				else
				SPEED = MAX_BALLSPEED;

				set_brick_collision_protocol(BAR_HIT, BAR_HIT);

				return 1;
			}
		}
		//zone S-
		for (i=-41; i>=-60; i--)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				ID = 4;  //hitting the top of the bar is same as hitting bottom of screen
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision

				//Speed slows by 100 px/sec = 4 pixels/frame - minimum speed is 50 px/sec or 2 px/frame

				if (SPEED - 4 >= MIN_BALLSPEED)
				SPEED -=4;
				else
				SPEED = MIN_BALLSPEED;

				set_brick_collision_protocol(BAR_HIT, BAR_HIT);

				return 1;
			}
		}

		//zone A+
		for (i=60; i<80; i++)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				//ID = 9;  //hitting the top of the bar is same as hitting bottom of screen
				ID = 9;
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}
		//zone A-
		for (i=-61; i>=-80; i--)
		{
			if ( cursor_bar_local + i == *xcoordinate)
			{
				//ID = 10;  //hitting the top of the bar is same as hitting bottom of screen
				ID= 10;
				* ycoordinate = BAR_TOP - CIRCLE_RADIUS - 1; // enforcing y co-ordinate in case of collision
				set_brick_collision_protocol(BAR_HIT, BAR_HIT);
				return 1;
			}
		}
	}// anglecheck if condition end
	return 0;
   }// barlengthen if condition end

}
