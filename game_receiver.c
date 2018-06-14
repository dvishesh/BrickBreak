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
#include "xtft.h"
#include "xgpio.h"
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

#define TFT_DEVICE_ID 	XPAR_TFT_0_DEVICE_ID
#define DDR_HIGH_ADDR 	XPAR_PS7_DDR_0_S_AXI_HIGHADDR

#ifndef DDR_HIGH_ADDR
#warning "CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H"
#endif

// display macros
#define DISPLAY_COLUMNS	640
#define DISPLAY_ROWS 480
#define GAMEAREA_LEFT 60
#define GAMEAREA_TOP 60
#define GAMEAREA_RIGHT 514
#define GAMEAREA_BTM 419
#define MAXREDCOLUMNS 2

//bar
#define BAR_TOP 405
#define BAR_BTM	409
#define HALFBARLENG	40
#define INITIAL_BAR 288
#define CURSOR_LEFTX 61 //GAMEAREA_LEFT + HALFBARLENG + 1
#define CURSOR_RIGHTX 513 //GAMEAREA_RIGHT - HALFBARLENG -1

//msgqueue addresses
#define DRAWBRICK_Q 	21
#define GAME_Q 	22

//colour
#define BRICK_COLOUR 0x00ffff00	//yellow
#define BRICK_COLOUR_RED 0x00ff0000	//red
#define BRICK_COLOUR_STRIPES 0x00000000	//black
#define GAMEAREA_COLOUR 0x0000ff00	//green
#define BAR_COLOUR 0x000000ff	//blue
#define BAR_S_COLOUR 0x0020b2aa //light sea green
#define BAR_A_COLOUR 0x00ffd700	//gold
#define BALL_COLOUR 0x00ffa500	//orange
#define TEXT_COLOUR	0x000000ff	//green
#define TEXTBOX_COLOUR 0x00003d7c //cyan
#define BLACK_COLOUR 0x00000000	//black

//gamestate
#define GAME_NORMAL 0
#define GAME_WIN 1
#define GAME_LOSE 2
#define GAME_PAUSE 3
#define GAME_RESET 4
#define GAME_BALLHELD 5

//message for
#define MESSAGE_FOR_BALL 0
#define MESSASGE_FOR_GAME 1

//buttons
#define BTN_CENTER 1
#define BTN_BTM 2
#define BTN_LEFT 4
#define BTN_RIGHT 8
#define BTN_TOP 16

//ball macros
#define CIRCLE_RADIUS 7
#define INITIAL_X INITIAL_BAR
#define INITIAL_Y BAR_TOP - CIRCLE_RADIUS
#define	INITIAL_BALLSPEED 5

//brick
#define TOTAL_COLUMNS 10
#define TOTAL_ROWS 8
#define INTERBRICK_Y 20
#define INTERBRICK_X 45
#define BRICK_LENGTH 40
#define BRICK_HEIGHT 15
#define NUM_CRYSTAL_BRICK 10

//collision
#define BOTTOM_HIT 20
#define BAR_HIT 24

/**
* User has to specify a 2MB memory space for filling the frame data.
* This constant has to be updated based on the memory map of the
* system.
*/
#define TFT_FRAME_ADDR        0x10000000

/**************************** Type Definitions ******************************/

typedef struct {
	int id;
	int status;
	int isRed;
} msg_col;


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
//display
void* thread_bar(void);						// manages and draws bar
void* thread_game(void);					// handles collision, draw score/speed/brick/ball
void* thread_drawbrick(void);			// draws bricks whenever receive new info from msgqueue
void* thread_bcol1(void);
void* thread_bcol2(void);
void* thread_reset(void);


//generic function declarations
int main_prog(void);
int Tft_init(u32 TftDeviceId);	// draw initial game screen
void init_screen();
void init_variables();
void init_threads();
static void gpPBIntHandler(void *arg);	// handle pushbutton
void send(int destid, void *msgptr, size_t msgsize);	// send to msgqueue
void receive(int msgqid, void* msgptr, size_t msgsize);	// receive from rmsgqueue
void numbertocstring(int num, char* charptr);	//convert number to null-terminated char string
void tryRed2(unsigned int redID);
int generatebitmask(int brickrow);
void resetHandler();

void drawBar(XTft *Tft, int cursor, int* cursor_drawn_ptr, int poweruplengthen_prev);
void drawBall(XTft *Tft, int x, int y, int* ball_ptr_x, int* ball_ptr_y);
void drawBCol(XTft *Tft, int bcol_id, int bcol_status, int bcol_isRed);	//draws a single column
int XTft_DrawSolidBox(XTft *Tft, int x1, int y1, int x2, int y2, unsigned int col);
void XTft_DrawTextBox(XTft *Tft, int x1, int y1, int x2, int y2, char* cstringtext);
void XTft_DrawNumberBox(XTft *Tft, int x1, int y1, int x2, int y2, int someNumber);
void XTft_DrawSolidCircle(XTft *Tft, int x, int y);

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

// declare the semaphore
sem_t sem_red, sem_changeback;

// software mutex declaration
//display
pthread_mutex_t uart_mutex;
pthread_mutex_t cursor_mutex;
pthread_mutex_t tft_mutex;
pthread_mutex_t brick_mutex;		//protect bricks[]
pthread_mutex_t red_mutex;			//protect redcol[]
//pthread_mutex_t gamestate_mutex;	//might be needed [debug]


// threads declaration
//display threads
pthread_t tdrawbrick, tgame, tbar, treset;
pthread_t tcol1, tcol2;


static XTft TftInstance;
XGpio gpPB; //PB device instance.


//global variables
//display
unsigned int clock_ticks_previrq;
unsigned int clock_ticks_buttonheld;
unsigned int redcol[MAXREDCOLUMNS];
unsigned int redcolprev[MAXREDCOLUMNS];
unsigned int bricks[TOTAL_COLUMNS];
signed int cursor_curr;			// absolute position
signed int redcount;
unsigned char val_prev;

int col_count;
int gamestateflag;
int receive_packet_no;
int ball_drawn_x;
int ball_drawn_y;
int crystalbrick[NUM_CRYSTAL_BRICK];
int poweruplengthen;

//
//	Thread functions
//

void* thread_bar()
{
	int cursor_temp = INITIAL_BAR;
	int cursor_drawn = INITIAL_BAR;
	int poweruplengthen_last = 0;
	int halfbarlength_local = HALFBARLENG;

	unsigned int clock_ticks_curr;

	while(1)
	{
		//handles win/lose/pause by sleeping
		while((gamestateflag == GAME_WIN) || (gamestateflag == GAME_LOSE) || (gamestateflag == GAME_PAUSE))
		{
			sleep(40);
		}
		if(gamestateflag == GAME_RESET)
		{
			pthread_mutex_lock(&uart_mutex);
			xil_printf("Bar thread exiting..\n\n");
			pthread_mutex_unlock(&uart_mutex);

			pthread_exit(0);
		}

		if (val_prev == BTN_LEFT || val_prev == BTN_RIGHT )	// button is being held
		{
			clock_ticks_curr = xget_clock_ticks();

			if(clock_ticks_curr - clock_ticks_buttonheld >25)	//held for more than 250 ms
			{
				//move bar quickly
				//200px per sec @ 25fps
				//8px per 1/25 sec @ 1 frame per 1/25 sec
				//1/25 sec = 40ms

				if(val_prev == BTN_LEFT)
				{
					pthread_mutex_lock (&cursor_mutex);	//[debug] to check if endcase are set correctly
					cursor_curr = cursor_curr - 8;
					if (cursor_curr < CURSOR_LEFTX + halfbarlength_local )
					cursor_curr = CURSOR_LEFTX + halfbarlength_local ;		// set to leftmost possible position
					pthread_mutex_unlock (&cursor_mutex);

				}
				else if (val_prev == BTN_RIGHT)
				{
					pthread_mutex_lock (&cursor_mutex);
					cursor_curr = cursor_curr + 8;
					if (cursor_curr > CURSOR_RIGHTX - halfbarlength_local)
					cursor_curr = CURSOR_RIGHTX - halfbarlength_local; 	//set to rightmost possible location
					pthread_mutex_unlock (&cursor_mutex);
				}
			}

		}
		pthread_mutex_lock (&cursor_mutex);
		cursor_temp = cursor_curr;
		pthread_mutex_unlock (&cursor_mutex);

		// redraw bar if bar is outdated
		if ((cursor_temp != cursor_drawn) || (poweruplengthen_last != poweruplengthen))
		{
			if (poweruplengthen == 1)
				halfbarlength_local  = HALFBARLENG + HALFBARLENG;
			else
				halfbarlength_local  = HALFBARLENG;

			pthread_mutex_lock (&cursor_mutex);
			if (cursor_curr < CURSOR_LEFTX + halfbarlength_local )
				cursor_curr = CURSOR_LEFTX + halfbarlength_local ;	// set to leftmost possible position
			else if (cursor_curr > CURSOR_RIGHTX - halfbarlength_local)
				cursor_curr = CURSOR_RIGHTX - halfbarlength_local; 	//set to rightmost possible location
			cursor_temp = cursor_curr;
			pthread_mutex_unlock (&cursor_mutex);


			if(gamestateflag == GAME_BALLHELD) //also draw the ball
			{
				drawBall(&TftInstance, ball_drawn_x + cursor_temp - cursor_drawn, ball_drawn_y, &ball_drawn_x, &ball_drawn_y);
			}

			// send new cursor position to ball thread	[debug]
			drawBar(&TftInstance, cursor_temp, &cursor_drawn, poweruplengthen_last);
			poweruplengthen_last = poweruplengthen;

		}

		sleep(40); //to update bar every 40ms while being held
	}
}

void* thread_drawbrick()
{
	msg_col msg_recd;
	int drawbrick_timestamp = xget_clock_ticks();
	int drawbrick_timestamp2;
	int drawbrick_timeinterval;

	while (1)
	{
		// read msg from msgqueue and update that column
		receive(DRAWBRICK_Q, &msg_recd, sizeof(msg_col));
		//does not need to handle win/lose/pause/ballheld as will be blocked at blocking receive


		if(gamestateflag == GAME_RESET)
		{
			pthread_mutex_lock(&uart_mutex);
			xil_printf("drawbrick thread exiting..\n\n");
			pthread_mutex_unlock(&uart_mutex);
			pthread_exit(0);
		}
		drawbrick_timestamp2 = xget_clock_ticks();
		drawbrick_timeinterval = drawbrick_timestamp2 - drawbrick_timestamp;
		drawbrick_timestamp = xget_clock_ticks();

		//pthread_mutex_lock(&uart_mutex);
		//xil_printf("Draw brick thread invoked after %d * 10 milliseconds since previous invocation\n\n", drawbrick_timeinterval);
		//pthread_mutex_unlock(&uart_mutex);

		//draw Column - //msg_recd.id ranges from 1-10
		drawBCol(&TftInstance, msg_recd.id, msg_recd.status, msg_recd.isRed);

		//pthread_mutex_lock(&uart_mutex);
		//xil_printf(" \nCol: %d drawn with status: %x isRed = %d \n", msg_recd.id, msg_recd.status, msg_recd.isRed);
		//pthread_mutex_unlock(&uart_mutex);
	}
}
void* thread_game(void)
{
	int i,k;
	int score_nextlevel = 10;
	int ballspeed_shown = INITIAL_BALLSPEED;
	int brickrow = 0;
	int brickcol = 0;
	int tempmask = 0xFF;
	int game_score = 0;
	int game_brickleft = TOTAL_ROWS *  TOTAL_COLUMNS;
	int shown_score = 0;
	int shown_brickleft = TOTAL_ROWS * TOTAL_COLUMNS;
	int firstrun = 1;
	int poweruphold = 0;

	unsigned int lastupdated_time = xget_clock_ticks();
	unsigned int curr_time = xget_clock_ticks();
	unsigned int shown_time = 0;
	unsigned int game_time = 0;
	unsigned int poweruphold_timer = xget_clock_ticks();
	unsigned int poweruplengthen_timer = xget_clock_ticks();

	msg_col msg_temp;
	msg_temp.id = 1;
	msg_temp.status = 0xFF;
	msg_temp.isRed = 0;

	msg_ball msg_ball_recd;
	msg_game msg_game_tosend;

	int gamethread_timestamp = xget_clock_ticks();
	int gamethread_timestamp2;
	int gamethread_timeinterval;
	int gamethread_finishtimestamp;
	int gamethread_finishinterval;
	int fps_prev = xget_clock_ticks();
	int frame_count = 0;


	while(1)
	{
		// use mailbox etc to be notified of collision
		// receives info on ballx, bally, ballspeed, brickrow, brickcol
		// brickrow is 1-8 or BOTTOM_HIT or BAR_HIT
		// brickcol is 1-10


		// could be GAME_WIN, GAME_PAUSE, GAME_RESET, GAME_NORMAL
		XMbox_ReadBlocking(&Mbox, &msg_ball_recd, sizeof(msg_ball));
		//XMutex_Lock(&mutex, MUTEX_NUM);
		//xil_printf("-- Sucessfully received IN RECEIVER --\r\n");
		//XMutex_Unlock(&mutex, MUTEX_NUM);

		//debug
		if (msg_ball_recd.message_for == MESSAGE_FOR_BALL)
		{
			XMutex_Lock(&mutex, MUTEX_NUM);
			xil_printf("-- Error - GAME received message meant for BALL!! --\r\n");
			XMutex_Unlock(&mutex, MUTEX_NUM);
			while(1); //stall here
		}

		//process the received msg
		//GAME_LOSE
		if (msg_ball_recd.brickrow == BOTTOM_HIT)
		{
			gamestateflag = GAME_LOSE;
			XTft_DrawTextBox(&TftInstance, 305, 230, 364, 249, "LOSE!!!");
			//breaks only when player resets
			while(gamestateflag == GAME_LOSE)
			{
				sleep(40);
			}
		}

		// GAME_PAUSE
		if(gamestateflag == GAME_PAUSE)
		{
			XTft_DrawTextBox(&TftInstance, 315, 230, 379, 249, "<PAUSE>");

			// updates game_time at point of pause
			curr_time = xget_clock_ticks();
			game_time = game_time + curr_time - lastupdated_time;

			while(gamestateflag == GAME_PAUSE)
			{
				sleep(40);
			}


			XTft_DrawSolidBox(&TftInstance, 315, 230, 379, 249, GAMEAREA_COLOUR);
			//msg_game_tosend.game_status = gamestateflag;
			//XMbox_WriteBlocking(&Mbox, &msg_game_tosend, sizeof(msg_game));

			curr_time = xget_clock_ticks();
			lastupdated_time = curr_time;
		}
		//cannot be at GAME_WIN as has been handled

		//GAME_RESET
		if(gamestateflag == GAME_RESET)
		{
			resetHandler();
			// add in non-blocking receive to clear game_Q
			pthread_mutex_lock(&uart_mutex);
			xil_printf("game thread exiting..\n\n");
			pthread_mutex_unlock(&uart_mutex);
			pthread_exit(0);
		}


		// GAME_BALLHELD
		if(msg_ball_recd.brickrow == BAR_HIT && poweruphold == 1) //barcollision occured
		{
			gamestateflag = GAME_BALLHELD;
			XTft_DrawTextBox(&TftInstance, 315, 230, 399, 249, "<BALLHELD>");

			// updates game_time at point of pause
			//curr_time = xget_clock_ticks();
			//game_time = game_time + curr_time - lastupdated_time;

			//draws ball with latest bar collided coordinates
			drawBall(&TftInstance, msg_ball_recd.ballx, msg_ball_recd.bally, &ball_drawn_x, &ball_drawn_y);


			//can only break out of loop if gamestateflag change to GAME_NORMAL
			while(gamestateflag == GAME_BALLHELD)
			{
				sleep(40);
			}


			XTft_DrawSolidBox(&TftInstance, 315, 230, 399, 249, GAMEAREA_COLOUR);

			//curr_time = xget_clock_ticks();
			//lastupdated_time = curr_time;

			msg_ball_recd.ballx = ball_drawn_x;

		}
		//cannot be at GAME_WIN as has been handled


		//GAME_NORMAL
		if(firstrun == 1)	//debug
		{
			lastupdated_time = xget_clock_ticks();
			firstrun = 0;
		}

		// start of brick collision handling
		if (msg_ball_recd.brickrow != 0 && msg_ball_recd.brickrow <= TOTAL_ROWS)
		{
			brickrow = msg_ball_recd.brickrow;
			brickcol = msg_ball_recd.brickcol;
			msg_temp.id = brickcol;	//ranges from 1-10
			tempmask = generatebitmask(brickrow);

			msg_temp.isRed = 0;	//assume column is not red first
			pthread_mutex_lock(&red_mutex);
			for(i = 0; i < MAXREDCOLUMNS; i++)
			{
				//check if the collided column is red
				if(redcol[i] == brickcol)
				{
					msg_temp.isRed = 1;
					break;
				}
			}
			pthread_mutex_unlock(&red_mutex);

			// update bricks[]
			pthread_mutex_lock(&brick_mutex);
			bricks[brickcol-1]	= tempmask & bricks[brickcol-1];
			msg_temp.status  = bricks[brickcol-1];
			pthread_mutex_unlock(&brick_mutex);


			if (msg_temp.status == 0)
			col_count--;

			// use msgq to redraw bricks
			//have to consider if brick disappear at the instant of the collision
			send(DRAWBRICK_Q, &msg_temp, sizeof(msg_col));

			//update brickleft
			game_brickleft--;

			//indicate ballheld =1

			k = ((msg_ball_recd.brickrow-1) * TOTAL_COLUMNS) + (msg_ball_recd.brickcol-1);
			for(i=0; i < NUM_CRYSTAL_BRICK; i++)
			{
				if(k == crystalbrick[i])
				{
					poweruphold = 1;
					poweruphold_timer = xget_clock_ticks();
					poweruplengthen = 1;
					poweruplengthen_timer = xget_clock_ticks();

					pthread_mutex_lock(&uart_mutex);
					xil_printf(" Crystal brick %d: %d\n", i, crystalbrick[i]);
					pthread_mutex_unlock(&uart_mutex);
					break;
				}
			}


			// update game_score
			if (msg_temp.isRed == 0)
			{
				game_score += 1;
			}
			else
			{
				game_score += 2;
			}

			if (game_score >= score_nextlevel)
			{
				// set next level benchmark
				score_nextlevel += 10;

				// change the red columns
				//pthread_mutex_lock(&uart_mutex);
				//xil_printf(" Columns about to change back \n");
				//pthread_mutex_unlock(&uart_mutex);
				sem_post(&sem_changeback);
				sem_post(&sem_changeback);

				// ball speed will be increased by ball thread after next mailbox msg
				// since updated by ball, will not update screen immediately

			}

			//GAME_WIN
			if(game_brickleft == 0)
			{
				gamestateflag = GAME_WIN;
				XTft_DrawTextBox(&TftInstance, 315, 230, 369, 249, "WIN!!!");

				//breaks only when player resets
				while(gamestateflag == GAME_WIN)
				{
					sleep(40);
				}
				//when it exits, gamestateflag == GAME_RESET

				//GAME_RESET
				resetHandler();
				// add in non-blocking receive to clear game_Q
				pthread_mutex_lock(&uart_mutex);
				xil_printf("game thread exiting..\n\n");
				pthread_mutex_unlock(&uart_mutex);
				pthread_exit(0);

			}

		}		// end of collision handling

		if(poweruphold == 1)
		{
			if(xget_clock_ticks() - poweruphold_timer > 1500)
			{
				poweruphold = 0;	//will off powerflag after 5000ms
			}
		}

		if(poweruplengthen == 1)
		{
			if(xget_clock_ticks() - poweruplengthen_timer > 1500)
			{
				poweruplengthen = 0;	//will off powerflag after 5000ms
			}
		}



		//send message
		msg_game_tosend.message_for = MESSAGE_FOR_BALL;
		pthread_mutex_lock (&cursor_mutex);
		msg_game_tosend.bar_position = cursor_curr;
		pthread_mutex_unlock (&cursor_mutex);
		msg_game_tosend.score = game_score;
		msg_game_tosend.game_status = gamestateflag;
		//ensuring message sizes to and fro game and ball applications are similar in size
		msg_game_tosend.poweruphold = poweruphold;
		msg_game_tosend.poweruplengthen = poweruplengthen;
		msg_game_tosend.ballheldx = ball_drawn_x;

		//send(GAME_Q, &msg_ball_tosend, sizeof(msg_ball));	//debug - can delete

		XMbox_WriteBlocking(&Mbox, &msg_game_tosend, sizeof(msg_game));
		//XMutex_Lock(&mutex, MUTEX_NUM);
		//xil_printf("-- Sucessfully send from GAME to BALL --\r\n");
		//XMutex_Unlock(&mutex, MUTEX_NUM);

		if ( xget_clock_ticks() - fps_prev> 100)
		{
			XTft_DrawNumberBox(&TftInstance, 600, 10, 639, 39, frame_count);
			fps_prev = xget_clock_ticks();
			frame_count = 0;
		}
		else
		{
			frame_count++;
		}

		//draw ball
		if (ball_drawn_x != msg_ball_recd.ballx || ball_drawn_y != msg_ball_recd.bally)
		{
			//debug can refactor this function
			drawBall(&TftInstance, msg_ball_recd.ballx, msg_ball_recd.bally, &ball_drawn_x, &ball_drawn_y);
		}

		//update score
		if(shown_score != game_score)
		{
			shown_score = game_score;
			XTft_DrawNumberBox(&TftInstance, 540, 100, 619, 160, shown_score);
		}
		//update speed
		if(ballspeed_shown != msg_ball_recd.speed)
		{
			ballspeed_shown = msg_ball_recd.speed;
			XTft_DrawNumberBox(&TftInstance, 535, 330, 629, 359, ballspeed_shown);
		}
		//update time
		curr_time = xget_clock_ticks();
		game_time = game_time + curr_time - lastupdated_time;
		lastupdated_time = curr_time;
		if(game_time/100 != shown_time)
		{
			shown_time = game_time/ 100;
			XTft_DrawNumberBox(&TftInstance, 535, 240, 629, 269, shown_time);
		}

		//update brickleft
		if(shown_brickleft != game_brickleft)
		{
			shown_brickleft = game_brickleft;
			XTft_DrawNumberBox(&TftInstance, 535, 420, 629, 449, shown_brickleft);
		}


		/*
		gamethread_timestamp2 = xget_clock_ticks();
		gamethread_timeinterval = gamethread_timestamp2 - gamethread_timestamp;
		pthread_mutex_lock(&uart_mutex);
		xil_printf("new ball displayed receiving packet_no %d, %d * 10 milliseconds after previous update \n\n", receive_packet_no, gamethread_timeinterval);
		pthread_mutex_unlock(&uart_mutex);
		gamethread_timestamp = xget_clock_ticks();

		receive_packet_no +=1;
		*/
		/*
		gamethread_finishtimestamp = xget_clock_ticks();
		gamethread_finishinterval = gamethread_finishtimestamp - gamethread_timestamp;
		pthread_mutex_lock(&uart_mutex);
		xil_printf("game thread finishes after %d * 10 milliseconds since current invocation\n\n", gamethread_finishinterval);
		pthread_mutex_unlock(&uart_mutex);
		*/


		//sleep(40);   //Try to see if this smoothens out the lag in any way //debug - is this still needed?

	}
}

// brick thread suppose to compute if ball hits,
// but not needed in our implementation
void* thread_bcol1()
{
	while (1)
	{
		tryRed2((unsigned int) 0);
		sleep(40);
	}
}
void* thread_bcol2()
{
	while (1)
	{
		tryRed2((unsigned int) 1);
		sleep(40);
	}
}

void* thread_reset()
{
	//kill off existing threads that don't suicide
	msg_col msg_temp;
	msg_temp.id = 1;
	msg_temp.status = 0xFF;
	msg_temp.isRed = 0;
	msg_game msg_game_tosend;

	msg_ball msg_ball_tosend;
	msg_ball_tosend.ballx = INITIAL_X;
	msg_ball_tosend.bally = INITIAL_Y;
	msg_ball_tosend.brickrow = 0;
	msg_ball_tosend.brickcol = 0;

	//bcol1 and bcol2 alive if col_count >2
	if (col_count > 2)
	{
		//by freeing these threads with sem_post, they will be able to suicide
		sem_post(&sem_changeback);
		sem_post(&sem_changeback);
	}

	//by freeing this thread with send, it will be able to suicide
	send(DRAWBRICK_Q, &msg_temp, sizeof(msg_col));

	//attempts to lock, then unlock all mutex
	pthread_mutex_trylock(&uart_mutex);
	pthread_mutex_unlock(&uart_mutex);

	pthread_mutex_trylock(&cursor_mutex);
	pthread_mutex_unlock(&cursor_mutex);

	pthread_mutex_trylock(&tft_mutex);
	pthread_mutex_unlock(&tft_mutex);

	pthread_mutex_trylock(&brick_mutex);
	pthread_mutex_unlock(&brick_mutex);

	pthread_mutex_trylock(&red_mutex);
	pthread_mutex_unlock(&red_mutex);

	//pthread_mutex_trylock(&gamestate_mutex);
	//pthread_mutex_unlock(&gamestate_mutex);

	//reset hwmutex debug

	//removed ball mutexes unlocking

	pthread_mutex_lock(&uart_mutex);
	xil_printf("mutex reset, reset thread sleeping...\n\n");
	pthread_mutex_unlock(&uart_mutex);

	sleep(400); //buffer time to let other threads die

	init_variables();
	init_screen();
	init_threads();	// may need to re-init semaphores

	sleep(100); //let threads be launched before resetting the sender

	//mb0 finished resetting
	//send message to reset mb1 - as mb0 has finished resetting

	pthread_mutex_lock(&uart_mutex);
	xil_printf("thread reset finished init..\n");
	xil_printf("sending mail to reset ball\n");
	pthread_mutex_unlock(&uart_mutex);

	msg_game_tosend.message_for = MESSAGE_FOR_BALL;
	msg_game_tosend.bar_position = 0;
	msg_game_tosend.score = 0;
	msg_game_tosend.game_status = GAME_RESET;
	msg_game_tosend.poweruphold = 0;
	msg_game_tosend.poweruplengthen = 0;
	msg_game_tosend.ballheldx = 0;
	XMbox_WriteBlocking(&Mbox, &msg_game_tosend, sizeof(msg_game));

	pthread_mutex_lock(&uart_mutex);
	xil_printf("thread reset about exiting..\n\n");
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
	XStatus Status_TFT;
	XMutex_Config *mutex_cfg;
	XMbox_Config *ConfigPtr;
	int Status_MB;

	print("-- Entering main_prog() uB0 RECEIVER--\r\n");

	// init TFT
	Status_TFT = Tft_init(TFT_DEVICE_ID);
	if ( Status_TFT != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//xil_printf("Initializing PB\r\n");
	// Initialise the PB instance
	Status_TFT = XGpio_Initialize(&gpPB, XPAR_GPIO_0_DEVICE_ID);
	// set PB gpio direction to input.
	XGpio_SetDataDirection(&gpPB, 1, 0x000000FF);

	//xil_printf("Enabling PB interrupts\r\n");
	//global enable
	XGpio_InterruptGlobalEnable(&gpPB);
	// interrupt enable. both global enable and this function should be called to enable gpio interrupts.
	XGpio_InterruptEnable(&gpPB, 1);
	//register the handler with xilkernel
	register_int_handler(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpPBIntHandler, &gpPB);
	//enable the interrupt in xilkernel
	enable_interrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);

	// initialize the SW Mutex here
	//display
	ret = pthread_mutex_init(&uart_mutex, NULL);
	//if (ret != 0)
	//xil_printf("-- ERROR (%d) init uart_mutex...\r\n", ret);
	// initialize Mutex
	ret = pthread_mutex_init (&cursor_mutex, NULL);
	//if (ret != 0)
	//xil_printf ("-- ERROR (%d) init cursor_mutex...\r\n", ret);
	ret = pthread_mutex_init (&tft_mutex, NULL);
	//if (ret != 0)
	//xil_printf ("-- ERROR (%d) init tft_mutex...\r\n", ret);
	ret = pthread_mutex_init (&brick_mutex, NULL);
	//if (ret != 0)
	//xil_printf ("-- ERROR (%d) init brick_mutex...\r\n", ret);
	ret = pthread_mutex_init (&red_mutex, NULL);
	//if (ret != 0)
	//xil_printf ("-- ERROR (%d) init red_mutex...\r\n", ret);
	//ret = pthread_mutex_init (&gamestate_mutex, NULL);
	//if (ret != 0)
	//xil_printf ("-- ERROR (%d) init gamestate_mutex...\r\n", ret);

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
	Status_MB = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status_MB != XST_SUCCESS) {
		print("-- Error initializing Mbox uB1 Sender--\r\n");
		return NULL;
	}


	// initialize the semaphore
	if( sem_init(&sem_red, 1, 2) < 0 )	// may need to re-init semaphores
	{
		//xil_printf("Error while initializing semaphore sem.\r\n");
	}
	if( sem_init(&sem_changeback, 1, 0) < 0 )	// may need to re-init semaphores
	{
		//xil_printf("Error while initializing semaphore sem22.\r\n");
	}

	//xil_printf("--Initialized -- uB0 \r\n");

	init_variables();
	init_screen();
	init_threads();	// may need to re-init semaphores
	return NULL;

}

void init_screen()
{
	int i;
	// Draw Initial Game Screen
	XTft_DrawSolidBox(&TftInstance, GAMEAREA_LEFT, GAMEAREA_TOP, GAMEAREA_RIGHT, GAMEAREA_BTM, GAMEAREA_COLOUR);
	// Draw the box labels
	XTft_DrawTextBox(&TftInstance, 555, 10, 589, 39, "fps");
	XTft_DrawTextBox(&TftInstance, 555, 60, 599, 79, "SCORE");
	XTft_DrawTextBox(&TftInstance, 555, 200, 599, 219, "TIME");
	XTft_DrawTextBox(&TftInstance, 555, 290, 599, 309, "SPEED");
	XTft_DrawTextBox(&TftInstance, 555, 380, 609, 399, "BRICKS");

	// Draw the initial values
	XTft_DrawNumberBox(&TftInstance, 600, 10, 639, 39, 0);
	XTft_DrawNumberBox(&TftInstance, 540, 100, 619, 160, 0);
	XTft_DrawNumberBox(&TftInstance, 535, 240, 629, 269, 0);
	XTft_DrawNumberBox(&TftInstance, 535, 330, 629, 359, INITIAL_BALLSPEED);
	XTft_DrawNumberBox(&TftInstance, 535, 420, 629, 449, 80);

	// Draw bricks
	for(i=1;i< TOTAL_COLUMNS+1 ;i++)
	{
		drawBCol(&TftInstance, i, 0xFF, 0);
	}

	// Initial ball
	XTft_DrawSolidCircle(&TftInstance, INITIAL_X, INITIAL_Y);

	// Initial bar
	drawBar(&TftInstance, cursor_curr, &cursor_curr, poweruplengthen);

	pthread_mutex_lock(&uart_mutex);
	xil_printf(" Finished init_screen.\r\n");
	pthread_mutex_unlock(&uart_mutex);
}
void init_variables()
{
	int i,j, random_num;

	//display
	clock_ticks_previrq = xget_clock_ticks();
	clock_ticks_buttonheld = xget_clock_ticks();
	cursor_curr = INITIAL_BAR;		// absolute position
	redcount = 0;
	redcol[0]= 0;
	redcol[1]= 0;
	redcolprev[0]= 0;
	redcolprev[1]= 0;
	val_prev = 0;
	col_count = TOTAL_COLUMNS;
	gamestateflag = GAME_NORMAL;
	ball_drawn_x = INITIAL_X;
	ball_drawn_y = INITIAL_Y;
	poweruplengthen = 0;

	srand(0); 	//fixed seed for easy debugging [debug]
	//srand(clock_ticks_previrq);		//random seed

	receive_packet_no = 1;

	//display
	for(i=0;i<TOTAL_COLUMNS;i++)
	{
		bricks[i]=0xFF;		//set bricks
	}

	// Generate crystalbrick
	for(i=0;i< NUM_CRYSTAL_BRICK ;i++)
	{
		random_num = rand()% (TOTAL_COLUMNS*TOTAL_ROWS);
		if(i>0)
		{
			for(j=0;j<i;j++)
			{
				 if (crystalbrick[j] == random_num)
				 {
					 random_num = rand()% (TOTAL_COLUMNS*TOTAL_ROWS);
					 j=-1;	//recheck from first crystalbrick
				 }
			}
		}
		//random_num don't match any exisitng crystalbrick
		crystalbrick[i] = random_num;
	}

	for(i=0;i< NUM_CRYSTAL_BRICK ;i++)
	{
		pthread_mutex_lock(&uart_mutex);
		xil_printf(" Crystal brick %d: %d\n", i, crystalbrick[i]);
		pthread_mutex_unlock(&uart_mutex);
	}

	pthread_mutex_lock(&uart_mutex);
	xil_printf(" Finished init_variables.\r\n");
	pthread_mutex_unlock(&uart_mutex);
}
void init_threads()
{
	int ret;
	// thread launches

	// may need to re-init semaphores

	//display
	ret = pthread_create(&tbar, NULL, (void*) thread_bar, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_bar...\r\n", ret);
	}
	else
	{
		xil_printf("thread_bar launched with ID %d \r\n", tbar);
	}

	ret = pthread_create(&tgame, NULL, (void*) thread_game, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_game...\r\n", ret);
	}
	else
	{
		xil_printf("thread_game launched with ID %d \r\n", tgame);
	}

	ret = pthread_create(&tdrawbrick, NULL, (void*) thread_drawbrick, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_drawbrick...\r\n", ret);
	}
	else
	{
		xil_printf("thread_drawbrick launched with ID %d \r\n", tdrawbrick);
	}


	ret = pthread_create(&tcol1, NULL, (void*) thread_bcol1, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_bcol1...\r\n", ret);
	}
	else
	{
		xil_printf("thread_bcol1 launched with ID %d \r\n", tcol1);
	}

	ret = pthread_create(&tcol2, NULL, (void*) thread_bcol2, NULL);
	if (ret != 0)
	{
		xil_printf("-- ERROR (%d) launching thread_bcol2...\r\n", ret);
	}
	else
	{
		xil_printf("thread_bcol2 launched with ID %d \r\n", tcol2);
	}



	pthread_mutex_lock(&uart_mutex);
	xil_printf(" Finished init_threads.\r\n");
	pthread_mutex_unlock(&uart_mutex);
}
int Tft_init(u32 TftDeviceId)
{
	int Status;
	XTft_Config *TftConfigPtr;

	/*
	* Get address of the XTft_Config structure for the given device id.
	*/
	TftConfigPtr = XTft_LookupConfig(TftDeviceId);
	if (TftConfigPtr == (XTft_Config *)NULL)
	{
		return XST_FAILURE;
	}

	/*
	* Initialize all the TftInstance members and fills the screen with
	* default background color.
	*/
	Status = XTft_CfgInitialize(&TftInstance, TftConfigPtr, TftConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	/*
	* Wait till Vsync(Video address latch) status bit is set before writing
	* the frame address into the Address Register. This ensures that the
	* current frame has been displayed and we can display a new frame of
	* data. Checking the Vsync state ensures that there is no data flicker
	* when displaying frames in real time though there is some delay due to
	* polling.
	*/
	while (XTft_GetVsyncStatus(&TftInstance) != XTFT_IESR_VADDRLATCH_STATUS_MASK);

	/*
	* Change the Video Memory Base Address from default value to
	* a valid Memory Address and clear the screen.
	*/
	XTft_SetFrameBaseAddr(&TftInstance, TFT_FRAME_ADDR);
	XTft_ClearScreen(&TftInstance);

	//xil_printf("Finish initializing TFT \r\n");

	XTft_SetColor(&TftInstance, 0, 0);
	XTft_ClearScreen(&TftInstance);

	return 0;
}


// 1 clock tick = 10 millisec
static void gpPBIntHandler(void *arg) //Should be very short (in time). In a practical program, don't print etc.
{
	unsigned char val;
	unsigned int clock_ticks_curr;
	int halfbarlength_local = HALFBARLENG;

	clock_ticks_curr = xget_clock_ticks();

	//pthread_mutex_lock (&uart_mutex);
	//xil_printf("irq status: %d \n",XGpio_InterruptGetStatus(&gpPB));
	//pthread_mutex_unlock (&uart_mutex);

	//clear the interrupt flag. if this is not done, gpio will keep interrupting the microblaze.--
	// --Possible to use (XGpio*)arg instead of &gpPB
	XGpio_InterruptClear(&gpPB,1);

	//pthread_mutex_lock (&uart_mutex);
	//xil_printf("After clear, irq status: %d \n",XGpio_InterruptGetStatus(&gpPB));
	//pthread_mutex_unlock (&uart_mutex);

	//Read the state of the push buttons.
	val = XGpio_DiscreteRead(&gpPB, 1);

	//pthread_mutex_lock (&uart_mutex);
	//xil_printf("PB event, val = %d \r\n", val);
	//pthread_mutex_unlock (&uart_mutex);

	val_prev = val;

	// if clock_ticks elapsed greater than threshold, we consider it a genuine input
	// de-bouncing logic
	if((clock_ticks_curr - clock_ticks_previrq ) > 5)
	{
		if(gamestateflag == GAME_NORMAL)
		{
			if(poweruplengthen == 1)
				halfbarlength_local = HALFBARLENG + HALFBARLENG;
			else
				halfbarlength_local = HALFBARLENG;

			if (val == BTN_LEFT) //left
			{
				cursor_curr = cursor_curr - 25;	//[debug] to check if end cases are set correctly
				if (cursor_curr < CURSOR_LEFTX + halfbarlength_local)
				cursor_curr = CURSOR_LEFTX + halfbarlength_local;	// set to leftmost possible position
				clock_ticks_buttonheld = xget_clock_ticks();
			}
			else if (val == BTN_RIGHT) //right
			{
				cursor_curr = cursor_curr + 25;
				if (cursor_curr > CURSOR_RIGHTX - halfbarlength_local)
				cursor_curr = CURSOR_RIGHTX - halfbarlength_local; 	//set to rightmost possible location
				clock_ticks_buttonheld = xget_clock_ticks();
			}
			else if (val == BTN_CENTER) //centre button
			{
				gamestateflag = GAME_PAUSE;	//pause game
			}
			else if (val == BTN_TOP) //top button
			{
				gamestateflag = GAME_RESET;
			}
	}
	else if ((gamestateflag == GAME_LOSE) || (gamestateflag == GAME_WIN))
	{
		if (val == BTN_TOP) //top button
		{
		gamestateflag = GAME_RESET;
		}
	}
	else if (gamestateflag == GAME_PAUSE)
	{
		///debug
		if (val == BTN_CENTER) //centre button
		{
			gamestateflag = GAME_NORMAL;	//resume game
		}
		if (val == BTN_TOP) //top button
		{
		gamestateflag = GAME_RESET;
		}
	}
	else if (gamestateflag == GAME_BALLHELD)
	{
		if (val == BTN_BTM) //btm button
		{
		gamestateflag = GAME_NORMAL;
		}
	}
	//gamestateflag == GAME_RESET



clock_ticks_previrq = xget_clock_ticks();
}

}

void send(int destid, void *msgptr, size_t msgsize) {

	int msgid;

	// ENTER YOUR CODE HERE TO SEND DATA TO THREAD DISPLAY
	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("PRODCON: Sending to Queueid: %d -- Start !\n", destid);
	//pthread_mutex_unlock(&uart_mutex);

	msgid = msgget(destid, IPC_CREAT);
	if (msgid == -1)
	{
		//pthread_mutex_lock(&uart_mutex);
		//xil_printf("PRODCON: ERROR while opening Message Queue: %d Errno: %d\n", destid, errno);
		//pthread_mutex_unlock(&uart_mutex);
		pthread_exit(&errno);
	}

	if (msgsnd(msgid, msgptr, msgsize, 0) < 0)
	{ // blocking send
		//pthread_mutex_lock(&uart_mutex);
		//xil_printf("PRODCON: msgsnd to Message Queue: %d ran into ERROR. Errno: %d. Halting..\n", destid, errno);
		//pthread_mutex_unlock(&uart_mutex);
		pthread_exit(&errno);
	}

	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("PRODCON: Successfully sent to Queueid: %d!\r\n", destid);
	//pthread_mutex_unlock(&uart_mutex);
}

void receive(int msgqid, void* msgptr, size_t sizeofmsg)
{
	int msgid;

	// wait for message from send
	//ENTER YOUR CODE HERE TO RECEIVE THE DATA FROM THREADS

	// read msg from msgqueue
	msgid = msgget(msgqid, IPC_CREAT);

	if (msgid == -1)
	{
		//pthread_mutex_lock(&uart_mutex);
		//xil_printf("PRODCON: ERROR while opening Message Queue: %d Errno: %d \r\n", msgqid, errno);
		//pthread_mutex_unlock(&uart_mutex);
		pthread_exit(&errno);
	}
	if (msgrcv(msgid, msgptr, sizeofmsg, 0, 0) != sizeofmsg )
	{ // blocking recv
		//pthread_mutex_lock(&uart_mutex);
		//xil_printf("PRODCON: msgrcv to Message Queue: %d ran into ERROR. Errno: %d. Halting...\r\n", msgqid, errno);
		//pthread_mutex_unlock(&uart_mutex);
		//       consret = errno;
		pthread_exit(&errno);
	}

	// print message from msg_col_recd
	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("Successfully received from Queueid: %d.\n", msgqid);
	//pthread_mutex_unlock(&uart_mutex);
}

//
//	Drawing Functions
//

void drawBar(XTft *Tft, int cursor, int* cursor_drawn_ptr, int poweruplengthen_last)
{
	int y1 = BAR_TOP;
	int y2 = BAR_BTM;
	int dbllength = HALFBARLENG + HALFBARLENG;
	/*
	if (cursor_curr > *cursor_drawn_ptr) 	//bar moved to the right
	{
	XTft_DrawSolidBox(Tft, *cursor_drawn_ptr-HALFBARLENG, y1, cursor-HALFBARLENG-1, y2, GAMEAREA_COLOUR);
	XTft_DrawSolidBox(Tft, *cursor_drawn_ptr+HALFBARLENG+1, y1, cursor+HALFBARLENG, y2, BAR_COLOUR);
}
else
{
XTft_DrawSolidBox(Tft, cursor+HALFBARLENG+1, y1, *cursor_drawn_ptr+HALFBARLENG, y2, GAMEAREA_COLOUR);
XTft_DrawSolidBox(Tft, cursor-HALFBARLENG, y1, *cursor_drawn_ptr-HALFBARLENG-1, y2, BAR_COLOUR);
}
*/
	//erase previous bar
	if(poweruplengthen_last == 0)
	{
		XTft_DrawSolidBox(Tft, *cursor_drawn_ptr-HALFBARLENG, y1, *cursor_drawn_ptr+HALFBARLENG-1, y2, GAMEAREA_COLOUR);
	}
	else
	{
		XTft_DrawSolidBox(Tft, *cursor_drawn_ptr-dbllength, y1, *cursor_drawn_ptr+dbllength-1, y2, GAMEAREA_COLOUR);
	}

if(poweruplengthen == 0)
{

	//draw multicolor bar
	XTft_DrawSolidBox(Tft, cursor-HALFBARLENG, y1, cursor-HALFBARLENG +9, y2, BAR_A_COLOUR);
	XTft_DrawSolidBox(Tft, cursor-HALFBARLENG+10, y1, cursor-HALFBARLENG+19, y2, BAR_S_COLOUR);
	XTft_DrawSolidBox(Tft, cursor-20, y1, cursor+19, y2, BAR_COLOUR);
	XTft_DrawSolidBox(Tft, cursor+HALFBARLENG-20, y1, cursor+HALFBARLENG-11, y2, BAR_S_COLOUR);
	XTft_DrawSolidBox(Tft, cursor+HALFBARLENG-10, y1, cursor+HALFBARLENG-1, y2, BAR_A_COLOUR);
}
else
{
	//draw multicolor bar
	XTft_DrawSolidBox(Tft, cursor-dbllength, y1, cursor-dbllength +19, y2, BAR_A_COLOUR);
	XTft_DrawSolidBox(Tft, cursor-dbllength+20, y1, cursor-dbllength+39, y2, BAR_S_COLOUR);
	XTft_DrawSolidBox(Tft, cursor-40, y1, cursor+39, y2, BAR_COLOUR);
	XTft_DrawSolidBox(Tft, cursor+dbllength-40, y1, cursor+dbllength-21, y2, BAR_S_COLOUR);
	XTft_DrawSolidBox(Tft, cursor+dbllength-20, y1, cursor+dbllength-1, y2, BAR_A_COLOUR);
}
*cursor_drawn_ptr = cursor;
//pthread_mutex_lock (&uart_mutex);
//xil_printf("  Bar updated to cursor value: %d.\n", *cursor_drawn_ptr);
//pthread_mutex_unlock (&uart_mutex);
}
//bcol_id ranges from 1-10
void drawBCol(XTft *Tft, int bcol_id, int bcol_status, int bcol_isRed)
{
	int i, j, k, bricktop, brickleft, bitmask;
	bricktop = GAMEAREA_TOP + 5;
	brickleft = GAMEAREA_LEFT + 5 +  ( INTERBRICK_X * (bcol_id - 1) );

	for (i = 0; i<8; i++)
	{
		bitmask = 1 << i;	//bitmask for different row

		if (bcol_status & bitmask)	// assert when status bit is true
		{
			if (bcol_isRed == 1)
				XTft_DrawSolidBox(Tft, brickleft, bricktop, brickleft + BRICK_LENGTH, bricktop + BRICK_HEIGHT, BRICK_COLOUR_RED);
			else
				XTft_DrawSolidBox(Tft, brickleft, bricktop, brickleft + BRICK_LENGTH, bricktop + BRICK_HEIGHT, BRICK_COLOUR);
			//if special brick
			k = (i*TOTAL_COLUMNS) + (bcol_id-1);
			for(j=0;j<NUM_CRYSTAL_BRICK;j++)
			{
				if(k == crystalbrick[j])
				{
					//draw special pattern
					XTft_DrawStripes(Tft, brickleft, bricktop, brickleft + BRICK_LENGTH, bricktop + BRICK_HEIGHT, BRICK_COLOUR_STRIPES);
					break;
				}
			}
		}
		else
		{
			XTft_DrawSolidBox(Tft, brickleft, bricktop, brickleft + BRICK_LENGTH, bricktop + BRICK_HEIGHT, GAMEAREA_COLOUR);
		}
		bricktop = bricktop + INTERBRICK_Y;
	}

	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("  Brick Column %d, Status %d updated on display!\n", bcol_id, bcol_status);
	//pthread_mutex_unlock(&uart_mutex);
}

void tryRed2(unsigned int redID) {

	int columnid = rand()%10 + 1;
	int otherred = (redID + 1) %2;
	int acceptflag = 0;

	msg_col msg_temp;

	sem_wait(&sem_red);

	while(acceptflag==0)
	{

		while( (columnid == redcolprev[0]) || (columnid == redcolprev[1]) || (columnid == redcol[otherred]) )
		{
			columnid = rand()%10 + 1;
		}
		pthread_mutex_lock(&brick_mutex);
		if(bricks[columnid-1]!=0)	//check this column is not empty
		{
			acceptflag =1;
		}
		else
		{
			columnid = rand()%10 + 1;
		}
		pthread_mutex_unlock(&brick_mutex);
	}

	//Changing Red
	pthread_mutex_lock (&red_mutex);
	redcol[redID] = columnid;
	pthread_mutex_unlock (&red_mutex);

	msg_temp.id = columnid;
	msg_temp.isRed = 1;
	pthread_mutex_lock(&brick_mutex);
	msg_temp.status = bricks[columnid-1];
	pthread_mutex_unlock(&brick_mutex);

	send(DRAWBRICK_Q, &msg_temp, sizeof(msg_col));

	//pthread_mutex_lock (&uart_mutex);
	//xil_printf ("  Column %d turned Red!!\n", columnid);
	//pthread_mutex_unlock (&uart_mutex);

	sem_wait(&sem_changeback);	//stall till the next time to change colour

	//for win/lose/reset/ballheld, thread will be block at this sem_wait

	if(gamestateflag == GAME_RESET)
	{
		sem_post(&sem_red);
		pthread_mutex_lock(&uart_mutex);
		xil_printf("bcol %d exiting..\n\n", redID);
		pthread_mutex_unlock(&uart_mutex);
		pthread_exit(0);
	}

	//stop changing the colors if there are 2 or fewer columns of bricks left
	//or kill thread if game needs to be reset
	if(col_count <= 2)
	{
		sem_post(&sem_red);
		pthread_exit(0);
	}
	else
	{
		//Changing Back
		pthread_mutex_lock (&red_mutex);
		redcol[redID] = 0;		//clear the value
		redcolprev[redID] = columnid;
		pthread_mutex_unlock (&red_mutex);

		msg_temp.isRed = 0;
		pthread_mutex_lock(&brick_mutex);
		msg_temp.status = bricks[columnid-1];
		pthread_mutex_unlock(&brick_mutex);

		send(DRAWBRICK_Q, &msg_temp, sizeof(msg_col));

		//pthread_mutex_lock (&uart_mutex);
		//xil_printf ("  Column %d turn back to Original Colour !!\n", columnid);
		//pthread_mutex_unlock (&uart_mutex);
		sem_post(&sem_red);
	}


}

void drawBall(XTft *Tft, int x, int y, int* ball_ptr_x, int* ball_ptr_y)
{
	int x1 = *ball_ptr_x - CIRCLE_RADIUS;
	int x2 = *ball_ptr_x + CIRCLE_RADIUS;
	int y1 = *ball_ptr_y - CIRCLE_RADIUS;
	int y2 = *ball_ptr_y + CIRCLE_RADIUS;

	//trim off the parts that is out of gamearea
	if (x1 < GAMEAREA_LEFT)
	x1 = GAMEAREA_LEFT;
	if (x2 > GAMEAREA_RIGHT)
	x2 = GAMEAREA_RIGHT;
	if (y1 < GAMEAREA_TOP)
	y1 = GAMEAREA_TOP;
	if (y2 > GAMEAREA_BTM)
	y2 = GAMEAREA_BTM;

	//Erase previous ball
	XTft_DrawSolidBox(Tft, x1, y1, x2, y2, GAMEAREA_COLOUR );

	//Draw new ball
	XTft_DrawSolidCircle(Tft, x, y);

	*ball_ptr_x = x;
	*ball_ptr_y = y;

	//pthread_mutex_lock (&uart_mutex);
	//xil_printf ("  Ball erased from %d, %d.!!\n", *ball_ptr_x, *ball_ptr_x);
	//xil_printf ("  Ball drawn at %d, %d.!!\n", x, y);
	//pthread_mutex_unlock (&uart_mutex);

}

// Draws a circle at centered at x, y
void XTft_DrawSolidCircle(XTft *Tft, int x, int y)
{
	int radius = CIRCLE_RADIUS;
	int i, j, height, pixel_x, pixel_y;

	pthread_mutex_lock(&uart_mutex);
	//xil_printf("  Ball to draw at  to: %d, %d!\n", x, y);
	pthread_mutex_unlock(&uart_mutex);

	pthread_mutex_lock(&tft_mutex);
	for(i=-radius; i<=radius; i++)
	{
		height = (int) ( sqrt(radius * radius - i * i) );
		pixel_x = i + x;

		for(j = -height; j<= height; j++)
		{
			pixel_y = j + y;
			//only draw the part of the ball that is within gamearea
			if((pixel_x >= GAMEAREA_LEFT) && (pixel_x <= GAMEAREA_RIGHT) && (pixel_y >= GAMEAREA_TOP) && (pixel_y <= GAMEAREA_BTM))
			XTft_SetPixel(Tft, pixel_x, pixel_y, BALL_COLOUR);
		}
	}
	pthread_mutex_unlock(&tft_mutex);

	return;
}

int XTft_DrawSolidBox(XTft *Tft, int x1, int y1, int x2, int y2, unsigned int col)
{
	int xmin,xmax,ymin,ymax,i,j;

	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("  Box drawn at  to: %d, %d, %d, %d!\n", x1, y1, x2, y2);
	//pthread_mutex_unlock(&uart_mutex);

	// ensures all 4 coords within screen and drawing from topleft to bottomright
	if (x1 >= 0 && x1 <= DISPLAY_COLUMNS-1 && x2 >= 0 && x2 <= DISPLAY_COLUMNS-1 && y1 >= 0 && y1 <= DISPLAY_ROWS-1 && y2 >= 0 && y2 <= DISPLAY_ROWS-1)
	{
		if (x2 < x1)
		{
			xmin = x2;
			xmax = x1;
		}
		else
		{
			xmin = x1;
			xmax = x2;
		}
		if (y2 < y1)
		{
			ymin = y2;
			ymax = y1;
		}
		else
		{
			ymin = y1;
			ymax = y2;
		}

		pthread_mutex_lock(&tft_mutex);
		for (i=xmin; i<=xmax; i++)
		{
			for (j=ymin; j<=ymax; j++)
			{
				XTft_SetPixel(Tft, i, j, col);
			}
		}
		pthread_mutex_unlock(&tft_mutex);

		return 0;	// returns 0 if draw is successful
	}

	return 1; 	// returns 1 if unable to draw

}

int XTft_DrawStripes(XTft *Tft, int x1, int y1, int x2, int y2, unsigned int col)
{
	int xmin,xmax,ymin,ymax,i,j;

	//pthread_mutex_lock(&uart_mutex);
	//xil_printf("  Box drawn at  to: %d, %d, %d, %d!\n", x1, y1, x2, y2);
	//pthread_mutex_unlock(&uart_mutex);

	// ensures all 4 coords within screen and drawing from topleft to bottomright
	if (x1 >= 0 && x1 <= DISPLAY_COLUMNS-1 && x2 >= 0 && x2 <= DISPLAY_COLUMNS-1 && y1 >= 0 && y1 <= DISPLAY_ROWS-1 && y2 >= 0 && y2 <= DISPLAY_ROWS-1)
	{
		if (x2 < x1)
		{
			xmin = x2;
			xmax = x1;
		}
		else
		{
			xmin = x1;
			xmax = x2;
		}
		if (y2 < y1)
		{
			ymin = y2;
			ymax = y1;
		}
		else
		{
			ymin = y1;
			ymax = y2;
		}

		pthread_mutex_lock(&tft_mutex);
		for (i=xmin; i<=xmax; i++)
		{
			for (j=ymin; j<=ymax; j++)
			{
				if (i%3 ==0 && j %3 ==1 )
					XTft_SetPixel(Tft, i, j, col);
				else if (i%3 == 1 && j %3 ==2 )
					XTft_SetPixel(Tft, i, j, col);
				else if (i%3 == 2 && j %3 ==0)
					XTft_SetPixel(Tft, i, j, col);

			}
		}
		pthread_mutex_unlock(&tft_mutex);

		return 0;	// returns 0 if draw is successful
	}

	return 1; 	// returns 1 if unable to draw

}

void XTft_DrawTextBox(XTft *Tft, int x1, int y1, int x2, int y2, char* cstringtext)
{
	char* txtptr = cstringtext;
	int cstringleng = 0;

	// Check cstringtext ends with a NULL
	while(*txtptr != '\0')
	{
		txtptr++;
		cstringleng++;
		if (cstringleng > 10)	//do not expect cstringtext to be longer than 9 chars
		return;
	}
	if (cstringleng==0) 	//stop printing if cstringtext is empty
	return;
	else 	// move txtptr back to start of cstringtext
	txtptr = cstringtext;

	//draw bgbox first - also help to erase previous info
	XTft_DrawSolidBox(Tft, x1, y1, x2, y2, TEXTBOX_COLOUR);

	// draw text after that
	pthread_mutex_lock(&tft_mutex);
	XTft_SetPos(Tft, x1, y1);
	XTft_SetPosChar(Tft, x1+3, y1+5);
	XTft_SetColor(Tft, TEXT_COLOUR, TEXTBOX_COLOUR);

	while(*txtptr != '\0')
	{
		XTft_Write(Tft, *txtptr);
		txtptr++;
	}
	pthread_mutex_unlock(&tft_mutex);

}

void XTft_DrawNumberBox(XTft *Tft, int x1, int y1, int x2, int y2, int someNumber)
{
	char txtbuffer[10] = "";
	numbertocstring(someNumber, txtbuffer);
	XTft_DrawTextBox(Tft, x1, y1, x2, y2, txtbuffer);
}

void numbertocstring(int num, char* charptr)
{
	int reversednum = 0;
	int digitcount = 0;

	if (num == 0)
	{
		*charptr = 48;
		charptr++;
		*charptr = '\0';
		return;
	}

	while (num != 0)
	{
		reversednum = reversednum * 10;
		reversednum = reversednum + num % 10;
		num = num / 10;
		digitcount++;
	}
	while(digitcount!=0)
	{
		*charptr = (reversednum % 10) + 48;  //convert digit to its corresponding ascii
		reversednum =  reversednum/ 10;
		charptr++;
		digitcount--;
	}
	*charptr = '\0';     // terminates with null
}

int generatebitmask(int brickrow)
{
	//debug
	// use
	//int bitmask = 1 << brickrow;	//bitshift
	//return !bitmask;	//invert
	switch(brickrow)
	{
		case 1:
		return 0xfe;
		case 2:
		return 0xfd;
		case 3:
		return 0xfb;
		case 4:
		return 0xf7;
		case 5:
		return 0xef;
		case 6:
		return 0xdf;
		case 7:
		return 0xbf;
		case 8:
		return 0x7f;
		default:
		return 0xff;
	}
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
