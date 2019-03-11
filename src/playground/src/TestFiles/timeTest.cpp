#include <iostream>
#include <chrono>
#include <ncurses.h>
#include <stdbool.h>
using namespace std;

#define THRESHOLD_IN_SECONDS 1 
#define THRESHOLD_MILLISEC 1

void idle();
void setIdleState(bool newState);
bool kbhit();

bool loop = true;
bool idleState= true;
bool skip= true;
int  ch;

chrono::time_point<chrono::steady_clock> zeroMark, timer;
chrono::duration<float> timeElapsedFloat;

int main(void){

	initscr();
	cbreak();
	nodelay(stdscr, TRUE);
	noecho();
	zeroMark= chrono::steady_clock::now();
	 while(loop){

 		timer= chrono::steady_clock::now();
		timeElapsedFloat= timer - zeroMark;		
		 if(!kbhit()  && (timeElapsedFloat.count() > THRESHOLD_MILLISEC))
		 {
			idle();
		 }
		 else{
			ch= getch(); 
			if(ch == ERR){
				qiflush();
			}
			else{
				switch(getch()){

					case 'X':
						refresh();
						printw("Exiting");
						loop= false;
					break;

					case 'W':
						move(0,0);
						printw("Forward");
						setIdleState(false);
						zeroMark= chrono::steady_clock::now();
					break;

					case 'S':
						move(10,10);
						printw("Reverse");
						setIdleState(false);
						zeroMark= chrono::steady_clock::now();
					break;

					default:
						move(0,0);
						printw("Not a key");
					break;
				}
			}
		 }
		

	 }

	endwin();
	return 0;
}

void idle(){
	if(idleState == false){
		printw("Motor stopped");
		setIdleState(true);
	}
}

void setIdleState(bool newState){
	idleState= newState;
}

bool kbhit(){
	return (ch = getch()) != ERR;
}
